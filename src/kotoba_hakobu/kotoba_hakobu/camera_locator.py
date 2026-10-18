"""茶碗の位置を教える部品（カメラ版）：天井のカメラの画像から色で茶碗を見つけ、机の上の位置を求める"""
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

# ===== 天井のカメラ（chakai.sdf と camera_info の値） =====
CAM_X, CAM_Y, CAM_Z = 0.10, 0.0, 0.55   # カメラの位置（ロボットの座標。z は机からの高さ、m）
FOCAL = 467.74                          # 焦点距離（ピクセル）
CX, CY = 320.0, 240.0                   # 画像の中心（ピクセル）
IMAGE_TOPIC = "/chakai_camera/image"
FRESH_TIMEOUT = 3.0                     # 新しい画像を待つ時間の上限（秒）

# ===== 茶碗 =====
BOWL_RADIUS, BOWL_HEIGHT = 0.03, 0.04
TOP_H = CAM_Z - BOWL_HEIGHT             # カメラから茶碗の上の面までの高さ（0.51m）
TOP_R_PX = BOWL_RADIUS * FOCAL / TOP_H  # 上の面が画像に映る半径（約27.5ピクセル）
TOP_AREA_PX = math.pi * TOP_R_PX ** 2   # 上の面が画像に映る面積（約2380ピクセル）

# ===== 色の決まり（10/18 に measure_color で測った値から決めた。H:0〜179、S・V:0〜255） =====
# 測った値：机 H14 S134〜146 V95〜101／黒楽の上の面 S0 V54／赤楽の上の面 H0 S152 V191／
#           席 H23 S60 V214／ロボットの土台 S0 V0
COLOR_RULES = {
    "black": [((0, 0, 30), (179, 40, 90))],
    "red": [((0, 100, 130), (6, 255, 255)), ((174, 100, 130), (179, 255, 255))],
}
AREA_MIN, AREA_MAX = 0.6 * TOP_AREA_PX, 1.6 * TOP_AREA_PX   # かたまりの大きさの範囲
BASE_EXCLUDE_R = 0.12     # 土台からこの距離（m）以内は探さない（ロボットと間違えないため）

# ===== 立っているかの決まり（2つとも満たせば立っている。迷うときは倒れているとする） =====
# 10/18 に測った値：立っている 縦横比1.00・1.17／面積0.997倍・1.067倍、倒れている 縦横比1.09／面積0.74倍。
# 倒れた茶碗は真上から 57×38 ピクセルの長四角に見えるので、縦横比1.25未満なら面積は最大でも約0.76倍。
UPRIGHT_ASPECT = 1.25     # 縦横の比がこれより小さい（丸に近い）
UPRIGHT_AREA = 0.85       # 面積が、上の面の面積（TOP_AREA_PX）のこの倍率以上


def to_table(u, v):
    """画像の位置（u：横, v：縦）→ 茶碗の上の面の高さでの、机の上の位置（x, y）。"""
    x = CAM_X - TOP_H * (v - CY) / FOCAL
    y = CAM_Y - TOP_H * (u - CX) / FOCAL
    return x, y


class CameraLocator:
    """locate(色) で茶碗の位置を返す。真の値版（TruePoseLocator）と同じ形で返す。"""

    def __init__(self, node):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image, self.received = None, 0.0
        node.create_subscription(Image, IMAGE_TOPIC, self.on_image, 1)

    def on_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self.lock:
            self.image, self.received = image, time.time()

    def fresh_image(self):
        """呼ばれたあとに届いた画像を返す（古い画像は使わない）。届かなければ None。"""
        asked = time.time()
        while time.time() - asked < FRESH_TIMEOUT:
            with self.lock:
                if self.image is not None and self.received > asked:
                    return self.image
            time.sleep(0.02)
        return None

    def find(self, image, color):
        """色の決まりに合うかたまりのうち、大きさと場所の条件に合う最も大きいものを返す。"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for low, high in COLOR_RULES[color]:
            mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if not AREA_MIN <= area <= AREA_MAX:
                continue
            m = cv2.moments(contour)
            u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
            x, y = to_table(u, v)
            if math.hypot(x, y) < BASE_EXCLUDE_R:
                continue
            _, (w, h), _ = cv2.minAreaRect(contour)
            aspect = max(w, h) / max(min(w, h), 1e-6)
            if best is None or area > best["area_px"]:
                best = {"u": u, "v": v, "x": x, "y": y, "area_px": area,
                        "area_ratio": area / TOP_AREA_PX, "aspect": aspect}
        return best

    def locate(self, color):
        """茶碗の位置 {x, y, z, tilt_deg, ...} を返す。見つからなければ None。"""
        image = self.fresh_image()
        if image is None:
            return None
        found = self.find(image, color)
        if found is None:
            return None
        upright = found["aspect"] < UPRIGHT_ASPECT and found["area_ratio"] >= UPRIGHT_AREA
        found["z"] = BOWL_HEIGHT / 2 if upright else BOWL_RADIUS
        found["tilt_deg"] = 0.0 if upright else 90.0
        return found


def main():
    """比べる道具：カメラ版と真の値版の位置を並べ、ずれ（mm）を表示する。"""
    from kotoba_hakobu.bowl_locator import TruePoseLocator
    rclpy.init()
    node = Node("compare_locators")
    camera = CameraLocator(node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    truth = TruePoseLocator()
    for color, name_ja in (("black", "黒楽"), ("red", "赤楽")):
        c, t = camera.locate(color), truth.locate(color)
        if c is None:
            print(f"{name_ja}：カメラでは見つかりません")
            continue
        state = "立っている" if c["tilt_deg"] == 0.0 else "倒れている"
        line = (f"{name_ja}：カメラ x={c['x']:.4f} y={c['y']:.4f}（{state}・縦横比 {c['aspect']:.2f}・"
                f"面積 {c['area_px']:.0f}px＝{c['area_ratio']:.2f}倍）")
        if t is not None:
            error_mm = math.hypot(c["x"] - t["x"], c["y"] - t["y"]) * 1000
            line += f"\n　　　本当   x={t['x']:.4f} y={t['y']:.4f}（傾き {t['tilt_deg']:.1f}度）　ずれ {error_mm:.1f}mm"
        print(line)
    executor.shutdown()        # 先に受信係を止める
    thread.join()
    node.destroy_node()
    rclpy.shutdown()           # そのあとで ROS を止める


if __name__ == "__main__":
    main()