"""色を測る道具：天井のカメラの画像を1枚受け取り、茶碗・影・机・席・ロボットの色（HSV）を測る"""
import math
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from kotoba_hakobu.bowl_locator import TruePoseLocator

# ===== 天井のカメラ（chakai.sdf に書いた値と、camera_info で確かめた値） =====
CAM_X, CAM_Y, CAM_Z = 0.10, 0.0, 0.55   # カメラの位置（ロボットの座標。z は机からの高さ、m）
FOCAL = 467.74                          # 焦点距離（ピクセル）
CX, CY = 320.0, 240.0                   # 画像の中心（ピクセル）
BOWL_TOP_Z = 0.04                       # 茶碗の上の面の高さ（m）
RING_R = 0.055                          # 茶碗のまわりを測る円の半径（m）
IMAGE_TOPIC = "/chakai_camera/image"
OUT_DIR = "/workspace/results"


def to_pixel(x, y, z):
    """机の上の位置（x, y, 高さ z）が、画像のどこ（u：横, v：縦）に映るかを計算する。
    画像の上 = ロボットの前（x）、画像の右 = ロボットの右（y のマイナス）。"""
    h = CAM_Z - z
    u = CX - (y - CAM_Y) * FOCAL / h
    v = CY - (x - CAM_X) * FOCAL / h
    return int(round(u)), int(round(v))


def patch_hsv(hsv, u, v, half=2):
    """(u, v) のまわり 5×5 ピクセルの色（H, S, V）の中央値。画像の外なら None。"""
    height, width = hsv.shape[:2]
    if not (half <= u < width - half and half <= v < height - half):
        return None
    patch = hsv[v - half:v + half + 1, u - half:u + half + 1].reshape(-1, 3)
    return np.median(patch, axis=0)


class ColorMeter(Node):
    """カメラの画像を受け取るだけのノード。"""

    def __init__(self):
        super().__init__("measure_color")
        self.bridge = CvBridge()
        self.image = None
        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, 1)

    def on_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main():
    rclpy.init()
    node = ColorMeter()
    start = time.time()
    while node.image is None and time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    image = node.image
    node.destroy_node()
    rclpy.shutdown()
    if image is None:
        print("画像が届きません。橋渡し（parameter_bridge）が動いているか確かめてください")
        return

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    locator = TruePoseLocator()
    points = []   # （名前, x, y, 高さ）
    for color, name_ja in (("black", "黒楽"), ("red", "赤楽")):
        bowl = locator.locate(color)
        if bowl is None:
            print(f"{name_ja}の本当の位置が読めません")
            continue
        points.append((f"{name_ja} 上の面", bowl["x"], bowl["y"], BOWL_TOP_Z))
        for k in range(8):                      # 0°＝前、90°＝左、180°＝後ろ、270°＝右
            a = math.radians(45 * k)
            points.append((f"{name_ja} まわり{45 * k:3d}°",
                           bowl["x"] + RING_R * math.cos(a), bowl["y"] + RING_R * math.sin(a), 0.0))
    points += [("机（右前）", 0.30, -0.25, 0.0), ("机（左）", 0.05, 0.25, 0.0),
               ("席（お次客様）", 0.24, 0.0, 0.0), ("ロボットの土台", 0.0, 0.0, 0.03)]

    print(f"{'測った場所':<16}{'u':>5}{'v':>5}{'H':>6}{'S':>6}{'V':>6}")
    marked = image.copy()
    for name, x, y, z in points:
        u, v = to_pixel(x, y, z)
        c = patch_hsv(hsv, u, v)
        if c is None:
            print(f"{name:<16}  画像の外")
            continue
        print(f"{name:<16}{u:5d}{v:5d}{c[0]:6.0f}{c[1]:6.0f}{c[2]:6.0f}")
        cv2.circle(marked, (u, v), 3, (0, 255, 0), -1)

    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    cv2.imwrite(f"{OUT_DIR}/camera_{stamp}.png", image)
    cv2.imwrite(f"{OUT_DIR}/camera_{stamp}_marked.png", marked)
    print(f"画像を保存しました：{OUT_DIR}/camera_{stamp}.png（緑の点を付けたものは _marked）")


if __name__ == "__main__":
    main()