"""模倣学習の手本を集める：茶碗の場所をくじで変えながら、今の MoveIt の動きで
「茶碗の上へ → まっすぐ下ろす → 指を閉じる」（お出しの5〜7の段階）を行い、rosbag に記録する。

記録するトピック：
  /joint_states          関節の今の角度（観測と、行動のもと）
  /kotoba/bowl_position  つかむ茶碗の場所（ロボットの座標。まわりの状態）
使い方（シミュレータを起動してから、/workspace で。サーバーは動かさない）：
  python3 -m kotoba_hakobu.collect_grasp_demos --episodes 2 --seed 2026     # 試し
  python3 -m kotoba_hakobu.collect_grasp_demos --episodes 60 --seed 2026    # 本番
"""
import argparse
import json
import math
import random
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from pymoveit2 import MoveIt2

from kotoba_hakobu import planning_scene as scene
from kotoba_hakobu.bowl_locator import BOWL_NAMES, TruePoseLocator
from kotoba_hakobu.eval_serve import PARK, SETTLE_SEC, X_MIN, set_pose
from kotoba_hakobu.try_grasp import hand_angle
from kotoba_hakobu.try_level_move import move
from kotoba_hakobu.try_pose_goal import set_gripper
from kotoba_hakobu.try_serve import (ARM_JOINTS, HOME, GRIPPER_OPEN_DEG, GRIPPER_CLOSE_DEG,
                                     AIM_FORWARD, AIM_LEFT, Z_ABOVE, Z_GRASP, wrist_for)

BOWL, OTHER = "red", "black"       # 手本では赤楽だけを使い、黒楽はよけておく
R_RANGE = (0.165, 0.18)            # 茶碗を置く、土台からの距離（m。評価と同じ）
DEG_RANGE = (-120.0, -50.0)        # 茶碗を置く向き（度。評価で黒楽・赤楽を置く範囲をまとめたもの）
GRASP_EMPTY_DEG = 34.0             # 指がここまで閉じたら、何もはさめていない（サーバーと同じ）
OUT_ROOT = Path("/workspace/il_data/rosbags")
CAM_RAW = "/chakai_camera/image"     # 天井カメラの元画像（640×480・約10Hz）
CAM_SMALL = "/kotoba/camera_small"   # 1/4 に縮めた画像（160×120）。元のまま記録すると大きすぎて /joint_states を取りこぼしたため
CAM_SCALE = 4                        # 縦横とも、4画素ごとに1画素を取る
TOPICS = ["/joint_states", "/kotoba/bowl_position", CAM_SMALL]
BAG_START_SEC = 1.5                # 記録が始まるまで待つ時間（秒）
AFTER_CLOSE_SEC = 1.0              # 指を閉じたあと、記録を続ける時間（秒）
PLACE_TOL = 0.005                  # 置き直したあと、ねらった場所からこれ以上ずれていたら置き直す（m。評価と同じ）
PLACE_TRIES = 3                    # 置き直しを試す回数


class Collector(Node):
    """腕と指を動かし、つかむ茶碗の場所を 20Hz で流すノード。"""

    def __init__(self):
        super().__init__("collect_grasp_demos")
        group = ReentrantCallbackGroup()
        self.arm = MoveIt2(node=self, joint_names=ARM_JOINTS,
                           base_link_name="crane_plus_base", end_effector_name="crane_plus_link4",
                           group_name="arm", callback_group=group)
        self.gripper = MoveIt2(node=self, joint_names=["crane_plus_joint_hand"],
                               base_link_name="crane_plus_link4", end_effector_name="crane_plus_link_hand",
                               group_name="gripper", callback_group=group)
        self.arm.allowed_planning_time = 2.0
        for m in (self.arm, self.gripper):
            m.max_velocity = 0.3
            m.max_acceleration = 0.3
        self.bowl_pub = self.create_publisher(PointStamped, "/kotoba/bowl_position", 10)
        self.bowl_xy = None
        self.create_timer(0.05, self.publish_bowl, callback_group=group)
        # 天井カメラの画像を 1/4 に縮めて流し直す（記録するのは縮めた方）
        self.cam_pub = self.create_publisher(Image, CAM_SMALL, 10)
        self.create_subscription(Image, CAM_RAW, self.shrink_image, qos_profile_sensor_data,
                                 callback_group=group)

    def publish_bowl(self):
        """記録している間だけ、つかむ茶碗の場所を流す（1秒間に20回）。"""
        if self.bowl_xy is None:
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "crane_plus_base"
        msg.point.x, msg.point.y = self.bowl_xy
        msg.point.z = 0.02
        self.bowl_pub.publish(msg)

    def shrink_image(self, msg):
        """640×480 の画像を 160×120 に縮めて流し直す（4画素ごとに1画素を取る）。"""
        if msg.encoding != "rgb8":
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        small = arr[::CAM_SCALE, ::CAM_SCALE].copy()
        out = Image()
        out.header = msg.header
        out.height, out.width = small.shape[0], small.shape[1]
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = small.shape[1] * 3
        out.data = small.tobytes()
        self.cam_pub.publish(out)

    def go_home(self):
        self.arm.move_to_configuration(HOME)
        self.arm.wait_until_executed()


def pick_place(rng):
    """茶碗を置く場所をくじで決める。机の端に近すぎたら引き直す。"""
    while True:
        r = rng.uniform(*R_RANGE)
        d = math.radians(rng.uniform(*DEG_RANGE))
        x, y = r * math.cos(d), r * math.sin(d)
        if x >= X_MIN:
            return x, y


def place_bowl(locator, x, y):
    """黒楽をよけておき、赤楽を (x, y) に置く。本当の場所を読んで確かめ、ずれていたら置き直す。
    置けたら本当の場所（辞書）を返す。PLACE_TRIES 回やっても置けなければ None を返す。"""
    for _ in range(PLACE_TRIES):
        set_pose(BOWL_NAMES[OTHER], *PARK[OTHER])
        set_pose(BOWL_NAMES[BOWL], x, y)
        time.sleep(SETTLE_SEC)
        bowl = locator.locate(BOWL)
        if bowl and math.hypot(bowl["x"] - x, bowl["y"] - y) <= PLACE_TOL \
                and abs(bowl["z"] - 0.02) <= 0.01 and bowl["tilt_deg"] <= 5.0:
            return bowl
    return None


def run_episode(node, locator, x, y, bag_dir, aim_left, jitter, jrng):
    """手本を1回とる。結果の辞書を返す。
    jitter > 0 のときは、わざとずれた所（安全な高さ Z_ABOVE）へ行ってから、
    正しい位置へ立て直す動きも記録に入れる（立て直しの手本）。"""
    bowl = place_bowl(locator, x, y)
    if bowl is None:
        return {"grasped": False, "reason": "茶碗を置き直せなかった", "bowl": None}
    node.go_home()
    set_gripper(node, node.gripper, GRIPPER_OPEN_DEG)
    gx, gy, _ = wrist_for(bowl["x"], bowl["y"], AIM_FORWARD, aim_left)

    # ----- ここから記録（お出しの5〜7の段階と同じ動き） -----
    node.bowl_xy = (bowl["x"], bowl["y"])
    bag = subprocess.Popen(["ros2", "bag", "record", "-o", str(bag_dir), *TOPICS],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(BAG_START_SEC)
    jit = 0.0
    if jitter > 0:
        # わざとずれた所へ（ずらすのは安全な高さの階だけ。半径は jitter の3割〜10割、向きはくじ）
        jit = jrng.uniform(jitter * 0.3, jitter)
        th = jrng.uniform(0, 2 * math.pi)
        jx, jy = gx + jit * math.cos(th), gy + jit * math.sin(th)
        moved = move(node, node.arm, (jx, jy, Z_ABOVE))[0]
        time.sleep(0.5)  # ずれた画像をカメラに写す間
        moved = moved and move(node, node.arm, (gx, gy, Z_ABOVE))[0]  # 立て直し（ふつうの計画。4関節の腕は横のまっすぐ移動が作れない）
        time.sleep(0.5)
    else:
        moved = move(node, node.arm, (gx, gy, Z_ABOVE))[0]
    moved = moved and move(node, node.arm, (gx, gy, Z_GRASP), True)[0]
    if moved:
        set_gripper(node, node.gripper, GRIPPER_CLOSE_DEG)
    time.sleep(AFTER_CLOSE_SEC)
    bag.send_signal(signal.SIGINT)
    bag.wait(timeout=15)
    node.bowl_xy = None
    # ----- 記録はここまで -----

    angle = hand_angle(node.gripper)
    grasped = moved and angle is not None and angle < GRASP_EMPTY_DEG
    # 次の手本のために、指を開いて上へ離れ、HOME に戻る
    set_gripper(node, node.gripper, GRIPPER_OPEN_DEG)
    move(node, node.arm, (gx, gy, Z_ABOVE), True)
    node.go_home()
    return {"grasped": grasped, "reason": "" if grasped else ("腕の動きの失敗" if not moved else "つかめていない"),
            "bowl": {"x": bowl["x"], "y": bowl["y"]}, "hand_deg": angle,
            "aim_left": aim_left, "jitter_m": round(jit, 4)}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--aim-left", type=float, default=AIM_LEFT,
                    help="下ろすときの、茶碗の中心の左寄せ（m）。既定は本番と同じ")
    ap.add_argument("--jitter", type=float, default=0.0,
                    help="わざとずらす大きさの上限（m）。0なら今までどおり。例：0.02")
    args = ap.parse_args()

    rclpy.init()
    node = Collector()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(2.0)  # MoveIt とつながるのを待つ

    run_dir = OUT_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    (run_dir / "ok").mkdir(parents=True)
    (run_dir / "failed").mkdir()
    log_path = run_dir / "episodes.jsonl"
    rng = random.Random(args.seed)
    jrng = random.Random(args.seed + 1)  # ずらし専用のくじ（茶碗を置くくじと混ざらないように別にする）
    locator = TruePoseLocator()
    n_ok = 0
    print(f"ねらいの左寄せ aim_left={args.aim_left}（本番の既定は {AIM_LEFT}）・ずらし jitter={args.jitter}")
    try:
        scene.clear(node.arm)  # MoveIt に教えた物が残っていれば消す
        for i in range(args.episodes):
            x, y = pick_place(rng)
            tmp = run_dir / f"tmp_{i:03d}"
            start = time.time()
            res = run_episode(node, locator, x, y, tmp, args.aim_left, args.jitter, jrng)
            dest = run_dir / ("ok" if res["grasped"] else "failed") / f"ep_{i:03d}"
            if tmp.exists():
                tmp.rename(dest)
            n_ok += res["grasped"]
            res.update({"episode": i, "place": {"x": x, "y": y}, "seconds": round(time.time() - start, 1),
                        "bag": str(dest)})
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(res, ensure_ascii=False) + "\n")
            print(f"[{i + 1}/{args.episodes}] {'つかめた' if res['grasped'] else '失敗：' + res['reason']}"
                  f"（茶碗 x={x:.3f} y={y:.3f}・{res['seconds']}秒）")
    finally:
        print(f"つかめた手本 {n_ok}/{args.episodes}。記録：{run_dir}")
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()