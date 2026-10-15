"""届く範囲の地図：手先の位置と向きを少しずつ変えて「計画だけ」を試し、成功した回数を表にする"""
import csv
import math
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from pymoveit2 import MoveIt2

from kotoba_hakobu.try_pose_goal import quat_from_rpy

ARM_JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
POSITION_TOLERANCE = 0.005
ORIENTATION_TOLERANCE = 0.001
PLANNING_TIME = 2.0                                          # 1回の計画に使ってよい時間（秒）。初期値は 0.5
TRIES = 3                                                    # 1か所につき計画を試す回数
R_VALUES = [round(0.06 + 0.02 * i, 2) for i in range(6)]    # 前への距離 0.06〜0.16 m
Z_VALUES = [round(0.02 + 0.02 * i, 2) for i in range(8)]    # 高さ 0.02〜0.16 m
PITCHES = {"水平": 90.0, "下向き": 180.0}
RESULT_DIR = "/workspace/results"


def count_reach(arm, position, pitch_deg, tries=TRIES):
    """計画だけを tries 回試し、成功した回数を返す。腕は動かさない。"""
    yaw = math.atan2(position[1], position[0])
    quat = quat_from_rpy(0.0, math.radians(pitch_deg), yaw)
    successes = 0
    for _ in range(tries):
        trajectory = arm.plan(position=position, quat_xyzw=quat,
                              tolerance_position=POSITION_TOLERANCE,
                              tolerance_orientation=ORIENTATION_TOLERANCE)
        if trajectory is not None:
            successes += 1
    return successes


def main():
    rclpy.init()
    node = Node("check_reach")
    node.get_logger().set_level(LoggingSeverity.ERROR)   # 「計画に失敗」の警告を出さない
    arm = MoveIt2(node=node, joint_names=ARM_JOINTS,
                  base_link_name="crane_plus_base", end_effector_name="crane_plus_link4",
                  group_name="arm", callback_group=ReentrantCallbackGroup())
    arm.allowed_planning_time = PLANNING_TIME
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)

    rows = []
    for name, pitch in PITCHES.items():
        print(f"\n=== {name}（ピッチ {pitch:.0f}度・計画時間 {PLANNING_TIME}秒）"
              f"  数字={TRIES}回中の成功回数  .=0回 ===")
        print("z＼r " + "".join(f"{r:5.2f}" for r in R_VALUES))
        total = 0
        for z in reversed(Z_VALUES):        # 高い方から表示する
            marks = ""
            for r in R_VALUES:
                n = count_reach(arm, (r, 0.0, z), pitch)
                total += n
                rows.append({"pitch": pitch, "r": r, "z": z, "success": n,
                             "tries": TRIES, "planning_time": PLANNING_TIME})
                marks += f"{n if n > 0 else '.':>5}"
            print(f"{z:4.2f} {marks}")
        all_tries = TRIES * len(R_VALUES) * len(Z_VALUES)
        print(f"成功率: {total} / {all_tries} ＝ {total / all_tries:.0%}")

    os.makedirs(RESULT_DIR, exist_ok=True)
    path = f"{RESULT_DIR}/reach_map_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["pitch", "r", "z", "success", "tries", "planning_time"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n保存しました: {path}")

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()