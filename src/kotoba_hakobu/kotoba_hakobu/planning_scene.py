"""MoveIt に机と茶碗を教える（ぶつかる物として、計画の世界に入れる）。

試し方（シミュレータを起動してから、/workspace で）：
  python3 -m kotoba_hakobu.planning_scene          # 机と茶碗2つを教える
  python3 -m kotoba_hakobu.planning_scene --clear  # 教えた物を消す
"""
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2

from kotoba_hakobu.serve_server import ARM_JOINTS

FRAME = "crane_plus_base"   # MoveIt の座標（机の天板の高さが z = 0）
GAZEBO_Z = 1.015            # Gazebo の z からこれを引くと、MoveIt の z になる
NO_TURN = (0.0, 0.0, 0.0, 1.0)  # 回さずに置く（向きを変えない）

# 机の天板：Fuel の Table（1.5×0.8×0.03、真ん中は床から1.0m）を90度回して x=0.29 に置いている。
# 土台が天板にふれているのを「ぶつかっている」と考えさせないよう、2mm 下げて教える。
TABLE_ID = "table_top"
TABLE_SIZE = (0.8, 1.5, 0.03)      # 前後・左右・厚さ（90度回したあと）
TABLE_POS = (0.29, 0.0, -0.017)

# 茶碗（chakai.sdf と同じ：半径3cm・高さ4cm）
BOWL_RADIUS = 0.03
BOWL_HEIGHT = 0.04
BOWL_IDS = {"red": "chawan_red", "black": "chawan_black"}

# 試しに使う茶碗の場所（chakai.sdf のはじめの場所。z は机に置いたときの真ん中）
TEST_BOWLS = {"red": (-0.0453, -0.1690, 0.02), "black": (0.0740, -0.1586, 0.02)}


def add_table(arm):
    """机の天板を教える。"""
    arm.add_collision_box(id=TABLE_ID, size=TABLE_SIZE, position=TABLE_POS,
                          quat_xyzw=NO_TURN, frame_id=FRAME)


def add_bowl(arm, bowl, position):
    """茶碗を1つ教える。position は MoveIt の座標（x, y, 真ん中の z）。"""
    arm.add_collision_cylinder(id=BOWL_IDS[bowl], height=BOWL_HEIGHT, radius=BOWL_RADIUS,
                               position=position, quat_xyzw=NO_TURN, frame_id=FRAME)


def clear(arm):
    """教えた机と茶碗を消す。"""
    for object_id in [TABLE_ID, *BOWL_IDS.values()]:
        arm.remove_collision_object(object_id)


def main():
    rclpy.init()
    node = rclpy.create_node("planning_scene_test")
    arm = MoveIt2(node=node, joint_names=ARM_JOINTS, base_link_name=FRAME,
                  end_effector_name="crane_plus_link4", group_name="arm",
                  callback_group=ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)  # MoveIt とつながるのを待つ

    if "--clear" in sys.argv:
        clear(arm)
        print("教えた机と茶碗を消しました。")
    else:
        add_table(arm)
        for bowl, position in TEST_BOWLS.items():
            add_bowl(arm, bowl, position)
        print("机と茶碗2つを MoveIt に教えました。RViz で確かめてください。")

    time.sleep(1.0)  # 送り終わるのを待つ
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()