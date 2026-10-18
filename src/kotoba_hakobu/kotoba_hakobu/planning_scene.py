"""MoveIt に机と茶碗を教える（ぶつかる物として、計画の世界に入れる）。

試し方（シミュレータを起動してから、/workspace で）：
  python3 -m kotoba_hakobu.planning_scene          # 机と茶碗2つを教える
  python3 -m kotoba_hakobu.planning_scene --clear  # 教えた物を消す
サーバーからは、reset_scene・remove_bowl・hold_bowl・release_bowl・place_bowl を使う。
"""
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2

from kotoba_hakobu.try_serve import ARM_JOINTS

FRAME = "crane_plus_base"   # MoveIt の座標（机の天板の高さが z = 0）
GAZEBO_Z = 1.015            # Gazebo の z からこれを引くと、MoveIt の z になる
NO_TURN = (0.0, 0.0, 0.0, 1.0)  # 回さずに置く（向きを変えない）
SCENE_WAIT = 0.3            # MoveIt が受け取って反映するのを待つ秒数

# 机の天板：Fuel の Table（1.5×0.8×0.03、真ん中は床から1.0m）を90度回して x=0.29 に置いている。
# 土台が天板にふれているのを「ぶつかっている」と考えさせないよう、2mm 下げて教える。
TABLE_ID = "table_top"
TABLE_SIZE = (0.8, 1.5, 0.03)      # 前後・左右・厚さ（90度回したあと）
TABLE_POS = (0.29, 0.0, -0.017)

# 茶碗（chakai.sdf と同じ：半径3cm・高さ4cm）
BOWL_RADIUS = 0.03
BOWL_HEIGHT = 0.04
BOWL_Z = 0.02               # 机に置いた茶碗の真ん中の高さ
BOWL_IDS = {"red": "chawan_red", "black": "chawan_black"}
PAD_SIDE = 0.005            # 待っている茶碗は、本物より半径を5mm大きく教える（よけるときの余裕）
PAD_TOP = 0.01              # 待っている茶碗は、本物より1cm高く教える（底は机の上のまま）

# 茶碗を手に持たせるとき：持たせる部品と、茶碗に触れてもよい部品（2本の指）
HAND_LINK = "crane_plus_link4"
TOUCH_LINKS = ["crane_plus_link4", "crane_plus_link_hand"]

# 試しに使う茶碗の場所（chakai.sdf のはじめの場所。x, y）
TEST_BOWLS = {"red": (-0.0453, -0.1690), "black": (0.0740, -0.1586)}


def add_table(arm):
    """机の天板を教える。"""
    arm.add_collision_box(id=TABLE_ID, size=TABLE_SIZE, position=TABLE_POS,
                          quat_xyzw=NO_TURN, frame_id=FRAME)


def add_bowl(arm, bowl, position, padded=True):
    """茶碗を1つ教える。position は MoveIt の座標（x, y, 本物の真ん中の z）。
    padded=True なら、よけるときの余裕を持たせて少し大きく教える（底の高さは変えない）。"""
    x, y, z = position
    radius, height = BOWL_RADIUS, BOWL_HEIGHT
    if padded:
        radius, height = BOWL_RADIUS + PAD_SIDE, BOWL_HEIGHT + PAD_TOP
        z += PAD_TOP / 2
    arm.add_collision_cylinder(id=BOWL_IDS[bowl], height=height, radius=radius,
                               position=(x, y, z), quat_xyzw=NO_TURN, frame_id=FRAME)


def clear(arm):
    """教えた机と茶碗を、手に持たせた物も含めて、すべて消す。"""
    arm.detach_all_collision_objects()
    time.sleep(SCENE_WAIT)
    for object_id in [TABLE_ID, *BOWL_IDS.values()]:
        arm.remove_collision_object(object_id)
    time.sleep(SCENE_WAIT)


def reset_scene(arm, bowls):
    """まっさらにしてから、机と、bowls（{'red': (x, y), ...}）の茶碗を教え直す。"""
    clear(arm)
    add_table(arm)
    for bowl, (x, y) in bowls.items():
        add_bowl(arm, bowl, (x, y, BOWL_Z))
    time.sleep(SCENE_WAIT)


def remove_bowl(arm, bowl):
    """つかみに行く茶碗を外す（指が触れても「ぶつかる」と考えさせないため）。"""
    arm.remove_collision_object(BOWL_IDS[bowl])
    time.sleep(SCENE_WAIT)


def hold_bowl(arm, bowl, position):
    """手の中にある茶碗を教えて、手に持たせる（腕と一緒に動く物として扱わせる）。"""
    add_bowl(arm, bowl, position, padded=False)
    time.sleep(SCENE_WAIT)
    arm.attach_collision_object(BOWL_IDS[bowl], link_name=HAND_LINK, touch_links=TOUCH_LINKS)
    time.sleep(SCENE_WAIT)


def release_bowl(arm, bowl):
    """手から離して外す（離したあと腕を引くとき、指が触れても失敗させないため）。"""
    arm.detach_collision_object(BOWL_IDS[bowl])
    time.sleep(SCENE_WAIT)
    arm.remove_collision_object(BOWL_IDS[bowl])
    time.sleep(SCENE_WAIT)


def place_bowl(arm, bowl, x, y):
    """置いた茶碗を、その場所に教え直す（次にお出しするとき、よけさせるため）。"""
    add_bowl(arm, bowl, (x, y, BOWL_Z))
    time.sleep(SCENE_WAIT)


def main():
    rclpy.init()
    node = rclpy.create_node("planning_scene_test")
    arm = MoveIt2(node=node, joint_names=ARM_JOINTS, base_link_name=FRAME,
                  end_effector_name=HAND_LINK, group_name="arm",
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
        reset_scene(arm, TEST_BOWLS)
        print("机と茶碗2つを MoveIt に教えました。RViz で確かめてください。")

    time.sleep(1.0)  # 送り終わるのを待つ
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()