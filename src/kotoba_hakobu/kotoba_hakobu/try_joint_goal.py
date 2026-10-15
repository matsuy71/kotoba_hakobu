"""試運転③：関節の角度を指定して、CRANE+ V2 のアームを動かすノード"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pymoveit2 import MoveIt2

# CRANE+ V2 の4つの関節の名前（MoveIt の設定ファイルで確認した名前）
JOINT_NAMES = [
    "crane_plus_joint1",
    "crane_plus_joint2",
    "crane_plus_joint3",
    "crane_plus_joint4",
]

# 動かしたい姿勢（関節の角度。単位はラジアン）
POSES = {
    "vertical": [0.0, 0.0, 0.0, 0.0],        # まっすぐ上
    "bow": [0.0, -0.6, -0.8, -0.6],          # 少し前にかがむ
    "home": [0.0, -1.16, -2.01, -0.73],      # たたんだ姿勢
}


def main():
    # ① ROS 2 を使い始める
    rclpy.init()
    node = Node("try_joint_goal")

    # ② MoveIt に頼むための窓口（pymoveit2）を作る
    callback_group = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name="crane_plus_base",
        end_effector_name="crane_plus_link4",
        group_name="arm",
        callback_group=callback_group,
    )
    moveit2.max_velocity = 0.5      # 速さを最大の50%にする
    moveit2.max_acceleration = 0.5  # 加速の強さを最大の50%にする

    # ③ ノードが通信を受け取り続けられるよう、別の流れ（スレッド）で動かす
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)  # 準備が整うまで少し待つ

    # ④ 姿勢を1つずつ頼み、動き終わるまで待って、結果を表示する
    for name in ["vertical", "bow", "home"]:
        node.get_logger().info(f"{name} へ動かします: {POSES[name]}")
        moveit2.move_to_configuration(POSES[name])
        result = moveit2.wait_until_executed()
        node.get_logger().info(f"{name} の結果: {result}")

    # ⑤ 後片付け
    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()