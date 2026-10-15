"""試運転④：手先の位置を指定して腕を動かし、グリッパーを開け閉めするノード"""
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pymoveit2 import MoveIt2

# ===== 設定（数字はここにまとめる） =====
ARM_JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
HOME = [0.0, -1.16, -2.01, -0.73]
GRIPPER_OPEN_DEG = -30.0      # 指を開く角度（度）
GRIPPER_CLOSE_DEG = 10.0      # 指を閉じる角度（度）
POSITION_TOLERANCE = 0.005    # 位置の許容誤差（m）＝5mm
ORIENTATION_TOLERANCE = 0.001  # 向きの許容誤差（ラジアン）＝約1度
PITCH_SIDE_DEG = 90.0         # 手先を水平に向ける
PITCH_DOWN_DEG = 180.0        # 手先を真下に向ける

def quat_from_rpy(roll, pitch, yaw):
    """ロール・ピッチ・ヨー（ラジアン）を四元数 (x, y, z, w) に直す。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)

def move_arm_to(node, arm, position, pitch_deg=PITCH_SIDE_DEG):
    """手先を position へ動かす。向きは「目標の方向（ヨー）＋指定したピッチ」。
    計画できなければ動かずに False を返す。"""
    yaw = math.atan2(position[1], position[0])
    quat = quat_from_rpy(0.0, math.radians(pitch_deg), yaw)
    node.get_logger().info(
        f"計画します: 手先を {position} へ（ピッチ {pitch_deg} 度、ヨー {math.degrees(yaw):.0f} 度）")
    trajectory = arm.plan(position=position, quat_xyzw=quat,
                          tolerance_position=POSITION_TOLERANCE,
                          tolerance_orientation=ORIENTATION_TOLERANCE)
    if trajectory is None:
        node.get_logger().warning(f"届きません（計画できませんでした）: {position}")
        return False
    arm.execute(trajectory)
    result = arm.wait_until_executed()
    node.get_logger().info(f"実行の結果: {result}")
    return result


def set_gripper(node, gripper, angle_deg):
    """グリッパーを angle_deg（度）にする。マイナスで開く、プラスで閉じる。"""
    node.get_logger().info(f"グリッパーを {angle_deg} 度にします")
    gripper.move_to_configuration([math.radians(angle_deg)])
    result = gripper.wait_until_executed()
    node.get_logger().info(f"グリッパーの結果: {result}")
    return result


def main():
    rclpy.init()
    node = Node("try_pose_goal")
    callback_group = ReentrantCallbackGroup()

    arm = MoveIt2(node=node, joint_names=ARM_JOINTS,
                  base_link_name="crane_plus_base", end_effector_name="crane_plus_link4",
                  group_name="arm", callback_group=callback_group)
    gripper = MoveIt2(node=node, joint_names=["crane_plus_joint_hand"],
                      base_link_name="crane_plus_link4", end_effector_name="crane_plus_link_hand",
                      group_name="gripper", callback_group=callback_group)
    for m in (arm, gripper):
        m.max_velocity = 0.5
        m.max_acceleration = 0.5

    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)

    set_gripper(node, gripper, GRIPPER_OPEN_DEG)                     # 1. 指を開く
    move_arm_to(node, arm, (0.0, -0.09, 0.17))                       # 比較：サンプルと同じ位置と向き
    move_arm_to(node, arm, (0.12, 0.0, 0.12))                        # 2. 前へ（水平）
    move_arm_to(node, arm, (0.12, 0.0, 0.17), PITCH_DOWN_DEG)        # 3. 前・下向き（高さ 0.17）
    set_gripper(node, gripper, GRIPPER_CLOSE_DEG)                    # 4. 指を閉じる
    move_arm_to(node, arm, (0.0, 0.12, 0.17), PITCH_DOWN_DEG)        # 5. 左・下向き（高さ 0.17）
    set_gripper(node, gripper, GRIPPER_OPEN_DEG)                     # 6. 指を開く
    ok = move_arm_to(node, arm, (0.5, 0.0, 0.1))                     # 7. 届かない位置
    node.get_logger().info("届かない位置のテスト: " + ("正しく断った" if not ok else "動いてしまった（要確認）"))
    arm.move_to_configuration(HOME)                                  # 8. home へ戻る
    arm.wait_until_executed()

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()