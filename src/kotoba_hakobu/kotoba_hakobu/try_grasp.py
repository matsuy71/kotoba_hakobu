"""試運転⑦：赤い茶碗を、上から下ろして横からはさみ、持ち上げる"""
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pymoveit2 import MoveIt2

from kotoba_hakobu.try_pose_goal import set_gripper
from kotoba_hakobu.try_level_move import move

# ===== 設定 =====
ARM_JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
HOME = [0.0, -1.16, -2.01, -0.73]
GRIPPER_OPEN_DEG = -30.0
GRIPPER_CLOSE_DEG = 35.0     # 茶碗（直径5cm）をはさむには、指の真ん中が当たる +26°より深く閉じる
BOWL = (0.1631, 0.0761)      # 赤い茶碗の中心（ロボットの座標、m）
GRASP_FORWARD = 0.10         # 手首から、ねらう所まで：前に（m）。つかむと 9.1cm まですべり込む（6cm の茶碗で測定）
GRASP_LEFT = 0.007           # 手首から、つかむ所まで：左に（m）※直径6cm の茶碗の場合
Z_ABOVE = 0.08               # 茶碗の上で待つときの手首の高さ（m）
Z_GRASP = 0.03               # つかむときの手首の高さ（m）
Z_LIFT = 0.07                # 持ち上げたときの手首の高さ（m）
HOLD_SEC = 10                # 持ち上げたまま待つ時間（秒）

def hand_angle(gripper):
    """指の関節の、今の角度（度）を返す。読めなければ None。"""
    js = getattr(gripper, "joint_state", None)
    if js is None or "crane_plus_joint_hand" not in js.name:
        return None
    return math.degrees(js.position[js.name.index("crane_plus_joint_hand")])

def wrist_for_bowl(bx, by):
    """茶碗の中心 (bx, by) をつかむための、手首の位置 (x, y) と腕の向き（度）を返す。"""
    dist = math.hypot(bx, by)                           # 土台から茶碗の中心までの距離
    reach = math.sqrt(dist ** 2 - GRASP_LEFT ** 2)      # 腕の線に沿って、茶碗の横まで行く距離
    w = reach - GRASP_FORWARD                           # 土台から手首までの距離
    psi = math.atan2(by, bx) - math.atan2(GRASP_LEFT, reach)   # 茶碗の方向より、少し右に向ける
    return w * math.cos(psi), w * math.sin(psi), math.degrees(psi)


def main():
    rclpy.init()
    node = Node("try_grasp")
    group = ReentrantCallbackGroup()
    arm = MoveIt2(node=node, joint_names=ARM_JOINTS,
                  base_link_name="crane_plus_base", end_effector_name="crane_plus_link4",
                  group_name="arm", callback_group=group)
    gripper = MoveIt2(node=node, joint_names=["crane_plus_joint_hand"],
                      base_link_name="crane_plus_link4", end_effector_name="crane_plus_link_hand",
                      group_name="gripper", callback_group=group)
    arm.allowed_planning_time = 2.0
    for m in (arm, gripper):
        m.max_velocity = 0.3
        m.max_acceleration = 0.3
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)

    wx, wy, psi = wrist_for_bowl(*BOWL)
    log = node.get_logger()
    log.info(f"手首の位置 ({wx:.4f}, {wy:.4f})、腕の向き {psi:.1f}度")

    set_gripper(node, gripper, GRIPPER_OPEN_DEG)                       # 1. 指を開く
    log.info(f"2. 茶碗の上へ: {move(node, arm, (wx, wy, Z_ABOVE))}")
    log.info(f"3. まっすぐ下ろす: {move(node, arm, (wx, wy, Z_GRASP), True)}")
    set_gripper(node, gripper, GRIPPER_CLOSE_DEG)                      # 4. 指を閉じる
    log.info(f"   指が止まった角度: {hand_angle(gripper)}")
    log.info(f"5. まっすぐ持ち上げる: {move(node, arm, (wx, wy, Z_LIFT), True)}")
    log.info(f"6. {HOLD_SEC}秒待ちます。今のうちに、端末3で茶碗の位置と向きを確かめてください")
    time.sleep(HOLD_SEC)
    log.info(f"7. まっすぐ下ろす: {move(node, arm, (wx, wy, Z_GRASP), True)}")
    set_gripper(node, gripper, GRIPPER_OPEN_DEG)
    log.info(f"   まっすぐ上へ離れる: {move(node, arm, (wx, wy, Z_ABOVE), True)}")
    arm.move_to_configuration(HOME)
    arm.wait_until_executed()
    log.info("おわり")

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()