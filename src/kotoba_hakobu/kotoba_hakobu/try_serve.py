"""試運転⑧：赤い茶碗をつかみ、次客の席にまっすぐお出しする"""
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pymoveit2 import MoveIt2

from kotoba_hakobu.try_pose_goal import set_gripper
from kotoba_hakobu.try_level_move import TiltMonitor, move

# ===== 設定 =====
ARM_JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
HOME = [0.0, -1.16, -2.01, -0.73]
GRIPPER_OPEN_DEG = -30.0
GRIPPER_CLOSE_DEG = 35.0
BOWL = (0.1631, 0.0761)                 # 赤い茶碗の中心（ロボットの座標、m）
SEATS = {"正客": (0.1966, 0.1377), "次客": (0.24, 0.0), "三客": (0.1966, -0.1377)}
AIM_FORWARD, AIM_LEFT = 0.10, 0.007     # 下ろすときにねらう所（手首から、m）
HELD_FORWARD, HELD_LEFT = 0.091, 0.007  # つかんだ後に茶碗がある所（手首から、m。試運転⑦で測定）
Z_ABOVE = 0.08     # 茶碗の上で待つ・離れるときの手首の高さ
Z_GRASP = 0.03     # つかむときの手首の高さ
Z_CARRY = 0.07     # 運ぶときの手首の高さ
Z_PLACE = 0.031    # 置くときの手首の高さ（茶碗の底が机から1mm 上）


def wrist_for(tx, ty, forward, left):
    """茶碗の中心を (tx, ty) にするための、手首の位置 (x, y) と腕の向き（度）を返す。
    forward・left は、手首から見た茶碗の中心の位置（前・左、m）。"""
    dist = math.hypot(tx, ty)
    reach = math.sqrt(dist ** 2 - left ** 2)
    w = reach - forward
    psi = math.atan2(ty, tx) - math.atan2(left, reach)
    return w * math.cos(psi), w * math.sin(psi), math.degrees(psi)


def main():
    rclpy.init()
    node = Node("try_serve")
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
    monitor = TiltMonitor(node)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)
    log = node.get_logger()

    seat_name = "次客"
    gx, gy, _ = wrist_for(*BOWL, AIM_FORWARD, AIM_LEFT)                  # つかむときの手首
    sx, sy, psi = wrist_for(*SEATS[seat_name], HELD_FORWARD, HELD_LEFT)  # 置くときの手首
    w_grasp = math.hypot(gx, gy)
    tx, ty = w_grasp * math.cos(math.radians(psi)), w_grasp * math.sin(math.radians(psi))  # 向きを変えた後
    log.info(f"{seat_name}の席 {SEATS[seat_name]} へ。置くときの手首 ({sx:.4f}, {sy:.4f})、腕の向き {psi:.1f}度")

    # ----- つかむ（試運転⑦と同じ） -----
    set_gripper(node, gripper, GRIPPER_OPEN_DEG)
    log.info(f"茶碗の上へ: {move(node, arm, (gx, gy, Z_ABOVE))}")
    log.info(f"まっすぐ下ろす: {move(node, arm, (gx, gy, Z_GRASP), True)}")
    set_gripper(node, gripper, GRIPPER_CLOSE_DEG)
    log.info(f"まっすぐ持ち上げる: {move(node, arm, (gx, gy, Z_CARRY), True, monitor)}")

    # ----- お出しする -----
    log.info(f"向きを変える: {move(node, arm, (tx, ty, Z_CARRY), False, monitor)}")
    log.info(f"まっすぐ前へ: {move(node, arm, (sx, sy, Z_CARRY), True, monitor)}")
    log.info(f"まっすぐ下ろす: {move(node, arm, (sx, sy, Z_PLACE), True, monitor)}")
    set_gripper(node, gripper, GRIPPER_OPEN_DEG)
    log.info(f"まっすぐ上へ離れる: {move(node, arm, (sx, sy, Z_ABOVE), True)}")
    arm.move_to_configuration(HOME)
    arm.wait_until_executed()
    log.info(f"おわり。端末3で、茶碗の位置を測ってください（席の中心 {SEATS[seat_name]}）")

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()