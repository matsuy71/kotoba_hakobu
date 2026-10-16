"""試運転⑥：茶碗を水平に保って運べるか。ふつうの計画と直線の動きで、傾きを比べる（横からつかむ姿勢）"""
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from pymoveit2 import MoveIt2
from tf2_ros import Buffer, TransformException, TransformListener

from kotoba_hakobu.try_pose_goal import quat_from_rpy

# ===== 設定 =====
ARM_JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
HOME = [0.0, -1.16, -2.01, -0.73]
PITCH_SIDE_DEG = 90.0         # 手先を水平に向ける（茶碗を横からつかむ姿勢）
POSITION_TOLERANCE = 0.005
ORIENTATION_TOLERANCE = 0.001
PLANNING_TIME = 2.0           # 1回の計画に使ってよい時間（秒）
PLAN_TRIES = 3                # 計画に失敗したら、やり直す回数
CARTESIAN_FRACTION = 0.99     # 直線の動きは、道すじの99%以上を行けたときだけ使う
Z = 0.06                      # 手首の高さ（m）


class TiltMonitor:
    """腕が動いている間、茶碗の傾き（＝link4 の x 軸の、真下からのずれ）を測り続け、最大値を記録する。"""

    def __init__(self, node):
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.max_tilt = 0.0
        self.running = False
        self.thread = None

    def _loop(self):
        while self.running:
            try:
                t = self.buffer.lookup_transform("crane_plus_base", "crane_plus_link4", Time())
                q = t.transform.rotation
                xz = 2.0 * (q.x * q.z - q.w * q.y)   # link4 の x 軸（茶碗の上下の軸）の上下成分
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, -xz))))
                self.max_tilt = max(self.max_tilt, tilt)
            except TransformException:
                pass
            time.sleep(0.02)

    def start(self):
        self.max_tilt = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()
        return self.max_tilt


def move(node, arm, position, cartesian=False, monitor=None):
    """手先を水平・目標の方向に向けたまま position へ動かす。cartesian=True なら直線で動かす。
    計画は PLAN_TRIES 回までやり直す。戻り値は（成功したか, 最大の傾き〔度〕）。"""
    yaw = math.atan2(position[1], position[0])
    quat = quat_from_rpy(0.0, math.radians(PITCH_SIDE_DEG), yaw)
    for attempt in range(1, PLAN_TRIES + 1):
        trajectory = arm.plan(position=position, quat_xyzw=quat,
                              tolerance_position=POSITION_TOLERANCE,
                              tolerance_orientation=ORIENTATION_TOLERANCE,
                              cartesian=cartesian,
                              cartesian_fraction_threshold=CARTESIAN_FRACTION)
        if trajectory is not None:
            break
        node.get_logger().warning(f"計画に失敗（{attempt}回目）: {position}")
    else:
        node.get_logger().error(f"{PLAN_TRIES}回とも計画に失敗しました: {position}")
        return False, None
    if monitor:
        monitor.start()
    arm.execute(trajectory)
    ok = arm.wait_until_executed()
    tilt = monitor.stop() if monitor else None
    return ok, tilt


def main():
    rclpy.init()
    node = Node("try_level_move")
    arm = MoveIt2(node=node, joint_names=ARM_JOINTS,
                  base_link_name="crane_plus_base", end_effector_name="crane_plus_link4",
                  group_name="arm", callback_group=ReentrantCallbackGroup())
    arm.allowed_planning_time = PLANNING_TIME
    arm.max_velocity = 0.5
    arm.max_acceleration = 0.5
    monitor = TiltMonitor(node)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)

    # 0. 測る道具の確かめ：home（茶碗が立つ向き）→ 真上に伸ばす（茶碗が90度倒れる向き）
    arm.move_to_configuration(HOME)
    arm.wait_until_executed()
    monitor.start()
    arm.move_to_configuration([0.0, 0.0, 0.0, 0.0])
    arm.wait_until_executed()
    check = monitor.stop()

    near, far, up = (0.07, 0.0, Z), (0.15, 0.0, Z), (0.07, 0.0, Z + 0.04)
    a = math.radians(-40)
    near_r = (0.07 * math.cos(a), 0.07 * math.sin(a), Z)
    far_r = (0.15 * math.cos(a), 0.15 * math.sin(a), Z)

    results = []
    move(node, arm, near)                                                              # 準備
    for i in range(5):                                                                 # A を5往復
        results.append((f"A ふつう 手元→前方 {i + 1}回目", *move(node, arm, far, monitor=monitor)))
        results.append((f"A ふつう 前方→手元 {i + 1}回目", *move(node, arm, near, monitor=monitor)))
    results.append(("B 直線  手元→前方", *move(node, arm, far, True, monitor)))
    results.append(("B 直線  前方→手元", *move(node, arm, near, True, monitor)))
    results.append(("B 直線  真上に4cm", *move(node, arm, up, True, monitor)))
    move(node, arm, near_r)                                                            # 準備
    results.append(("B 直線  右斜め前 手元→前方", *move(node, arm, far_r, True, monitor)))
    arm.move_to_configuration(HOME)
    arm.wait_until_executed()

    print(f"\n道具の確かめ：home→真上の最大の傾き = {check:.1f}度（90度前後なら正しく測れている）")
    print("=== 結果（傾き＝茶碗の上下の軸の、真下からのずれの最大値。目標は15度以内） ===")
    for name, ok, tilt in results:
        tilt_text = f"{tilt:.1f}度" if tilt is not None else "―"
        print(f"{name}: 成功={ok}  最大の傾き={tilt_text}")

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()