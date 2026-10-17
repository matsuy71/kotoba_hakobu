"""お出しするアクションサーバー：茶碗と席を受け取り、茶碗を水平に保ったまま席へお出しする"""
import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pymoveit2 import MoveIt2

from kotoba_hakobu_interfaces.action import ServeTea
from kotoba_hakobu.bowl_locator import TruePoseLocator
from kotoba_hakobu.try_grasp import hand_angle
from kotoba_hakobu.try_level_move import TiltMonitor, move
from kotoba_hakobu.try_pose_goal import set_gripper
from kotoba_hakobu.try_serve import (
    ARM_JOINTS, HOME, GRIPPER_OPEN_DEG, GRIPPER_CLOSE_DEG,
    AIM_FORWARD, AIM_LEFT, HELD_FORWARD, HELD_LEFT,
    Z_ABOVE, Z_GRASP, Z_CARRY, Z_PLACE, wrist_for)

# ===== 設定 =====
BOWL_JA = {"red": "赤楽のお茶碗", "black": "黒楽のお茶碗"}
SEATS = {"shokyaku": ("お正客様", (0.1966, 0.1377)),
         "jikyaku": ("お次客様", (0.24, 0.0)),
         "sankyaku": ("お三客様", (0.1966, -0.1377))}
HOLDING_NOTE = "お茶碗を持ったまま止まっております。シミュレータを起動し直してください。"
STEP_COUNT = 14
WRIST_MIN, WRIST_MAX = 0.06, 0.16   # 手首が届く範囲（土台からの距離、m）
GRASP_EMPTY_DEG = 34.0              # 指がここまで閉じたら、何もはさめていない
LIFT_MIN_Z = 0.04                   # 持ち上げた茶碗の中心は、机からこれより高いはず（m）
SEAT_FREE_RADIUS = 0.06             # 席の中心からこの距離に茶碗があれば「あいていない」（m）
PLACE_TOL_M = 0.02                  # 置いた位置のずれの基準（m）
TILT_LIMIT_DEG = 15.0               # 傾きの基準（度）


class StepFailed(Exception):
    """段階が失敗したことを知らせる例外。どの段階か（step）と、亭主に伝える言葉（message）を持つ。"""

    def __init__(self, step, message):
        super().__init__(message)
        self.step = step
        self.message = message


class ServeTeaServer(Node):
    """アクション serve_tea を受け付けて、お茶をお出しするノード。"""

    def __init__(self):
        super().__init__("serve_tea_server")
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
        self.monitor = TiltMonitor(self)
        self.locator = TruePoseLocator()
        self.server = ActionServer(self, ServeTea, "serve_tea", self.execute, callback_group=group)
        self.get_logger().info("お出しするアクションサーバーを起動しました（serve_tea）")

    def report(self, goal_handle, index, text):
        """途中経過（フィードバック）を送り、画面にも表示する。"""
        feedback = ServeTea.Feedback()
        feedback.step, feedback.step_index, feedback.step_count = text, index, STEP_COUNT
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"[{index}/{STEP_COUNT}] {text}")

    def motion(self, step, position, cartesian=False, measure=False):
        """腕を動かす。失敗したら StepFailed を投げる。measure=True なら、その間の最大の傾きを返す。"""
        ok, tilt = move(self, self.arm, position, cartesian, self.monitor if measure else None)
        if not ok:
            raise StepFailed(step, f"腕の動き（{step}）がうまくいきませんでした。")
        return tilt or 0.0

    def execute(self, goal_handle):
        """注文（ゴール）を受けて、14段階でお茶をお出しし、結果（リザルト）を返す。"""
        goal = goal_handle.request
        start = time.time()
        result = ServeTea.Result()
        max_tilt = 0.0
        holding = False
        try:
            # ----- 動く前の確かめ（問題があれば、腕を動かさずに断る） -----
            if goal.bowl not in BOWL_JA or goal.seat not in SEATS:
                raise StepFailed("注文の確かめ", f"お茶碗「{goal.bowl}」またはお席「{goal.seat}」が分かりません。")
            bowl_ja = BOWL_JA[goal.bowl]
            seat_ja, (seat_x, seat_y) = SEATS[goal.seat]

            self.report(goal_handle, 1, "茶碗の位置を読んでいます")
            bowl = self.locator.locate(goal.bowl)
            if bowl is None:
                raise StepFailed("茶碗の位置を読む", f"{bowl_ja}が見当たりません。")
            if bowl["tilt_deg"] > TILT_LIMIT_DEG or abs(bowl["z"] - 0.02) > 0.01:
                self.get_logger().warning(f"茶碗の高さ {bowl['z']:.3f}m・傾き {bowl['tilt_deg']:.1f}度")
                raise StepFailed("茶碗の位置を読む", f"{bowl_ja}が、所定の場所にありません。")

            self.report(goal_handle, 2, "届くか確かめています")
            gx, gy, _ = wrist_for(bowl["x"], bowl["y"], AIM_FORWARD, AIM_LEFT)
            sx, sy, psi = wrist_for(seat_x, seat_y, HELD_FORWARD, HELD_LEFT)
            for name, wx, wy in ((bowl_ja, gx, gy), (f"{seat_ja}のお席", sx, sy)):
                if not WRIST_MIN <= math.hypot(wx, wy) <= WRIST_MAX:
                    self.get_logger().warning(f"手首までの距離 {math.hypot(wx, wy):.3f}m")
                    raise StepFailed("届くか確かめる", f"{name}に手が届きません。")

            self.report(goal_handle, 3, "席があいているか確かめています")
            for other in BOWL_JA:
                if other == goal.bowl:
                    continue
                o = self.locator.locate(other)
                if o and math.hypot(o["x"] - seat_x, o["y"] - seat_y) < SEAT_FREE_RADIUS:
                    raise StepFailed("席があいているか確かめる", f"{seat_ja}のお席には、もうお茶碗がございます。")

            # ----- つかむ（失敗したら止まる。茶碗はまだ持っていない） -----
            self.report(goal_handle, 4, "指を開いています")
            set_gripper(self, self.gripper, GRIPPER_OPEN_DEG)
            self.report(goal_handle, 5, "茶碗の上へ動いています")
            self.motion("茶碗の上へ", (gx, gy, Z_ABOVE))
            self.report(goal_handle, 6, "まっすぐ下ろしています")
            self.motion("下ろす", (gx, gy, Z_GRASP), True)
            self.report(goal_handle, 7, "指を閉じています")
            set_gripper(self, self.gripper, GRIPPER_CLOSE_DEG)
            self.report(goal_handle, 8, "つかめたか確かめています")
            angle = hand_angle(self.gripper)
            if angle is None or angle >= GRASP_EMPTY_DEG:
                self.get_logger().warning(f"指の角度 {angle}")
                raise StepFailed("つかめたか確かめる", f"{bowl_ja}をおつかみできませんでした。")
            holding = True

            # ----- 運ぶ（失敗したら、茶碗を持ったまま止まる） -----
            self.report(goal_handle, 9, "まっすぐ持ち上げています")
            max_tilt = max(max_tilt, self.motion("持ち上げる", (gx, gy, Z_CARRY), True, True))
            self.report(goal_handle, 10, "持ち上がったか確かめています")
            lifted = self.locator.locate(goal.bowl)
            if lifted is None or lifted["z"] < LIFT_MIN_Z:
                if lifted:
                    self.get_logger().warning(f"持ち上げた茶碗の高さ {lifted['z']:.3f}m")
                raise StepFailed("持ち上がったか確かめる", "お茶碗が持ち上がっておりません。")
            w = math.hypot(gx, gy)
            tx, ty = w * math.cos(math.radians(psi)), w * math.sin(math.radians(psi))
            self.report(goal_handle, 11, f"{seat_ja}の方へ向きを変えています")
            max_tilt = max(max_tilt, self.motion("向きを変える", (tx, ty, Z_CARRY), False, True))
            self.report(goal_handle, 12, "まっすぐ前へ出しています")
            max_tilt = max(max_tilt, self.motion("前へ出す", (sx, sy, Z_CARRY), True, True))
            self.report(goal_handle, 13, "まっすぐ下ろしています")
            max_tilt = max(max_tilt, self.motion("置く", (sx, sy, Z_PLACE), True, True))

            # ----- 置いて、確かめる -----
            self.report(goal_handle, 14, "指を開いて離れ、置いた位置を確かめています")
            set_gripper(self, self.gripper, GRIPPER_OPEN_DEG)
            holding = False
            self.motion("離れる", (sx, sy, Z_ABOVE), True)
            self.arm.move_to_configuration(HOME)
            self.arm.wait_until_executed()
            placed = self.locator.locate(goal.bowl)
            if placed is None:
                raise StepFailed("置いた位置を確かめる", "置いたお茶碗の位置を確かめられません。")
            error = math.hypot(placed["x"] - seat_x, placed["y"] - seat_y)
            result.placement_error_m = error
            detail = f"ずれ {error * 1000:.1f}mm・運搬中の最大の傾き {max_tilt:.1f}度"
            if error <= PLACE_TOL_M and max_tilt <= TILT_LIMIT_DEG and placed["tilt_deg"] <= TILT_LIMIT_DEG:
                result.success = True
                result.message = f"{bowl_ja}を、{seat_ja}にお出しいたしました（{detail}）。"
            else:
                result.failed_step = "置いた位置を確かめる"
                result.message = f"お出ししましたが、ずれか傾きが大きくなりました（{detail}）。お確かめください。"

        except StepFailed as failure:
            result.success = False
            result.failed_step = failure.step
            result.message = failure.message + (HOLDING_NOTE if holding else "")

        result.max_tilt_deg = max_tilt
        result.elapsed_sec = time.time() - start
        self.get_logger().info(f"結果：{'成功' if result.success else '失敗'}　{result.message}")
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = ServeTeaServer()
    executor = MultiThreadedExecutor(8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()