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
from kotoba_hakobu import planning_scene as scene
from kotoba_hakobu.bowl_locator import TruePoseLocator
from kotoba_hakobu.camera_locator import CameraLocator
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
STEP_COUNT = 15
WRIST_MIN, WRIST_MAX = 0.06, 0.16   # 手首が届く範囲（土台からの距離、m）
GRASP_EMPTY_DEG = 34.0              # 指がここまで閉じたら、何もはさめていない
LIFT_MIN_Z = 0.04                   # 持ち上げた茶碗の中心は、机からこれより高いはず（m）
SEAT_FREE_RADIUS = 0.06             # 席の中心からこの距離に茶碗があれば「あいていない」（m）
PLACE_TOL_M = 0.02                  # 置いた位置のずれの基準（m）
TILT_LIMIT_DEG = 15.0               # 傾きの基準（度）
Z_PASS = 0.11                       # 向きを変えるときの手首の高さ（m）。持っている茶碗の底が8cmになり、机の上の茶碗（高さ4cm）の上を通れる
HELD_BELOW = 0.010                  # つかんだ茶碗の真ん中は、手首からこれだけ下にある（m。試運転⑦で測定）


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
        self.declare_parameter("locator", "true")
        locator_name = self.get_parameter("locator").value
        if locator_name not in ("true", "camera"):
            raise ValueError(f"locator は true か camera です（渡された値：{locator_name}）")
        self.use_camera = locator_name == "camera"
        self.locator = CameraLocator(self) if self.use_camera else TruePoseLocator()
        self.get_logger().info(f"茶碗の位置の読み方：{'カメラ版' if self.use_camera else '真の値版'}")
        self.declare_parameter("scene", False)
        self.use_scene = bool(self.get_parameter("scene").value)
        self.get_logger().info(f"机と茶碗を MoveIt に教える：{'する' if self.use_scene else 'しない'}")
        self.declare_parameter("z_pass", Z_PASS)
        self.z_pass = float(self.get_parameter("z_pass").value)
        if self.z_pass != Z_PASS:
            self.get_logger().warning(f"向きを変えるときの手首の高さを変えて起動しています（z_pass={self.z_pass}）")
        self.declare_parameter("grasp_empty_deg", GRASP_EMPTY_DEG)
        self.declare_parameter("lift_min_z", LIFT_MIN_Z)
        grasp_empty_deg = self.get_parameter("grasp_empty_deg").value
        lift_min_z = self.get_parameter("lift_min_z").value
        if grasp_empty_deg != GRASP_EMPTY_DEG or lift_min_z != LIFT_MIN_Z:
            self.get_logger().warning(
                f"テスト用の値で起動しています（grasp_empty_deg={grasp_empty_deg}、lift_min_z={lift_min_z}）")
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
        """注文（ゴール）を受けて、15段階でお茶をお出しし、結果（リザルト）を返す。"""
        goal = goal_handle.request
        start = time.time()
        result = ServeTea.Result()
        max_tilt = 0.0
        holding = False
        grasp_empty_deg = self.get_parameter("grasp_empty_deg").value
        lift_min_z = self.get_parameter("lift_min_z").value
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
            if self.use_scene:
                # 机と、見つかった茶碗すべてを、まっさらにしてから MoveIt に教え直す
                known = {goal.bowl: (bowl["x"], bowl["y"])}
                for other in BOWL_JA:
                    if other != goal.bowl:
                        o = self.locator.locate(other)
                        if o:
                            known[other] = (o["x"], o["y"])
                scene.reset_scene(self.arm, known)
                self.get_logger().info(f"MoveIt に机と茶碗（{', '.join(known)}）を教えました")
            else:
                # 教えない場合も、前に教えた物が残っていると計画が変わるので、消しておく
                scene.clear(self.arm)

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
            if self.use_scene:
                scene.remove_bowl(self.arm, goal.bowl)
            self.report(goal_handle, 5, "茶碗の上へ動いています")
            self.motion("茶碗の上へ", (gx, gy, Z_ABOVE))
            self.report(goal_handle, 6, "まっすぐ下ろしています")
            self.motion("下ろす", (gx, gy, Z_GRASP), True)
            self.report(goal_handle, 7, "指を閉じています")
            set_gripper(self, self.gripper, GRIPPER_CLOSE_DEG)
            self.report(goal_handle, 8, "つかめたか確かめています")
            angle = hand_angle(self.gripper)
            if angle is None or angle >= grasp_empty_deg:
                self.get_logger().warning(f"指の角度 {angle}")
                raise StepFailed("つかめたか確かめる", f"{bowl_ja}をおつかみできませんでした。")
            holding = True
            if self.use_scene:
                # 手の中の茶碗の場所（手首から前へ HELD_FORWARD・左へ HELD_LEFT・下へ HELD_BELOW）
                yaw = math.atan2(gy, gx)
                hx = gx + HELD_FORWARD * math.cos(yaw) - HELD_LEFT * math.sin(yaw)
                hy = gy + HELD_FORWARD * math.sin(yaw) + HELD_LEFT * math.cos(yaw)
                scene.hold_bowl(self.arm, goal.bowl, (hx, hy, Z_GRASP - HELD_BELOW))

            # ----- 運ぶ（失敗したら、茶碗を持ったまま止まる） -----
            self.report(goal_handle, 9, "まっすぐ高く持ち上げています")
            max_tilt = max(max_tilt, self.motion("持ち上げる", (gx, gy, Z_CARRY), True, True))
            self.report(goal_handle, 10, "持ち上がったか確かめています")
            if self.use_camera:
                # カメラ版：持ち上げた茶碗は腕に隠れて見えないので、指が茶碗をはさんだままかで判断する
                angle = hand_angle(self.gripper)
                lifted_ok = angle is not None and angle < grasp_empty_deg
                if not lifted_ok:
                    self.get_logger().warning(f"持ち上げたあとの指の角度 {angle}")
            else:
                # 真の値版：茶碗の本当の高さで判断する
                lifted = self.locator.locate(goal.bowl)
                lifted_ok = lifted is not None and lifted["z"] >= lift_min_z
                if lifted and not lifted_ok:
                    self.get_logger().warning(f"持ち上げた茶碗の高さ {lifted['z']:.3f}m")
            if not lifted_ok:
                raise StepFailed("持ち上がったか確かめる", "お茶碗が持ち上がっておりません。")
            w = math.hypot(gx, gy)
            tx, ty = w * math.cos(math.radians(psi)), w * math.sin(math.radians(psi))
            self.report(goal_handle, 11, f"{seat_ja}の方へ向きを変えています（高いまま）")
            max_tilt = max(max_tilt, self.motion("向きを変える", (tx, ty, self.z_pass), False, True))
            self.report(goal_handle, 12, "運ぶ高さまで下ろしています")
            if self.z_pass > Z_CARRY:
                max_tilt = max(max_tilt, self.motion("運ぶ高さへ下ろす", (tx, ty, Z_CARRY), True, True))
            self.report(goal_handle, 13, "まっすぐ前へ出しています")
            max_tilt = max(max_tilt, self.motion("前へ出す", (sx, sy, Z_CARRY), True, True))
            self.report(goal_handle, 14, "まっすぐ下ろしています")
            max_tilt = max(max_tilt, self.motion("置く", (sx, sy, Z_PLACE), True, True))

            # ----- 置いて、確かめる -----
            self.report(goal_handle, 15, "指を開いて離れ、置いた位置を確かめています")
            if self.use_scene:
                scene.release_bowl(self.arm, goal.bowl)
            set_gripper(self, self.gripper, GRIPPER_OPEN_DEG)
            holding = False
            self.motion("離れる", (sx, sy, Z_ABOVE), True)
            self.arm.move_to_configuration(HOME)
            self.arm.wait_until_executed()
            placed = self.locator.locate(goal.bowl)
            if placed is None:
                raise StepFailed("置いた位置を確かめる", "置いたお茶碗の位置を確かめられません。")
            if self.use_scene:
                scene.place_bowl(self.arm, goal.bowl, placed["x"], placed["y"])
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