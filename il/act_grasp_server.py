#!/usr/bin/env python3
"""ACT（模倣学習で手本をまねた動き）で、茶碗の上へ動き、下りて、手を閉じる。

serve_server の手順5〜7（茶碗の上へ → 下りる → 手を閉じる）の代わりに使う。
講義第5回 pai_training の scripts/infer.py の作りを参考にして書いた。講義の版と違うところ：
  1. 関節の順番を、手本（il/convert_grasp.py）と同じ joint1, 2, 3, 4, 手 にする
  2. まわりの状態（observation.environment_state）に、茶碗の位置 (x, y) を入れる
  3. サービスで頼まれて動き、手を閉じ終えたら返事をして止まる
  4. 安全のため、答えの角度を手本で動いた範囲におさめ、時間切れでも止まる
  5. 腕の行動は「次のコマまでに動く角度（差）」。目標は「今の角度+差×10コマ分」にする。コントローラは目標に約0.5秒遅れてついて来るので、1コマ分先だと遅すぎ（手本の約12分の1）、目標に差を積み上げる方式だと、止まるときに遅れの分（約5度）だけ行き過ぎて、指が茶碗のふちに乗った
pixi の箱（pai_training）の Python で動かす：
  python il/act_grasp_server.py

サービス /kotoba/act_grasp（kotoba_hakobu_interfaces/srv/ActGrasp）
  頼み：茶碗の位置 x, y（ロボットの座標 crane_plus_base、メートル）
  返事：手を閉じ終えたか（success）、説明（message）、かかった時間（seconds）
"""
import argparse
import math
import threading
import time

import numpy as np
import rclpy
import torch
from lerobot.common.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.common.policies.act.modeling_act import ACTPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kotoba_hakobu_interfaces.srv import ActGrasp

# 手本（il/convert_grasp.py）と同じ順番。/joint_states が流れてくる順番（joint1, 2, 4, 3, 手）とは違う
ARM_JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
HAND_JOINT = "crane_plus_joint_hand"
JOINTS = ARM_JOINTS + [HAND_JOINT]

MODEL_PATH = "/workspace/il_data/checkpoints/crane_plus_grasp_act_delta"
DATASET_ROOT = "/workspace/il_data/lerobot_dataset/crane_plus_grasp_delta"
REPO_ID = "crane_plus_grasp"

PERIOD = 0.05                 # 0.05秒ごと（1秒に20回）。手本と同じ
MOVE_SEC = PERIOD             # 「次の1コマ（0.05秒）でその角度にして」と送る。手本では、差の分を1コマで動いたため
GRIPPER_OPEN = math.radians(-30.0)
GRIPPER_CLOSE = math.radians(35.0)
CLOSE_LINE = (GRIPPER_OPEN + GRIPPER_CLOSE) / 2   # 手の答えがこれを超えたら「閉じ始めた」とみなす
HOLD_SEC = 1.5                # 閉じ始めてから、さらに続ける時間（手本は、閉じたあと1.0秒続けて記録した）
TIMEOUT_SEC = 30.0            # これを過ぎたら止める（手本は、止まっているコマを除いて1回が約11秒）
STATE_WAIT_SEC = 2.0          # 関節の角度が届くのを待つ時間
STATE_OLD_SEC = 0.5           # これより古い関節の角度は使わない
MARGIN = 0.1                  # 手本で腕が通った角度の範囲から、はみ出してよい角度（ラジアン。約6度）
DELTA_MARGIN = 0.01           # 手本で1コマに動いた角度（差）の範囲から、はみ出してよい角度（ラジアン。約0.6度）
LOG_EVERY = 10                # この回数ごと（0.5秒ごと）に、今の角度と目標の角度を表示する
REF_LEAD_MAX = 0.1            # 目標を、今の角度よりこれ（ラジアン。約5.7度）より先には置かない（安全の上限）
LOOKAHEAD = 10                # 目標を、差の何コマ分先に置くか（10コマ＝0.5秒。コントローラの遅れと同じくらい）


class ActGraspServer(Node):
    def __init__(self, model_path, threads):
        super().__init__("act_grasp_server")
        torch.set_num_threads(threads)

        # 学習した ACT を読みこむ
        self.policy = ACTPolicy.from_pretrained(model_path)
        self.policy.eval()
        self.policy.to("cpu")
        if "ENV" not in self.policy.config.normalization_mapping:
            raise RuntimeError("ACT の設定に ENV（まわりの状態）の正規化がない。il/train_grasp.py の変更1を確かめる")

        # 安全のため、ACT の答えを、手本の範囲（少しの余裕つき）におさめる
        stats = LeRobotDatasetMetadata(REPO_ID, root=DATASET_ROOT).stats
        # 腕の差：手本で1コマに動いた角度の範囲
        self.delta_low = np.asarray(stats["action"]["min"], dtype=np.float32).reshape(-1)[:4] - DELTA_MARGIN
        self.delta_high = np.asarray(stats["action"]["max"], dtype=np.float32).reshape(-1)[:4] + DELTA_MARGIN
        # 動いた先の角度：手本で腕が通った角度の範囲
        self.pos_low = np.asarray(stats["observation.state"]["min"], dtype=np.float32).reshape(-1)[:4] - MARGIN
        self.pos_high = np.asarray(stats["observation.state"]["max"], dtype=np.float32).reshape(-1)[:4] + MARGIN

        # 関節の角度は、別の係（スレッド）が書きこむので、鍵（lock）をかけて読み書きする
        self.lock = threading.Lock()
        self.state = None
        self.state_time = 0.0

        self.arm_pub = self.create_publisher(JointTrajectory, "/crane_plus_arm_controller/joint_trajectory", 10)
        self.hand_pub = self.create_publisher(JointTrajectory, "/crane_plus_gripper_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        # サービスは、関節の角度を聞く係とは別の組（callback group）にして、同時に動けるようにする
        self.create_service(ActGrasp, "/kotoba/act_grasp", self.on_request,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"ACT を読みこんだ（{model_path}、スレッド{torch.get_num_threads()}）。頼まれるのを待つ")

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not all(j in pos for j in JOINTS):
            return
        with self.lock:
            self.state = np.array([pos[j] for j in JOINTS], dtype=np.float32)   # 名前を見て、手本と同じ順番に並べる
            self.state_time = time.monotonic()

    def latest_state(self):
        with self.lock:
            if self.state is None or time.monotonic() - self.state_time > STATE_OLD_SEC:
                return None
            return self.state.copy()

    def send(self, target):
        for pub, names, values in ((self.arm_pub, ARM_JOINTS, target[:4]),
                                   (self.hand_pub, [HAND_JOINT], target[4:])):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in values]
            point.time_from_start.nanosec = int(MOVE_SEC * 1e9)
            traj = JointTrajectory()
            traj.joint_names = names
            traj.points.append(point)
            pub.publish(traj)

    def on_request(self, request, response):
        start = time.monotonic()
        self.get_logger().info(f"頼まれた：茶碗の位置 x={request.x:.3f} y={request.y:.3f}")
        response.success, response.message = self.run(request.x, request.y, start)
        response.seconds = time.monotonic() - start
        self.get_logger().info(f"{response.message}（{response.seconds:.1f}秒）")
        return response

    def run(self, x, y, start):
        while self.latest_state() is None:
            if time.monotonic() - start > STATE_WAIT_SEC:
                return False, "関節の角度（/joint_states）が届かない"
            time.sleep(PERIOD)

        self.policy.reset()   # 前の動きの「先の予定」を消してから始める
        bowl = torch.tensor([[x, y]], dtype=torch.float32)
        close_time = None
        next_tick = time.monotonic()
        tick = 0
        while True:
            now = time.monotonic()
            if now - start > TIMEOUT_SEC:
                return False, f"時間切れ（{TIMEOUT_SEC:.0f}秒たっても、手を閉じ終えなかった）"
            state = self.latest_state()
            if state is None:
                return False, "関節の角度（/joint_states）が途中で届かなくなった"

            obs = {"observation.state": torch.from_numpy(state).unsqueeze(0),
                   "observation.environment_state": bowl}
            infer_start = time.monotonic()
            with torch.no_grad():
                action = self.policy.select_action(obs).squeeze(0).numpy()
            infer_ms = (time.monotonic() - infer_start) * 1000

            # 腕：目標は「今の角度 ＋ 差×LOOKAHEAD」。コントローラは目標に約0.5秒遅れてついて来るので、
            # 0.5秒分（10コマ）先に目標を置くと、手本に近い速さが出る。止まる場所に近づくと差が0に
            # 近づくので、目標が今の角度に重なっていき、行き過ぎない
            delta = np.clip(action[:4], self.delta_low, self.delta_high)
            # 先に置く量が上限を超えるときは、関節ごとに切らず、全体を同じ割合で縮める。
            # 関節ごとに切ると、関節どうしの速さの比率（＝通り道の形）が壊れて、
            # 回りきる前に下り始め、茶碗の左に下りてしまった
            lead = delta * LOOKAHEAD
            biggest = np.abs(lead).max()
            if biggest > REF_LEAD_MAX:
                lead *= REF_LEAD_MAX / biggest
            target = np.empty(5, dtype=np.float32)
            target[:4] = np.clip(state[:4] + lead, self.pos_low, self.pos_high)
            target[4] = np.clip(action[4], GRIPPER_OPEN, GRIPPER_CLOSE)

            # 0.5秒ごとに、今の角度と目標の角度（どちらも度。joint1, 2, 3, 4, 手）と、計算にかかった時間を表示する
            if tick % LOG_EVERY == 0:
                self.get_logger().info(f"{now - start:4.1f}秒 今 {np.round(np.degrees(state), 1)} "
                                       f"→ 目標 {np.round(np.degrees(target), 1)}（計算 {infer_ms:.0f}ms）")
            tick += 1

            if close_time is None and target[4] > CLOSE_LINE:
                close_time = now
            if close_time is not None and now - close_time >= HOLD_SEC:
                target[4] = GRIPPER_CLOSE   # 最後は、手本と同じ「閉じる」角度で、はっきり閉じる
                self.send(target)
                return True, "手を閉じ終えた"
            self.send(target)

            next_tick += PERIOD
            if next_tick < time.monotonic():   # 計算が間に合わなかったときは、待たずに次へ
                next_tick = time.monotonic()
            time.sleep(max(0.0, next_tick - time.monotonic()))

def main():
    parser = argparse.ArgumentParser(description="ACT で茶碗をつかむサービス（/kotoba/act_grasp）")
    parser.add_argument("--model-path", default=MODEL_PATH, help="学習した ACT の保存先")
    parser.add_argument("--threads", type=int, default=4,
                        help="CPU のスレッドの数（Gazebo も CPU を使うので、学習のときより少なめ）")
    args = parser.parse_args()

    rclpy.init()
    node = ActGraspServer(args.model_path, args.threads)
    executor = MultiThreadedExecutor(num_threads=2)
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