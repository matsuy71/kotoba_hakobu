#!/usr/bin/env python3
"""画像つき ACT で、茶碗の上へ動き、下りて、手を閉じる。

il/act_grasp_server.py（座標版）の画像版。違い：
  1. 茶碗の座標の代わりに、天井カメラの画像（/kotoba/camera_small・160×120・約2Hz）を見る
  2. サービスの頼みの x, y は使わない（形をそろえるために受け取るだけ）
  3. 画像が古すぎる（2秒超）ときは止まる
目標の置き方（今の角度＋差×10コマ・全関節を同じ割合で縮める）は座標版と同じ。
pixi の箱の Python で動かす：python il/act_grasp_server_img.py
サービスの名前は座標版と同じ /kotoba/act_grasp なので、試すプログラムもそのまま使える
（同時に動かせるサーバーは、座標版か画像版のどちらか1つだけ）。
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kotoba_hakobu_interfaces.srv import ActGrasp

ARM_JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
HAND_JOINT = "crane_plus_joint_hand"
JOINTS = ARM_JOINTS + [HAND_JOINT]

MODEL_PATH = "/workspace/il_data/checkpoints/crane_plus_grasp_act_img_dagger"
DATASET_ROOT = "/workspace/il_data/lerobot_dataset/crane_plus_grasp_img_dagger"
REPO_ID = "crane_plus_grasp_img_dagger"
CAM_TOPIC = "/kotoba/camera_small"
CAM_RAW = "/chakai_camera/image"   # 元画像（640×480）。縮め係がいない編成でも、自分で縮めて使う
CAM_SCALE = 4                      # 縦横とも4画素ごとに1画素（手本を記録したときと同じ縮め方）
PERIOD = 0.05                 # 0.05秒ごと（1秒に20回）。手本と同じ
MOVE_SEC = PERIOD             # 「次の1コマ（0.05秒）でその角度にして」と送る
GRIPPER_OPEN = math.radians(-30.0)
GRIPPER_CLOSE = math.radians(35.0)
CLOSE_LINE = (GRIPPER_OPEN + GRIPPER_CLOSE) / 2   # 手の答えがこれを超えたら「閉じ始めた」とみなす
HOLD_SEC = 1.5                # 閉じ始めてから、さらに続ける時間
TIMEOUT_SEC = 30.0            # これを過ぎたら止める
STATE_WAIT_SEC = 3.0          # 関節の角度と画像がそろうのを待つ時間
STATE_OLD_SEC = 0.5           # これより古い関節の角度は使わない
IMG_OLD_SEC = 2.0             # これより古い画像は使わない（画像は約2Hzなので、ゆるめ）
MARGIN = 0.1                  # 手本で腕が通った角度の範囲から、はみ出してよい角度（ラジアン。約6度）
DELTA_MARGIN = 0.01           # 手本で1コマに動いた角度（差）の範囲から、はみ出してよい角度（ラジアン。約0.6度）
LOG_EVERY = 10                # この回数ごと（0.5秒ごと）に、今の角度と目標の角度を表示する
REF_LEAD_MAX = 0.1            # 目標を、今の角度よりこれ（ラジアン。約5.7度）より先には置かない（安全の上限）
LOOKAHEAD = 10                # 目標を、差の何コマ分先に置くか（10コマ＝0.5秒。コントローラの遅れと同じくらい）


class ActGraspImgServer(Node):
    def __init__(self, model_path, threads):
        super().__init__("act_grasp_server_img")
        torch.set_num_threads(threads)

        self.policy = ACTPolicy.from_pretrained(model_path)
        self.policy.eval()
        self.policy.to("cpu")

        # 安全のため、ACT の答えを、手本の範囲（少しの余裕つき）におさめる
        stats = LeRobotDatasetMetadata(REPO_ID, root=DATASET_ROOT).stats
        self.delta_low = np.asarray(stats["action"]["min"], dtype=np.float32).reshape(-1)[:4] - DELTA_MARGIN
        self.delta_high = np.asarray(stats["action"]["max"], dtype=np.float32).reshape(-1)[:4] + DELTA_MARGIN
        self.pos_low = np.asarray(stats["observation.state"]["min"], dtype=np.float32).reshape(-1)[:4] - MARGIN
        self.pos_high = np.asarray(stats["observation.state"]["max"], dtype=np.float32).reshape(-1)[:4] + MARGIN

        # 角度と画像は、別の係（スレッド）が書きこむので、鍵（lock）をかけて読み書きする
        self.lock = threading.Lock()
        self.state = None
        self.state_time = 0.0
        self.image = None
        self.image_time = 0.0

        self.arm_pub = self.create_publisher(JointTrajectory, "/crane_plus_arm_controller/joint_trajectory", 10)
        self.hand_pub = self.create_publisher(JointTrajectory, "/crane_plus_gripper_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(Image, CAM_TOPIC, self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image, CAM_RAW, self.on_image, qos_profile_sensor_data)
        self.create_service(ActGrasp, "/kotoba/act_grasp", self.on_request,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"画像つき ACT を読みこんだ（{model_path}、スレッド{torch.get_num_threads()}）。頼まれるのを待つ")

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not all(j in pos for j in JOINTS):
            return
        with self.lock:
            self.state = np.array([pos[j] for j in JOINTS], dtype=np.float32)
            self.state_time = time.monotonic()

    def on_image(self, msg):
        if msg.encoding != "rgb8":
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.height != 120:
            # 元画像（640×480）が来たときは、手本の記録と同じ縮め方で 160×120 にする
            arr = arr[::CAM_SCALE, ::CAM_SCALE]
            if arr.shape[0] != 120 or arr.shape[1] != 160:
                return
        # 学習のときと同じ形にする：色×縦×横、0〜1 の小数
        tensor = torch.from_numpy(arr.copy()).permute(2, 0, 1).float() / 255.0
        with self.lock:
            self.image = tensor
            self.image_time = time.monotonic()

    def latest(self):
        """新しい角度と画像の組を返す。どちらかが古い・ないときは None を返す。"""
        with self.lock:
            now = time.monotonic()
            if self.state is None or now - self.state_time > STATE_OLD_SEC:
                return None, None
            if self.image is None or now - self.image_time > IMG_OLD_SEC:
                return None, None
            return self.state.copy(), self.image

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
        self.get_logger().info(f"頼まれた（画像つき。頼みの x, y は使わない）")
        response.success, response.message = self.run(start)
        response.seconds = time.monotonic() - start
        self.get_logger().info(f"{response.message}（{response.seconds:.1f}秒）")
        return response

    def run(self, start):
        while self.latest()[0] is None:
            if time.monotonic() - start > STATE_WAIT_SEC:
                return False, "関節の角度か、カメラの画像（/kotoba/camera_small）が届かない。橋渡しを確かめる"
            time.sleep(PERIOD)

        self.policy.reset()   # 前の動きの「先の予定」を消してから始める
        close_time = None
        next_tick = time.monotonic()
        tick = 0
        while True:
            now = time.monotonic()
            if now - start > TIMEOUT_SEC:
                return False, f"時間切れ（{TIMEOUT_SEC:.0f}秒たっても、手を閉じ終えなかった）"
            state, image = self.latest()
            if state is None:
                return False, "関節の角度か画像が、途中で届かなくなった"

            obs = {"observation.state": torch.from_numpy(state).unsqueeze(0),
                   "observation.images.top": image.unsqueeze(0)}
            infer_start = time.monotonic()
            with torch.no_grad():
                action = self.policy.select_action(obs).squeeze(0).numpy()
            infer_ms = (time.monotonic() - infer_start) * 1000

            # 腕：目標は「今の角度 ＋ 差×LOOKAHEAD」。上限は、関節ごとに切らず全体を同じ割合で縮める
            delta = np.clip(action[:4], self.delta_low, self.delta_high)
            lead = delta * LOOKAHEAD
            biggest = np.abs(lead).max()
            if biggest > REF_LEAD_MAX:
                lead *= REF_LEAD_MAX / biggest
            target = np.empty(5, dtype=np.float32)
            target[:4] = np.clip(state[:4] + lead, self.pos_low, self.pos_high)
            target[4] = np.clip(action[4], GRIPPER_OPEN, GRIPPER_CLOSE)

            if tick % LOG_EVERY == 0:
                self.get_logger().info(f"{now - start:4.1f}秒 今 {np.round(np.degrees(state), 1)} "
                                       f"→ 目標 {np.round(np.degrees(target), 1)}（計算 {infer_ms:.0f}ms）")
            tick += 1

            if close_time is None and target[4] > CLOSE_LINE:
                close_time = now
            if close_time is not None and now - close_time >= HOLD_SEC:
                target[4] = GRIPPER_CLOSE
                self.send(target)
                return True, "手を閉じ終えた"
            self.send(target)

            next_tick += PERIOD
            if next_tick < time.monotonic():   # 計算が間に合わなかったときは、待たずに次へ
                next_tick = time.monotonic()
            time.sleep(max(0.0, next_tick - time.monotonic()))


def main():
    parser = argparse.ArgumentParser(description="画像つき ACT で茶碗をつかむサービス（/kotoba/act_grasp）")
    parser.add_argument("--model-path", default=MODEL_PATH)
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args()

    rclpy.init()
    node = ActGraspImgServer(args.model_path, args.threads)
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