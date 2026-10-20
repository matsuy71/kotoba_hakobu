#!/usr/bin/env python3
"""カメラ画像つきの手本の rosbag を、LeRobot のデータの形（LeRobotDataset v2.1）に変える。

il/convert_grasp.py（座標版）をもとにした画像版。座標版との違い：
  - 茶碗の座標（まわりの状態）は入れず、代わりに天井カメラの画像（160×120）を入れる
  - 画像は約2Hzでしか届かないので、20Hzの各コマには「その時点でいちばん新しい画像」を当てる
  - 腕の行動は「次のコマまでに動く角度（差）」、止まっているコマの除去は座標版と同じ
使い方（pixi の箱の中で、/workspace で）：
  python il/convert_grasp_img.py il/configs/conversion_img_config.yaml
"""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import torch
import yaml
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4",
          "crane_plus_joint_hand"]
GRIPPER_OPEN = math.radians(-30.0)    # 手本で、指を開いた命令
GRIPPER_CLOSE = math.radians(35.0)    # 手本で、指を閉じた命令
CLOSE_START = math.radians(2.0)       # 開いた角度からこれだけ閉じたら「閉じ始めた」とみなす
STILL = 0.001                         # 腕の関節が、次のコマまでにこれ（ラジアン）より動かなければ「止まっている」
TOPIC_JOINT = "/joint_states"
TOPIC_CAM = "/kotoba/camera_small"


def load_py(path):
    """データの形の決まり（.py のファイル）を読みこむ。"""
    spec = importlib.util.spec_from_file_location("features", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def read_bag(ep_dir):
    """1つの手本から、(時刻, 関節5つの角度) と (時刻, 画像) の並びを読む。時刻はナノ秒。"""
    jt, jv, ct, cv = [], [], [], []
    with Reader(ep_dir) as reader:
        conns = [c for c in reader.connections if c.topic in (TOPIC_JOINT, TOPIC_CAM)]
        for conn, ts, raw in reader.messages(connections=conns):
            msg = deserialize_cdr(raw, conn.msgtype)
            if conn.topic == TOPIC_JOINT:
                pos = dict(zip(msg.name, msg.position))
                if all(j in pos for j in JOINTS):
                    jt.append(ts)
                    jv.append([pos[j] for j in JOINTS])
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                ct.append(ts)
                cv.append(img)
    return np.array(jt), np.array(jv), np.array(ct), np.array(cv)


def hold(times, values, targets):
    """それぞれの目標の時刻で、その時点のいちばん新しい値を取る（取って、持ちつづける）。"""
    idx = np.searchsorted(times, targets, side="right") - 1
    return values[np.clip(idx, 0, len(values) - 1)]


def episode_frames(ep_dir, fps):
    """1つの手本を、1秒間に fps コマの（状態, 画像, 行動）にする。
    腕が止まっていて、指の行動が「開く」のコマは取りのぞく。"""
    jt, jv, ct, cv = read_bag(ep_dir)
    if len(jt) == 0 or len(ct) == 0:
        raise ValueError(f"{ep_dir}：関節の角度かカメラの画像が記録されていない")
    start, end = max(jt[0], ct[0]), min(jt[-1], ct[-1])
    targets = np.arange(start, end, int(1e9 / fps))
    state = hold(jt, jv, targets).astype(np.float32)        # (コマ数, 5)
    imgs = hold(ct, cv, targets)                            # (コマ数, 120, 160, 3)
    action = np.vstack([state[1:], state[-1:]]).copy()      # まず、1コマあとの角度
    action[:, :4] -= state[:, :4]                           # 腕：1コマあとまでに動く角度（差）にする
    closing = np.nonzero(state[:, 4] > GRIPPER_OPEN + CLOSE_START)[0]
    k = int(closing[0]) if len(closing) else len(state)
    action[:, 4] = GRIPPER_OPEN                             # 指：閉じ始めるまでは「開く」
    action[k:, 4] = GRIPPER_CLOSE                           # 指：閉じ始めたコマから「閉じる」
    still = np.abs(action[:, :4]).max(axis=1) < STILL       # 腕が止まっているコマ
    keep = ~(still & (action[:, 4] < 0))
    return state[keep], imgs[keep], action[keep], k, int((~keep).sum())


def main():
    cfg = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
    feat = load_py(cfg["config"])
    fps = cfg.get("target_freq", 20)
    out = Path(cfg["output_dir"])
    if out.exists():
        print(f"{out} はもうあります。作り直すなら、先にフォルダの名前を変えるか、別の output_dir にしてください。")
        sys.exit(1)
    episodes = sorted(p.parent for p in Path(cfg["bag_dir"]).rglob("*.db3"))
    print(f"手本 {len(episodes)} 個：{cfg['bag_dir']}")
    ds = LeRobotDataset.create(repo_id=feat.repo_id, fps=fps, root=out, robot_type=feat.robot_type,
                               features=feat.features, use_videos=False)
    total_drop = 0
    for i, ep in enumerate(episodes):
        state, imgs, action, k, n_drop = episode_frames(ep, fps)
        total_drop += n_drop
        for t in range(len(state)):
            ds.add_frame({"observation.state": torch.from_numpy(state[t]),
                          "observation.images.top": imgs[t],
                          "action": torch.from_numpy(action[t]),
                          "task": feat.task_name})
        ds.save_episode()
        print(f"[{i + 1}/{len(episodes)}] {ep.name}：{len(state)} コマ（{len(state) / fps:.1f}秒）"
              f"・止まっていて取りのぞいたコマ {n_drop}・指が閉じ始めたのは {k} コマ目（取りのぞく前）")
    print(f"できあがり：{out}（手本 {ds.meta.total_episodes} 個・{ds.meta.total_frames} コマ・"
          f"取りのぞいたコマ {total_drop}）")


if __name__ == "__main__":
    main()