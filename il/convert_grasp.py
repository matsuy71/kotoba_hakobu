#!/usr/bin/env python3
"""手本の rosbag を、LeRobot のデータの形（LeRobotDataset v2.1）に変える。

講義第5回 pai_training の scripts/rosbag2lerobot.py（松尾研究室）を参考に、自分で書いた。
講義との違い：
  - まわりの状態（observation.environment_state）に、茶碗の場所 (x, y) を入れる（/kotoba/bowl_position）
  - 腕の行動は「1コマあと（1/fps 秒後）の関節の角度」。手本は MoveIt で動かしたので、命令のトピックがないため
  - 指の行動は、手本で出した命令（開く −30度・閉じる 35度）。指が閉じ始めたコマで切りかえる
  - 関節は、名前で選んで並べる（joint1, joint2, joint3, joint4, 指）
  - 腕が止まっていて、指の行動が「開く」のコマは取りのぞく（止まっている時間が手本ごとにばらばらで、ACT が迷ったため）
使い方（pixi の道具箱を有効にしてから、/workspace で）：
  python il/convert_grasp.py il/configs/conversion_config.yaml
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
STILL = 0.001                         # 腕の関節が、次のコマまでにこれ（ラジアン。約0.06度）より動かなければ「止まっている」
TOPIC_JOINT = "/joint_states"
TOPIC_BOWL = "/kotoba/bowl_position"


def load_py(path):
    """データの形の決まり（.py のファイル）を読みこむ。"""
    spec = importlib.util.spec_from_file_location("features", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def read_bag(ep_dir):
    """1つの手本から、(時刻, 関節5つの角度) と (時刻, 茶碗の x, y) の並びを読む。時刻はナノ秒。"""
    jt, jv, bt, bv = [], [], [], []
    with Reader(ep_dir) as reader:
        conns = [c for c in reader.connections if c.topic in (TOPIC_JOINT, TOPIC_BOWL)]
        for conn, ts, raw in reader.messages(connections=conns):
            msg = deserialize_cdr(raw, conn.msgtype)
            if conn.topic == TOPIC_JOINT:
                pos = dict(zip(msg.name, msg.position))
                if all(j in pos for j in JOINTS):
                    jt.append(ts)
                    jv.append([pos[j] for j in JOINTS])
            else:
                bt.append(ts)
                bv.append([msg.point.x, msg.point.y])
    return np.array(jt), np.array(jv), np.array(bt), np.array(bv)


def hold(times, values, targets):
    """それぞれの目標の時刻で、その時点のいちばん新しい値を取る（取って、持ちつづける）。"""
    idx = np.searchsorted(times, targets, side="right") - 1
    return values[np.clip(idx, 0, len(values) - 1)]


def episode_frames(ep_dir, fps):
    """1つの手本を、1秒間に fps コマの（状態, まわり, 行動）にする。
    腕が止まっていて、指の行動が「開く」のコマは取りのぞく。
    指が閉じ始めたコマの番号（取りのぞく前の番号）と、取りのぞいたコマの数も返す。"""
    jt, jv, bt, bv = read_bag(ep_dir)
    if len(jt) == 0 or len(bt) == 0:
        raise ValueError(f"{ep_dir}：関節の角度か茶碗の場所が記録されていない")
    start, end = max(jt[0], bt[0]), min(jt[-1], bt[-1])
    targets = np.arange(start, end, int(1e9 / fps))
    state = hold(jt, jv, targets).astype(np.float32)        # (コマ数, 5)
    env = hold(bt, bv, targets).astype(np.float32)          # (コマ数, 2)
    action = np.vstack([state[1:], state[-1:]]).copy()      # 腕：1コマあとの角度（最後のコマは同じ角度）
    closing = np.nonzero(state[:, 4] > GRIPPER_OPEN + CLOSE_START)[0]
    k = int(closing[0]) if len(closing) else len(state)
    action[:, 4] = GRIPPER_OPEN                             # 指：閉じ始めるまでは「開く」
    action[k:, 4] = GRIPPER_CLOSE                           # 指：閉じ始めたコマから「閉じる」
    still = np.abs(action[:, :4] - state[:, :4]).max(axis=1) < STILL   # 腕が止まっているコマ
    keep = ~(still & (action[:, 4] < 0))                    # 止まっていて、指が「開く」のコマ以外を残す
    return state[keep], env[keep], action[keep], k, int((~keep).sum())


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
        state, env, action, k, n_drop = episode_frames(ep, fps)
        total_drop += n_drop
        for t in range(len(state)):
            ds.add_frame({"observation.state": torch.from_numpy(state[t]),
                          "observation.environment_state": torch.from_numpy(env[t]),
                          "action": torch.from_numpy(action[t]),
                          "task": feat.task_name})
        ds.save_episode()
        print(f"[{i + 1}/{len(episodes)}] {ep.name}：{len(state)} コマ（{len(state) / fps:.1f}秒）"
              f"・止まっていて取りのぞいたコマ {n_drop}"
              f"・指が閉じ始めたのは {k} コマ目（取りのぞく前）・茶碗 x={env[0][0]:.3f} y={env[0][1]:.3f}")
    print(f"できあがり：{out}（手本 {ds.meta.total_episodes} 個・{ds.meta.total_frames} コマ・"
          f"取りのぞいたコマ {total_drop}）")


if __name__ == "__main__":
    main()