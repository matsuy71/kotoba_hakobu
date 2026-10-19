"""学習1回（8コマ分）の時間を、CPU のスレッドの数と、ACT の大きさを変えて測る。

使い方（pixi の道具箱を有効にしてから、/workspace で。ほかに重い計算をしていないときに）：
  PYTHONUNBUFFERED=1 python il/check_speed.py
"""
import importlib.util
import time

import torch
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.common.datasets.utils import dataset_to_policy_features
from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.configs.types import FeatureType, NormalizationMode

ROOT = "/workspace/il_data/lerobot_dataset/crane_plus_grasp"
FEATURES = "/workspace/il/configs/grasp_features.py"
NORM = {"VISUAL": NormalizationMode.MEAN_STD, "STATE": NormalizationMode.MEAN_STD,
        "ENV": NormalizationMode.MEAN_STD, "ACTION": NormalizationMode.MEAN_STD}


def load_batch():
    """データから8コマを取り出して、学習1回分のまとまり（バッチ）にする。"""
    dt = {"observation.state": [0.0], "observation.environment_state": [0.0],
          "action": [round(0.05 * i, 2) for i in range(16)]}
    ds = LeRobotDataset("crane_plus_grasp", root=ROOT, delta_timestamps=dt)
    batch = torch.utils.data.default_collate([ds[i] for i in range(0, 1600, 200)])
    batch["observation.state"] = batch["observation.state"][:, -1, :]
    batch["observation.environment_state"] = batch["observation.environment_state"][:, -1, :]
    return batch


def build(ffn, enc, vae):
    """ACT を作る。ffn＝層の中の計算の幅、enc＝読む係の層の数、vae＝くせをまとめる係の層の数。"""
    spec = importlib.util.spec_from_file_location("f", FEATURES)
    f = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(f)
    feats = dataset_to_policy_features(f.features)
    out = {k: v for k, v in feats.items() if v.type is FeatureType.ACTION}
    inp = {k: v for k, v in feats.items() if k not in out}
    cfg = ACTConfig(input_features=inp, output_features=out, chunk_size=16, n_action_steps=1,
                    dim_feedforward=ffn, n_encoder_layers=enc, n_vae_encoder_layers=vae,
                    normalization_mapping=NORM)
    policy = ACTPolicy(cfg, dataset_stats=LeRobotDatasetMetadata("crane_plus_grasp", root=ROOT).stats)
    policy.train()
    return policy


def step_time(policy, batch, n=3):
    """学習の計算を n 回して、いちばん速かった1回の時間（秒）を返す。"""
    opt = torch.optim.Adam(policy.parameters(), lr=1e-4)
    times = []
    for _ in range(n):
        t = time.time()
        loss, _ = policy.forward(batch)
        loss.backward()
        opt.step()
        opt.zero_grad()
        times.append(time.time() - t)
    return min(times)


def main():
    batch = load_batch()
    big = build(3200, 4, 4)
    print(f"いまの大きさ（幅3200・読む係4層・くせ4層）：数 {sum(p.numel() for p in big.parameters()) / 1e6:.1f}百万個")
    for threads in (16, 8, 4, 2):
        torch.set_num_threads(threads)
        print(f"  スレッド {threads:2d}：学習1回 {step_time(big, batch):.2f}秒")
    torch.set_num_threads(4)
    small = build(1024, 2, 2)
    print(f"小さい大きさ（幅1024・読む係2層・くせ2層）：数 {sum(p.numel() for p in small.parameters()) / 1e6:.1f}百万個")
    for threads in (8, 4, 2):
        torch.set_num_threads(threads)
        print(f"  スレッド {threads:2d}：学習1回 {step_time(small, batch):.2f}秒")


if __name__ == "__main__":
    main()