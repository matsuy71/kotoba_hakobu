# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Copyright 2024 Tatsuya Kamijo. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Tatsuya Kamijo (based on HuggingFace LeRobot lerobot/examples/3_train_policy.py)
#
# --- kotoba_hakobu での変更（元：matsuolab/pai_training scripts/train.py、記録 b4d6b54）---
#   1. ACT の正規化の決まりに ENV（まわりの状態 = 茶碗の場所）を足し、平均とばらつきで正規化する
#   2. 設定のファイルの場所を、コマンドの1つ目の引数で受け取る（なければ configs/train_config.yaml）
#   3. wandb のプロジェクト名を kotoba_hakobu_grasp にする
#   4. 記録のたびに、かかった時間（秒）も表示する
#   5. CPU のスレッドの数を、設定のファイル（num_threads）で決める（16 だと非常に遅かったため）

"""
Train ACT (Action Chunking with Transformer) policy on the collected dataset.

使い方（pixi の道具箱を有効にしてから、/workspace で）：
    python il/train_grasp.py il/configs/train_config.yaml
"""

import sys
import time
from pathlib import Path
import importlib.util
import wandb
import torch
import yaml
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.common.datasets.utils import dataset_to_policy_features
from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.configs.types import FeatureType, NormalizationMode

def load_config_py(config_path: Path):
    spec = importlib.util.spec_from_file_location("crane_features", config_path)
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    return config

def load_train_config(yaml_path: Path):
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)

# =====================
# Load parameters from YAML
# =====================
train_cfg = load_train_config(sys.argv[1] if len(sys.argv) > 1 else "configs/train_config.yaml")
DATASET_ROOT = Path(train_cfg["dataset_root"])
REPO_ID = train_cfg["repo_id"]
OUTPUT_DIRECTORY = Path(train_cfg["output_directory"])
CONFIG_PATH = Path(train_cfg["config_path"])
DEVICE = torch.device(train_cfg["device"])
NUM_THREADS = train_cfg.get("num_threads", 8)
TRAINING_STEPS = train_cfg["training_steps"]
LOG_FREQ = train_cfg["log_freq"]
NUM_WORKERS = train_cfg["num_workers"]
BATCH_SIZE = train_cfg["batch_size"]
CHUNK_SIZE = train_cfg["chunk_size"]
N_ACTION_STEPS = train_cfg["n_action_steps"]
LEARNING_RATE = train_cfg["learning_rate"]
DELTA_TIMESTAMPS = train_cfg["delta_timestamps"]
# =====================

def main():
    # 変更5：CPU のスレッドの数を決める
    torch.set_num_threads(NUM_THREADS)
    print(f"CPU のスレッドの数: {torch.get_num_threads()}")
    # Create a directory to store the training checkpoint.
    OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)

    config = load_config_py(CONFIG_PATH)
    features = config.features

    wandb.init(project="kotoba_hakobu_grasp", config={
        "repo_id": REPO_ID,
        "dataset_root": str(DATASET_ROOT),
        "training_steps": TRAINING_STEPS,
        "batch_size": BATCH_SIZE,
        "chunk_size": CHUNK_SIZE,
        "n_action_steps": N_ACTION_STEPS,
        "learning_rate": LEARNING_RATE,
        "device": str(DEVICE),
    })

    dataset_metadata = LeRobotDatasetMetadata(REPO_ID, root=DATASET_ROOT)
    features = dataset_to_policy_features(features)
    output_features = {key: ft for key, ft in features.items() if ft.type is FeatureType.ACTION}
    input_features = {key: ft for key, ft in features.items() if key not in output_features}
    print(f"input_features: {input_features}")
    print(f"output_features: {output_features}")

    cfg = ACTConfig(
        input_features=input_features,
        output_features=output_features,
        chunk_size=CHUNK_SIZE,
        n_action_steps=N_ACTION_STEPS,
        # 変更1：まわりの状態（ENV = 茶碗の場所）も、平均とばらつきで正規化する
        normalization_mapping={
            "VISUAL": NormalizationMode.MEAN_STD,
            "STATE": NormalizationMode.MEAN_STD,
            "ENV": NormalizationMode.MEAN_STD,
            "ACTION": NormalizationMode.MEAN_STD,
        },
    )
    policy = ACTPolicy(cfg, dataset_stats=dataset_metadata.stats)
    policy.train()
    policy.to(DEVICE)

    # Instantiate the dataset
    dataset = LeRobotDataset(REPO_ID, root=DATASET_ROOT, delta_timestamps=DELTA_TIMESTAMPS)

    optimizer = torch.optim.Adam(policy.parameters(), lr=LEARNING_RATE)
    dataloader = torch.utils.data.DataLoader(
        dataset,
        num_workers=NUM_WORKERS,
        batch_size=BATCH_SIZE,
        shuffle=True,
        pin_memory=False,
        drop_last=True,
    )

    step = 0
    done = False
    start = time.time()
    while not done:
        for batch in dataloader:
            batch = {k: (v.to(DEVICE) if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}
            # Remove the sequence_length dimension to adapt to the ACT model implementation
            batch["observation.state"] = batch["observation.state"][:, -1, :]  # (B, D)
            if "observation.environment_state" in batch:
                batch["observation.environment_state"] = batch["observation.environment_state"][:, -1, :]  # (B, D)
            if "observation.images.top" in batch:
                batch["observation.images.top"] = batch["observation.images.top"][:, -1]  # (B, 3, 120, 160)
            # action is left as is
            loss, _ = policy.forward(batch)
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()

            if step % LOG_FREQ == 0:
                # 変更4：かかった時間も表示する
                print(f"step: {step} loss: {loss.item():.3f} time: {time.time() - start:.1f}s")
                wandb.log({"loss": loss.item(), "step": step})
            step += 1
            if step >= TRAINING_STEPS:
                done = True
                break

    policy.save_pretrained(OUTPUT_DIRECTORY)
    print(f"policy saved to {OUTPUT_DIRECTORY}（{time.time() - start:.1f}秒）")
    wandb.finish()

if __name__ == "__main__":
    main()