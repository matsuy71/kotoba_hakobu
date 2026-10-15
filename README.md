# kotoba_hakobu

**Listen. Confirm. Carry. Report.** — A language-driven pick-and-place robot on CRANE+ V2 (ROS 2 / Gazebo / MoveIt 2).

言葉で頼める・確認してから動く・結果を報告する運搬ロボット（Gazebo 上の CRANE+ V2）。
松尾・岩澤研究室「Physical AI 応用編1」最終課題。

## 状態
開発中（環境の構築と、MoveIt でアームを動かす試運転まで完了）

## 環境
- Windows 11 ＋ WSL2（Ubuntu 24.04）＋ Docker Desktop ＋ VS Code（Dev Containers）
- 開発コンテナ：`.devcontainer/Dockerfile`（ROS 2 Humble、Gazebo、MoveIt 2、CRANE+ V2、pymoveit2）

## 動作を確かめた版（2026-10-15 時点）
| 部品 | 版 |
|---|---|
| 土台のイメージ | `osrf/ros:humble-desktop@sha256:fb07245b32187d74350be25323d8ad2f8ca5c25c325759911a1eff2267a49c1e` |
| CRANE+ V2（rt-net/crane_plus, humble） | `87486bb21abbdcba05aba4e06e377dae95d4b5ff`（2024-12-03） |
| pymoveit2（AndrejOrsula/pymoveit2） | `33314af864f86403e33f33d0c47e0c303e81630b`（2026-10-10） |

rosdep・apt で入れる部品（MoveIt 2 など）は、配布元が最新版のみを置いているため固定していない。

## 動かし方（試運転）
1. VS Code でこのフォルダを開き、「開発コンテナー: コンテナーで再度開く」を実行する
2. 端末1：`ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py`
3. 端末2：
```
   cd /workspace
   colcon build --symlink-install --packages-select kotoba_hakobu
   source install/setup.bash
   ros2 run kotoba_hakobu try_joint_goal
```

## 出典・参考にしたもの
- [rt-net/crane_plus](https://github.com/rt-net/crane_plus)（CRANE+ V2 の ROS 2 パッケージ）
- [AndrejOrsula/pymoveit2](https://github.com/AndrejOrsula/pymoveit2)（Python から MoveIt 2 を使う部品）
- 松尾・岩澤研究室「Physical AI 応用編1」講義資料
- 書籍『ROS2とPythonで作って学ぶAIロボット入門 改訂第2版』（講談社）

## AI の利用
開発には Claude（Claude Code）を使用しています。どの部分を AI の助けで作り、どう確認したかは、レポートに記載します。

## ライセンス
MIT License（[LICENSE](LICENSE) を参照）