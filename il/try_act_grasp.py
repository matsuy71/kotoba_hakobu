#!/usr/bin/env python3
"""ACT（模倣学習）と MoveIt で、茶碗をつかめるかを試して比べる。

同じくじ（--seed）で茶碗を置き、どちらの方法でも
  はじめの姿勢・手を開く → つかむ（茶碗の上へ → 下りる → 手を閉じる） → 5cm 持ち上げる
を行い、茶碗が本当に持ち上がったかを、茶碗の高さで確かめる。
手本を集めたプログラム（kotoba_hakobu.collect_grasp_demos）の部品を使い回す。

使い方（シミュレータを起動してから、pixi の箱の外のターミナルで、/workspace で）：
  python3 il/try_act_grasp.py --method act --trials 2 --seed 1018      # 試し（ACT のサーバーも起動しておく）
  python3 il/try_act_grasp.py --method act --trials 20 --seed 1018     # 本番
  python3 il/try_act_grasp.py --method moveit --trials 20 --seed 1018  # くらべる相手
"""
import argparse
import json
import random
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor

from kotoba_hakobu import planning_scene as scene
from kotoba_hakobu.bowl_locator import TruePoseLocator
from kotoba_hakobu.collect_grasp_demos import BOWL, GRASP_EMPTY_DEG, Collector, pick_place, place_bowl
from kotoba_hakobu.try_grasp import hand_angle
from kotoba_hakobu.try_level_move import move
from kotoba_hakobu.try_pose_goal import set_gripper
from kotoba_hakobu.try_serve import (GRIPPER_OPEN_DEG, GRIPPER_CLOSE_DEG, AIM_FORWARD, AIM_LEFT,
                                     Z_ABOVE, Z_GRASP, wrist_for)
from kotoba_hakobu_interfaces.srv import ActGrasp

BOWL_Z = 0.02            # 机に置いたときの、茶碗の真ん中の高さ（m）
LIFT_MIN = 0.02          # 茶碗の真ん中がこれ以上上がったら「持ち上がった」（m）。手首は5cm上げる
SETTLE_AFTER_SEC = 1.0   # 手を閉じたあと、角度を読むまで待つ時間（手本を集めたときと同じ）
SERVICE_WAIT_SEC = 5.0   # ACT のサービスが見つかるまで待つ時間
ACT_TIMEOUT_SEC = 40.0   # ACT の返事を待つ時間（ACT のサーバーは20秒で止まる）
OUT_DIR = Path("/workspace/results")


def call_act(client, x, y):
    """ACT のサービスに茶碗の位置を渡して頼み、返事を待つ。(手を閉じ終えたか, 説明) を返す。"""
    req = ActGrasp.Request()
    req.x, req.y = float(x), float(y)
    future = client.call_async(req)
    start = time.time()
    while not future.done():
        if time.time() - start > ACT_TIMEOUT_SEC:
            return False, "ACT の返事が来ない"
        time.sleep(0.05)
    res = future.result()
    return res.success, res.message


def grasp(node, client, method, bowl, gx, gy):
    """つかむ（お出しの5〜7の段階）。(手を閉じ終えたか, 説明) を返す。"""
    if method == "act":
        return call_act(client, bowl["x"], bowl["y"])
    moved = move(node, node.arm, (gx, gy, Z_ABOVE))[0] and move(node, node.arm, (gx, gy, Z_GRASP), True)[0]
    if not moved:
        return False, "腕の動きの失敗"
    set_gripper(node, node.gripper, GRIPPER_CLOSE_DEG)
    return True, "手を閉じ終えた"


def run_trial(node, client, locator, method, x, y):
    """1回試す。結果の辞書を返す。"""
    bowl = place_bowl(locator, x, y)
    if bowl is None:
        return {"lifted": False, "reason": "茶碗を置き直せなかった"}
    node.go_home()
    set_gripper(node, node.gripper, GRIPPER_OPEN_DEG)
    gx, gy, _ = wrist_for(bowl["x"], bowl["y"], AIM_FORWARD, AIM_LEFT)

    start = time.time()
    ok, message = grasp(node, client, method, bowl, gx, gy)
    grasp_sec = round(time.time() - start, 1)
    time.sleep(SETTLE_AFTER_SEC)
    angle = hand_angle(node.gripper)
    result = {"bowl": {"x": bowl["x"], "y": bowl["y"]}, "grasp_ok": ok, "message": message,
              "grasp_sec": grasp_sec, "hand_deg": angle, "bowl_z_after": None, "lifted": False, "reason": ""}
    if not ok:
        result["reason"] = message
    elif angle is None or angle >= GRASP_EMPTY_DEG:
        result["reason"] = "何もはさめていない"
    elif not move(node, node.arm, (gx, gy, Z_ABOVE), True)[0]:
        result["reason"] = "持ち上げる動きの失敗"
    else:
        time.sleep(0.5)
        after = locator.locate(BOWL)
        if after:
            result["bowl_z_after"] = round(after["z"], 3)
            result["lifted"] = after["z"] - BOWL_Z >= LIFT_MIN
        if not result["lifted"]:
            result["reason"] = "茶碗が持ち上がらなかった"

    # 次の回のために、手を開き、上へ離れて HOME に戻る（茶碗は次の回に置き直す）
    set_gripper(node, node.gripper, GRIPPER_OPEN_DEG)
    move(node, node.arm, (gx, gy, Z_ABOVE), True)
    node.go_home()
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", choices=["act", "moveit"], required=True)
    ap.add_argument("--trials", type=int, default=2)
    ap.add_argument("--seed", type=int, default=1018)
    args = ap.parse_args()

    rclpy.init()
    node = Collector()   # 手本を集めたときと同じ、腕と手を動かすノード（名前も collect_grasp_demos のまま）
    client = node.create_client(ActGrasp, "/kotoba/act_grasp")
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(2.0)  # MoveIt とつながるのを待つ

    OUT_DIR.mkdir(exist_ok=True)
    log_path = OUT_DIR / f"try_act_grasp_{args.method}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    rng = random.Random(args.seed)
    locator = TruePoseLocator()
    n_lifted = 0
    done = 0
    try:
        if args.method == "act" and not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            print("ACT のサービス /kotoba/act_grasp が見つからない。pixi の箱のターミナルで il/act_grasp_server.py を起動してください")
            return
        scene.clear(node.arm)  # MoveIt に教えた物が残っていれば消す
        for i in range(args.trials):
            x, y = pick_place(rng)
            start = time.time()
            res = run_trial(node, client, locator, args.method, x, y)
            done += 1
            n_lifted += res["lifted"]
            res.update({"trial": i, "method": args.method, "seed": args.seed, "place": {"x": x, "y": y},
                        "seconds": round(time.time() - start, 1)})
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(res, ensure_ascii=False) + "\n")
            print(f"[{i + 1}/{args.trials}] {'持ち上がった' if res['lifted'] else '失敗：' + res['reason']}"
                  f"（茶碗 x={x:.3f} y={y:.3f}・つかむまで {res.get('grasp_sec', '-')}秒・"
                  f"手 {res.get('hand_deg')}度）")
    finally:
        print(f"{args.method}：持ち上がった {n_lifted}/{done}。記録：{log_path}")
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()