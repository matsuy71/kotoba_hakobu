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
import math
import random
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor

from kotoba_hakobu import planning_scene as scene
from kotoba_hakobu.bowl_locator import BOWL_NAMES, TruePoseLocator
from kotoba_hakobu.eval_serve import SETTLE_SEC, set_pose
from kotoba_hakobu.serve_server import SEATS
import kotoba_hakobu.collect_grasp_demos as cgd
from kotoba_hakobu.collect_grasp_demos import BOWL, GRASP_EMPTY_DEG, Collector, pick_place, place_bowl
from kotoba_hakobu.try_grasp import hand_angle
from kotoba_hakobu.try_level_move import move
from kotoba_hakobu.try_pose_goal import set_gripper
from kotoba_hakobu.try_serve import (GRIPPER_OPEN_DEG, GRIPPER_CLOSE_DEG, AIM_FORWARD, AIM_LEFT,
                                     Z_ABOVE, Z_GRASP, wrist_for)
from kotoba_hakobu_interfaces.srv import ActGrasp

BOWL_Z = 0.02            # 机に置いたときの、茶碗の真ん中の高さ（m）
LIFT_MIN = 0.02          # 茶碗の真ん中がこれ以上上がったら「持ち上がった」（m）。手首は5cm上げる
BUMP_CM = 1.0            # つかむ間に、茶碗がこれ（cm）より横へ動いたら「当てて動かした」（お茶がこぼれる失敗）
GRIP_MIN_DEG = 15.0      # 手がここより閉じていたら「ふち挟み」（浅いつかみ。正しい持ち方は18〜20度）
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

def bad_grip(ok, angle):
    """つかみ直すべき状態か。空振り（35度近くまで閉じた）と、ふち挟み（15度未満）を失敗扱いにする。"""
    return (not ok) or angle is None or angle >= GRASP_EMPTY_DEG or angle < GRIP_MIN_DEG

def run_trial(node, client, locator, method, x, y, aim_left, retries, fallback, other_at="park"):
    """1回試す。結果の辞書を返す。
    retries > 0 のとき（act のみ）：空振り・安全停止なら、開いて戻ってやり直す（やり直しの輪）。
    fallback が真のとき（act のみ）：やり直しても挟めなければ、控えの MoveIt が引き継ぐ（案ろ）。
    other_at：もう片方の茶碗の置き場所。park＝退避（手本・本試験と同じ）、席の名前＝本番の景色の再現。"""
    bowl = place_bowl(locator, x, y)
    if bowl is None:
        return {"lifted": False, "bumped": False, "reason": "茶碗を置き直せなかった"}
    if other_at != "park":
        # 本番では、先に提供した茶碗が席に置かれた状態で次の茶碗をつかみに行く。その景色を再現する
        sx, sy = SEATS[other_at][1]
        set_pose(BOWL_NAMES[cgd.OTHER], sx, sy)
        time.sleep(SETTLE_SEC)
    node.go_home()
    set_gripper(node, node.gripper, GRIPPER_OPEN_DEG)
    gx, gy, _ = wrist_for(bowl["x"], bowl["y"], AIM_FORWARD, aim_left)

    start = time.time()
    attempts = []
    used_fallback = False
    ok, message = grasp(node, client, method, bowl, gx, gy)
    time.sleep(SETTLE_AFTER_SEC)
    angle = hand_angle(node.gripper)
    while method == "act" and len(attempts) < retries and bad_grip(ok, angle):
        attempts.append({"ok": ok, "message": message, "hand_deg": angle})
        # 開いて、上へ離れて、はじめの姿勢へ（手本と同じ出発点に戻す）
        set_gripper(node, node.gripper, GRIPPER_OPEN_DEG)
        move(node, node.arm, (gx, gy, Z_ABOVE))
        node.go_home()
        # 茶碗が動いていても、今の場所を読み直して持ち上げ役に伝える（ACT は自分の目で見る）
        now = locator.locate(BOWL)
        if now:
            gx, gy, _ = wrist_for(now["x"], now["y"], AIM_FORWARD, aim_left)
        ok, message = grasp(node, client, method, bowl, gx, gy)
        time.sleep(SETTLE_AFTER_SEC)
        angle = hand_angle(node.gripper)
    if method == "act" and fallback and bad_grip(ok, angle):
        attempts.append({"ok": ok, "message": message, "hand_deg": angle})
        used_fallback = True
        set_gripper(node, node.gripper, GRIPPER_OPEN_DEG)
        move(node, node.arm, (gx, gy, Z_ABOVE))
        node.go_home()
        now = locator.locate(BOWL)
        if now:
            gx, gy, _ = wrist_for(now["x"], now["y"], AIM_FORWARD, aim_left)
        ok, message = grasp(node, client, "moveit", bowl, gx, gy)
        time.sleep(SETTLE_AFTER_SEC)
        angle = hand_angle(node.gripper)
    grasp_sec = round(time.time() - start, 1)
    # つかむ間に、茶碗を押して横へ動かしていないかを測る（最初に置いた場所からの合計。持ち上げる前に測る）
    after_grasp = locator.locate(BOWL)
    moved_cm = round(math.hypot(after_grasp["x"] - bowl["x"], after_grasp["y"] - bowl["y"]) * 100, 1) \
        if after_grasp else None
    bumped = moved_cm is not None and moved_cm > BUMP_CM
    # 移動後の座標も残す（どの向きへ押したか。閉じたときの引き込みなら奥向きにそろうはず）
    bowl_after = {"x": round(after_grasp["x"], 4), "y": round(after_grasp["y"], 4)} if after_grasp else None
    result = {"bowl": {"x": bowl["x"], "y": bowl["y"]}, "bowl_after": bowl_after, "aim_left": aim_left,
              "grasp_ok": ok, "message": message, "tries": len(attempts) + 1, "fallback": used_fallback,
              "attempts": attempts,
              "grasp_sec": grasp_sec, "hand_deg": angle, "moved_cm": moved_cm, "bumped": bumped,
              "bowl_z_after": None, "lifted": False, "reason": ""}
    if not ok:
        result["reason"] = message
    elif angle is None or angle >= GRASP_EMPTY_DEG:
        result["reason"] = "何もはさめていない"
    elif angle < GRIP_MIN_DEG:
        result["reason"] = "ふちをつまんだ（浅いつかみ。危ないので持ち上げない）"
    elif not (move(node, node.arm, (gx, gy, Z_ABOVE), True)[0]
              or move(node, node.arm, (gx, gy, Z_ABOVE))[0]):
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
    ap.add_argument("--aim-left", type=float, default=AIM_LEFT,
                    help="MoveIt で下りるときの、茶碗の中心の左寄せ（m）。手本の通り道の実験用。既定は本番と同じ")
    ap.add_argument("--retries", type=int, default=0,
                    help="act の空振り・安全停止のやり直し回数（やり直しの輪）。0なら今までどおり")
    ap.add_argument("--fallback", action="store_true",
                    help="act がやり直しても挟めなければ、控えの MoveIt が引き継ぐ（案ろ）")
    ap.add_argument("--bowl", choices=["red", "black"], default="red",
                    help="試す茶碗。もう片方はよけておく")
    ap.add_argument("--other-at", choices=["park", "shokyaku", "jikyaku", "sankyaku"], default="park",
                    help="もう片方の茶碗の置き場所。park=退避（手本・本試験と同じ）、席の名前=本番の景色の再現")
    ap.add_argument("--service", default="/kotoba/act_grasp",
                    help="ACT のサービス名（本番の赤楽サーバーは /kotoba/act_grasp_red）")
    args = ap.parse_args()
    global BOWL
    BOWL = args.bowl
    cgd.BOWL, cgd.OTHER = args.bowl, ("black" if args.bowl == "red" else "red")

    rclpy.init()
    node = Collector()   # 手本を集めたときと同じ、腕と手を動かすノード（名前も collect_grasp_demos のまま）
    client = node.create_client(ActGrasp, args.service)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(2.0)  # MoveIt とつながるのを待つ

    OUT_DIR.mkdir(exist_ok=True)
    log_path = OUT_DIR / f"try_act_grasp_{args.method}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    rng = random.Random(args.seed)
    locator = TruePoseLocator()
    n_lifted = 0
    n_bumped = 0
    done = 0
    print(f"試す茶碗 {BOWL}・ねらいの左寄せ aim_left={args.aim_left}（本番の既定は {AIM_LEFT}）"
          f"・やり直し {args.retries}回まで・控えのMoveIt {'あり' if args.fallback else 'なし'}")
    try:
        if args.method == "act" and not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            print("ACT のサービス /kotoba/act_grasp が見つからない。pixi の箱のターミナルで il/act_grasp_server.py か il/act_grasp_server_img.py を起動してください")
            return
        scene.clear(node.arm)  # MoveIt に教えた物が残っていれば消す
        for i in range(args.trials):
            x, y = pick_place(rng)
            start = time.time()
            res = run_trial(node, client, locator, args.method, x, y, args.aim_left, args.retries, args.fallback, args.other_at)
            res["other_at"] = args.other_at
            done += 1
            n_lifted += res["lifted"]
            n_bumped += res.get("bumped", False)
            res.update({"trial": i, "method": args.method, "seed": args.seed, "place": {"x": x, "y": y},
                        "seconds": round(time.time() - start, 1)})
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(res, ensure_ascii=False) + "\n")
            extra = ""
            if res.get("tries", 1) > 1:
                extra += f"・{res['tries']}回目でつかんだ"
            if res.get("fallback"):
                extra += "・控えのMoveItが引き継いだ"
            print(f"[{i + 1}/{args.trials}] {'持ち上がった' if res['lifted'] else '失敗：' + res['reason']}"
                  f"{'・茶碗に当てて動かした' if res.get('bumped') else ''}{extra}"
                  f"（茶碗 x={x:.3f} y={y:.3f}・茶碗のずれ {res.get('moved_cm')}cm・"
                  f"つかむまで {res.get('grasp_sec', '-')}秒・手 {res.get('hand_deg')}度）")
    finally:
        print(f"{args.method}：持ち上がった {n_lifted}/{done}・茶碗に当てて動かした {n_bumped}/{done}。記録：{log_path}")
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()