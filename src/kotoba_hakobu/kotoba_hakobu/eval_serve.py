"""お出しする評価：茶碗の置き方を seed で決まるくじで変えながら何度もお出しし、成功率・ずれ・傾き・時間を記録する"""
import argparse
import json
import math
import os
import random
import subprocess
import threading
import time
from collections import Counter
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from kotoba_hakobu_interfaces.action import ServeTea
from kotoba_hakobu.bowl_locator import BOWL_NAMES, TruePoseLocator
from kotoba_hakobu.camera_locator import CameraLocator
from kotoba_hakobu.serve_server import SEATS

OUT_DIR = "/workspace/results"
TABLE_Z = 1.015                 # Gazebo の中の机の高さ（m）
R_RANGE = (0.165, 0.18)         # 茶碗を置く、土台からの距離（m）
BLACK_DEG = (-70.0, -50.0)      # 黒楽を置く向き（度。マイナスはロボットの右）
GAP_DEG = (42.0, 50.0)          # 赤楽を、黒楽よりこの角度だけ右に置く（度）
X_MIN = -0.07                   # これより後ろは机の端に近すぎるので、くじを引き直す（m）
MIN_APART = 0.115               # 2つの茶碗の中心の間の、いちばん短い距離（m）
PARK = {"black": (0.0, 0.32), "red": (0.08, 0.32)}   # 置き直す前に、いったんよけておく場所
SETTLE_SEC = 2.0                # 置いたあと、茶碗が落ち着くまで待つ時間（秒）
BOWL_JA = {"black": "黒楽", "red": "赤楽"}
# 腕を動かす前に断った失敗と、置いたあとの確かめの失敗は、腕がホームにあるので続けてよい
CONTINUE_STEPS = {"注文の確かめ", "茶碗の位置を読む", "届くか確かめる", "席があいているか確かめる",
                  "置いた位置を確かめる"}
LAYERS = {"注文の確かめ": "読み取り", "茶碗の位置を読む": "読み取り", "届くか確かめる": "届く",
          "席があいているか確かめる": "席", "茶碗の上へ": "つかむ", "下ろす": "つかむ",
          "つかめたか確かめる": "つかむ", "持ち上げる": "運ぶ", "持ち上がったか確かめる": "運ぶ",
          "向きを変える": "運ぶ", "運ぶ高さへ下ろす": "運ぶ", "前へ出す": "運ぶ", "置く": "運ぶ", "離れる": "置く",
          "置いた位置を確かめる": "置く"}


def make_trials(seed, n):
    """seed で決まるくじで、n 回分の茶碗の置き方と席を作る（何度作っても同じになる）。"""
    rng = random.Random(seed)
    trials = []
    while len(trials) < n:
        rb, rr = rng.uniform(*R_RANGE), rng.uniform(*R_RANGE)
        tb = math.radians(rng.uniform(*BLACK_DEG))
        tr = tb - math.radians(rng.uniform(*GAP_DEG))
        black = (rb * math.cos(tb), rb * math.sin(tb))
        red = (rr * math.cos(tr), rr * math.sin(tr))
        seats = list(SEATS)
        black_seat = rng.choice(seats)
        red_seat = rng.choice([s for s in seats if s != black_seat])
        if min(black[0], red[0]) < X_MIN or math.dist(black, red) < MIN_APART:
            continue
        trials.append({"trial": len(trials) + 1, "black": black, "red": red,
                       "seats": {"black": black_seat, "red": red_seat}})
    return trials


def set_pose(name, x, y):
    """Gazebo の中で、茶碗をまっすぐ立てて (x, y) に置き直す。"""
    req = f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {TABLE_Z + 0.025:.4f}}}, orientation: {{w: 1}}'
    subprocess.run(["ign", "service", "-s", "/world/default/set_pose", "--reqtype", "ignition.msgs.Pose",
                    "--reptype", "ignition.msgs.Boolean", "--timeout", "2000", "--req", req],
                   capture_output=True, text=True, timeout=15)


def place_bowls(trial, truth):
    """2つの茶碗を、くじで決めた位置に置き、本当の位置で確かめる。うまく置けなければ None。"""
    for color in ("black", "red"):                 # 置き直す途中でぶつからないように、いったんよける
        set_pose(BOWL_NAMES[color], *PARK[color])
    for color in ("black", "red"):
        set_pose(BOWL_NAMES[color], *trial[color])
    time.sleep(SETTLE_SEC)
    placed = {}
    for color in ("black", "red"):
        t = truth.locate(color)
        if t is None or math.hypot(t["x"] - trial[color][0], t["y"] - trial[color][1]) > 0.005 \
                or t["tilt_deg"] > 5.0:
            return None
        placed[color] = t
    return placed


def server_locator():
    """サーバーが、真の値版（true）とカメラ版（camera）のどちらで動いているかを確かめる（3回まで試す）。"""
    for _ in range(3):
        try:
            out = subprocess.run(["ros2", "param", "get", "/serve_tea_server", "locator"],
                                 capture_output=True, text=True, timeout=30).stdout
            value = out.strip().split(":")[-1].strip()
            if value in ("true", "camera"):
                return value
        except Exception:
            pass
        time.sleep(2.0)
    return "unknown"


def fmt(value, spec):
    return "  -  " if value is None else format(value, spec)


class Evaluator(Node):
    """注文を出すノード。カメラの画像も受け取る（カメラの誤差を測るため）。"""

    def __init__(self):
        super().__init__("eval_serve")
        self.client = ActionClient(self, ServeTea, "serve_tea")
        self.camera = CameraLocator(self)

    def serve(self, bowl, seat):
        goal = ServeTea.Goal()
        goal.bowl, goal.seat = bowl, seat
        future = self.client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.1)
        handle = future.result()
        if not handle.accepted:
            return None
        result_future = handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)
        return result_future.result().result


def summarize(path):
    """記録ファイルの全部の行から、成績をまとめて表示する。"""
    if not os.path.exists(path):
        return
    recs = [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]
    if not recs:
        return
    ok = [r for r in recs if r["success"]]

    def stats(values):
        values = [v for v in values if v is not None]
        return (sum(values) / len(values), max(values)) if values else (None, None)

    true_mean, true_max = stats([r["true_error_mm"] for r in ok])
    cam_mean, cam_max = stats([r["camera_error_mm"] for r in recs])
    tilt_mean, tilt_max = stats([r["max_tilt_deg"] for r in ok])
    time_mean, _ = stats([r["elapsed_sec"] for r in ok])
    layers = Counter(r["layer"] for r in recs if not r["success"])
    print("\n===== 成績 =====")
    print(f"位置の読み方：{recs[0]['locator']}　seed：{recs[0]['seed']}　記録：{path}")
    print(f"成功：{len(ok)}/{len(recs)}（{len(ok) / len(recs):.0%}）")
    print(f"本当のずれ（成功したお出し）：平均 {fmt(true_mean, '.1f')}mm・最大 {fmt(true_max, '.1f')}mm")
    print(f"カメラの誤差（お出しする前）：平均 {fmt(cam_mean, '.1f')}mm・最大 {fmt(cam_max, '.1f')}mm")
    print(f"運搬中の傾き（成功したお出し）：平均 {fmt(tilt_mean, '.2f')}度・最大 {fmt(tilt_max, '.2f')}度")
    print(f"時間（成功したお出し）：平均 {fmt(time_mean, '.1f')}秒")
    print(f"失敗の層：{dict(layers) if layers else 'なし'}")


def main():
    parser = argparse.ArgumentParser(description="お出しする評価")
    parser.add_argument("--seed", type=int, default=1018, help="くじの番号（同じなら同じ置き方）")
    parser.add_argument("--trials", type=int, default=10, help="試行の回数（1回で2つお出しする）")
    parser.add_argument("--start", type=int, default=1, help="この番号の試行から始める（続きからやり直すとき）")
    parser.add_argument("--out", default=None, help="書き足す記録ファイル（続きからやり直すとき）")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Evaluator()
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    truth = TruePoseLocator()
    locator = server_locator()
    if locator == "unknown":
        print("サーバーがどちらの版で動いているか分かりません。サーバーが動いているか確かめてください")
        return
    os.makedirs(OUT_DIR, exist_ok=True)
    out = args.out or f"{OUT_DIR}/eval_serve_{locator}_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    print(f"位置の読み方：{locator}　seed：{args.seed}　試行：{args.start}〜{args.trials}　記録：{out}")

    try:
        if not node.client.wait_for_server(timeout_sec=10.0):
            print("お出しするサーバーにつながりません。端末2でサーバーを起動してください")
            return
        for trial in make_trials(args.seed, args.trials)[args.start - 1:]:
            placed = place_bowls(trial, truth)
            if placed is None:
                print(f"試行{trial['trial']}：茶碗をうまく置けませんでした。シミュレータを起動し直し、"
                      f"--start {trial['trial']} --out {out} で続きから行ってください")
                break
            camera_error = {}
            for color in ("black", "red"):
                c = node.camera.locate(color)
                camera_error[color] = (None if c is None else
                                       math.hypot(c["x"] - placed[color]["x"], c["y"] - placed[color]["y"]) * 1000)
            stop = False
            for index, color in enumerate(("black", "red"), start=1):
                seat = trial["seats"][color]
                seat_ja, (sx, sy) = SEATS[seat]
                result = node.serve(color, seat)
                after = truth.locate(color)
                success = bool(result and result.success)
                step = result.failed_step if result else "サーバーの返事なし"
                true_error = (math.hypot(after["x"] - sx, after["y"] - sy) * 1000
                              if after and result and (success or step == "置いた位置を確かめる") else None)
                rec = {"time": datetime.now().isoformat(), "seed": args.seed, "locator": locator,
                       "trial": trial["trial"], "serve": index, "bowl": color, "seat": seat,
                       "start_cmd": trial[color], "start_true": [placed[color]["x"], placed[color]["y"]],
                       "camera_error_mm": camera_error[color], "success": success,
                       "failed_step": "" if success else step,
                       "layer": "" if success else LAYERS.get(step, "その他"),
                       "message": result.message if result else "",
                       "reported_error_mm": result.placement_error_m * 1000 if result else None,
                       "true_error_mm": true_error,
                       "max_tilt_deg": result.max_tilt_deg if result else None,
                       "elapsed_sec": result.elapsed_sec if result else None}
                with open(out, "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                state = "成功" if success else f"失敗（{step}）"
                print(f"試行{trial['trial']:2d}-{index} {BOWL_JA[color]}→{seat_ja:<5} {state}  "
                      f"本当のずれ {fmt(true_error, '.1f')}mm  カメラの誤差 {fmt(camera_error[color], '.1f')}mm  "
                      f"傾き {fmt(rec['max_tilt_deg'], '.2f')}度  {fmt(rec['elapsed_sec'], '.0f')}秒")
                if not success and step not in CONTINUE_STEPS:
                    print(f"腕が動いたあとで止まりました。シミュレータ・橋渡し・サーバーを起動し直し、"
                          f"--start {trial['trial'] + 1} --out {out} で次の試行から続けてください")
                    stop = True
                    break
            if stop:
                break
    finally:
        summarize(out)
        executor.shutdown()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()