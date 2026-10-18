"""2つ頼む指示文の評価：understand_orders の読み取りを、正解（orders 列）と比べて採点する。

使い方（/workspace で）：
  python3 tools/eval_orders.py eval/orders_eval.csv --check   # 形だけ確かめる（文は表示しない）
  python3 tools/eval_orders.py eval/orders_eval.csv           # 評価する（Gemini とキーワード規則）

正解の書き方：orders 列に「茶碗>席」を | でつなぐ。言われていないところは ?。intent が other なら空。
  例：black>shokyaku|red>jikyaku　?>shokyaku|?>jikyaku　red>sankyaku
"""
import argparse
import csv
import hashlib
import json
import sys
import time
from datetime import datetime
from pathlib import Path

from kotoba_hakobu.understanding import understand_orders

BOWLS = {"red", "black"}
SEATS = {"shokyaku", "jikyaku", "sankyaku"}
WAIT = 4.5  # Gemini の無料枠（1分15回）を超えないように、1文ごとに待つ秒数
METHODS = ("llm", "keyword")
METHOD_JA = {"llm": "Gemini", "keyword": "キーワード"}


def fingerprint(path):
    """ファイルの指紋（sha256）。1文字でも変われば、まったく違う値になる。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parse_orders(cell):
    """'red>shokyaku|?>jikyaku' を [{'bowl': 'red', 'seat': 'shokyaku'}, ...] にする。"""
    orders = []
    for part in cell.split("|"):
        part = part.strip()
        if not part:
            continue
        bowl, seat = (x.strip() for x in part.split(">"))
        orders.append({"bowl": "unknown" if bowl == "?" else bowl,
                       "seat": "unknown" if seat == "?" else seat})
    return orders


def check_row(row):
    """1行の形を確かめ、問題があればその説明のリストを返す（文の中身は表示しない）。"""
    problems = []
    if not (row.get("id") or "").strip():
        problems.append("id が空")
    if not (row.get("text") or "").strip():
        problems.append("text が空")
    intent = (row.get("intent") or "").strip()
    cell = (row.get("orders") or "").strip()
    if intent not in ("serve", "other"):
        problems.append(f"intent が serve / other ではない（{intent!r}）")
    if intent == "other" and cell:
        problems.append("intent が other なのに orders が書いてある")
    if intent == "serve":
        try:
            orders = parse_orders(cell)
        except ValueError:
            return problems + ["orders の形が違う（茶碗>席 を | でつなぐ）"]
        if not 1 <= len(orders) <= 2:
            problems.append(f"orders の数が1か2ではない（{len(orders)}）")
        for o in orders:
            if o["bowl"] not in BOWLS | {"unknown"}:
                problems.append(f"茶碗の名前が違う（{o['bowl']}）")
            if o["seat"] not in SEATS | {"unknown"}:
                problems.append(f"席の名前が違う（{o['seat']}）")
    return problems


def fill(orders):
    """操作盤と同じく、2つのうち茶碗が1つだけ分かっていれば、残りの茶碗を当てる。"""
    orders = [dict(o) for o in orders]
    if len(orders) == 2:
        known = [o["bowl"] for o in orders if o["bowl"] != "unknown"]
        if len(known) == 1 and known[0] in BOWLS:
            for o in orders:
                if o["bowl"] == "unknown":
                    o["bowl"] = "black" if known[0] == "red" else "red"
    return orders


def behavior(intent, orders):
    """読み取りから、操作盤がとる動きを決める：断る／聞き返す／復唱する。"""
    if intent != "serve" or not orders:
        return "断る"
    orders = fill(orders)
    if any(o["bowl"] == "unknown" or o["seat"] == "unknown" for o in orders):
        return "聞き返す"
    bowls = [o["bowl"] for o in orders]
    seats = [o["seat"] for o in orders]
    if len(set(bowls)) < len(bowls) or len(set(seats)) < len(seats):
        return "断る"
    return "復唱する"


def key(intent, orders):
    """比べるための形。お出しする順番（言った順）は関係ないので、並べ直す。"""
    if intent != "serve":
        return ("other",)
    return tuple(sorted((o["bowl"], o["seat"]) for o in fill(orders)))


def grade(expected, got):
    """正解と読み取りを比べる。危ない＝正解と違う内容を、聞き返さずに復唱してしまう読み。"""
    exp_b = behavior(expected["intent"], expected["orders"])
    got_b = behavior(got["intent"], got["orders"])
    same = key(expected["intent"], expected["orders"]) == key(got["intent"], got["orders"])
    danger = got_b == "復唱する" and (exp_b != "復唱する" or not same)
    return {"expected_behavior": exp_b, "behavior": got_b,
            "behavior_ok": exp_b == got_b and not danger, "exact": same, "danger": danger}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--check", action="store_true", help="形だけ確かめて終わる")
    args = ap.parse_args()

    with open(args.csv, encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    fp = fingerprint(args.csv)
    print(f"問題集：{args.csv}（{len(rows)}文）")
    print(f"指紋 sha256：{fp}")

    ids = [(r.get("id") or "").strip() for r in rows]
    bad = 0
    for r, i in zip(rows, ids):
        problems = check_row(r)
        if i and ids.count(i) > 1:
            problems.append("id が重なっている")
        if problems:
            bad += 1
            print(f"  {i or '（id なし）'}：" + "／".join(problems))
    if bad:
        print(f"形の問題が {bad} 行あります。直してから、もう一度動かしてください。")
        sys.exit(1)
    print("形はすべて正しいです。")
    if args.check:
        return

    out = Path("results") / f"eval_orders_{Path(args.csv).stem}_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    totals = {m: {"behavior_ok": 0, "exact": 0, "danger": 0, "fallback": 0} for m in METHODS}
    with open(out, "w", encoding="utf-8") as f:
        f.write(json.dumps({"csv": args.csv, "sha256": fp, "n": len(rows),
                            "time": datetime.now().isoformat()}, ensure_ascii=False) + "\n")
        for r, i in zip(rows, ids):
            intent = r["intent"].strip()
            expected = {"intent": intent,
                        "orders": parse_orders(r["orders"]) if intent == "serve" else []}
            marks = []
            for m in METHODS:
                got = understand_orders(r["text"], use_llm=(m == "llm"))
                g = grade(expected, {"intent": got.get("intent"), "orders": got.get("orders", [])})
                t = totals[m]
                t["behavior_ok"] += g["behavior_ok"]
                t["exact"] += g["exact"]
                t["danger"] += g["danger"]
                if m == "llm" and got.get("source") != "llm":
                    t["fallback"] += 1
                f.write(json.dumps({"id": i, "method": m, "expected": expected, "got": got, **g},
                                   ensure_ascii=False) + "\n")
                mark = "○" if g["behavior_ok"] else ("危" if g["danger"] else "×")
                marks.append(f"{METHOD_JA[m]} {mark}（{g['behavior']}）")
                if m == "llm":
                    time.sleep(WAIT)
            print(f"  {i}：正解は{behavior(expected['intent'], expected['orders'])}　" + "　".join(marks))
        f.write(json.dumps({"summary": totals}, ensure_ascii=False) + "\n")

    n = len(rows)
    print("\n＝ まとめ ＝")
    for m in METHODS:
        t = totals[m]
        line = (f"{METHOD_JA[m]}：動きが正しい {t['behavior_ok']}/{n}　"
                f"読みが正解と同じ {t['exact']}/{n}　危ない {t['danger']}/{n}")
        if m == "llm":
            line += f"　（Gemini が答えられずキーワードに切りかわった {t['fallback']}）"
        print(line)
    print(f"記録：{out}")


if __name__ == "__main__":
    main()