"""読み取りの評価：評価用の指示文（eval/instructions_eval.csv）を Gemini とキーワード規則の両方で読み取り、正解と比べる"""
import csv
import hashlib
import json
import os
import time
from datetime import datetime

from kotoba_hakobu.understanding import understand

EVAL_FILE = "/workspace/eval/instructions_eval.csv"
OUT_DIR = "/workspace/results"
WAIT_SEC = 4.5            # Gemini の無料の枠（1分に15回まで）を超えないための、1文ごとの待ち時間
FIELDS = ("bowl", "seat", "intent")
CATEGORIES = "ABCDE"
CLEAR_TARGET = 0.90       # 要件：はっきりした指示（A）のふるまいの正解率 90%以上


def behavior(r):
    """読み取り結果（または正解）から、ロボットのふるまいを決める（操作盤と同じ順番）。"""
    if r["intent"] == "other":
        return "断る"
    if r["bowl"] == "unknown":
        return "茶碗を聞き返す"
    if r["seat"] == "unknown":
        return "席を聞き返す"
    return f"復唱:{r['bowl']}/{r['seat']}"


def grade(answer, expected):
    """1つの読み取り結果を採点する。"""
    got, want = behavior(answer), behavior(expected)
    return {"fields_ok": all(answer[k] == expected[k] for k in FIELDS),
            "behavior_ok": got == want,
            "danger": got.startswith("復唱") and got != want,   # 間違ったまま復唱に進む
            "behavior": got, "expected_behavior": want}


def load_rows():
    """問題集を読む。先頭の見えない印（BOM）と、前後の空白は取り除く。"""
    with open(EVAL_FILE, encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        reader.fieldnames = [name.strip() for name in reader.fieldnames]
        return [{k: (v or "").strip() for k, v in row.items()} for row in reader]


def summarize(records, key):
    """Gemini（key="llm"）かキーワード規則（key="keyword"）の成績をまとめる。"""
    s = {"n": len(records),
         "behavior_ok": sum(r[key]["behavior_ok"] for r in records),
         "fields_ok": sum(r[key]["fields_ok"] for r in records),
         "danger": sum(r[key]["danger"] for r in records),
         "fallback": sum(r[key]["source"] != "llm" for r in records) if key == "llm" else 0,
         "by_category": {}}
    for c in CATEGORIES:
        rs = [r for r in records if r["category"] == c]
        if rs:
            s["by_category"][c] = {"n": len(rs), "behavior_ok": sum(r[key]["behavior_ok"] for r in rs)}
    return s


def main():
    rows = load_rows()
    with open(EVAL_FILE, "rb") as f:
        file_hash = hashlib.sha256(f.read()).hexdigest()   # どの版の問題集で評価したかの証拠
    print(f"評価用の指示文：{len(rows)}文（約{len(rows) * WAIT_SEC / 60:.0f}分かかります）")
    print("※ 指示文は表示しません。番号と結果だけを表示します。")

    records = []
    for i, row in enumerate(rows):
        expected = {k: row[k] for k in FIELDS}
        llm = understand(row["text"], use_llm=True)
        keyword = understand(row["text"], use_llm=False)
        record = {"id": row["id"], "category": row["category"], "text": row["text"], "note": row.get("note", ""),
                  "expected": expected,
                  "llm": {**llm, **grade(llm, expected)},
                  "keyword": {**keyword, **grade(keyword, expected)}}
        records.append(record)
        if llm["source"] != "llm":
            llm_mark = "予備に切替"
        else:
            llm_mark = "○" if record["llm"]["behavior_ok"] else "×"
        kw_mark = "○" if record["keyword"]["behavior_ok"] else "×"
        danger = []
        if record["llm"]["danger"]:
            danger.append("Gemini")
        if record["keyword"]["danger"]:
            danger.append("キーワード")
        note = f"  ※危ない間違い（{'・'.join(danger)}）" if danger else ""
        print(f"{row['id']:>4}（{row['category']}）  Gemini {llm_mark:<5}  キーワード {kw_mark}{note}")
        if i < len(rows) - 1:
            time.sleep(WAIT_SEC)

    summary = {key: summarize(records, key) for key in ("llm", "keyword")}
    print("\n===== 成績（ふるまいの正解）=====")
    print(f"{'':<10}{'ふるまい':>8}{'3項目':>8}{'危ない間違い':>8}  " + "  ".join(f"{c:>5}" for c in CATEGORIES))
    for key, name in (("llm", "Gemini"), ("keyword", "キーワード")):
        s = summary[key]
        cats = "  ".join(f"{s['by_category'][c]['behavior_ok']:>2}/{s['by_category'][c]['n']:<2}"
                         if c in s["by_category"] else "  -  " for c in CATEGORIES)
        print(f"{name:<10}{s['behavior_ok']:>4}/{s['n']:<4}{s['fields_ok']:>4}/{s['n']:<4}{s['danger']:>6}      {cats}")
    s = summary["llm"]
    print(f"Gemini が失敗して予備に切りかわった文：{s['fallback']}")
    a = s["by_category"].get("A")
    if a:
        rate = a["behavior_ok"] / a["n"]
        print(f"要件（はっきりした指示 A の正解率 {CLEAR_TARGET:.0%} 以上）：{rate:.0%} → "
              f"{'達成' if rate >= CLEAR_TARGET else '未達成'}")
    models = sorted({r["llm"].get("model") or "" for r in records if r["llm"]["source"] == "llm"})
    print(f"使ったモデル：{', '.join(models)}")

    os.makedirs(OUT_DIR, exist_ok=True)
    out = f"{OUT_DIR}/eval_understanding_{datetime.now():%Y%m%d_%H%M%S}.json"
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"time": datetime.now().isoformat(), "eval_file": EVAL_FILE, "eval_file_sha256": file_hash,
                   "models": models, "summary": summary, "records": records}, f, ensure_ascii=False, indent=2)
    print(f"記録：{out}（指示文を含むので、評価が終わるまで貼らないでください）")


if __name__ == "__main__":
    main()