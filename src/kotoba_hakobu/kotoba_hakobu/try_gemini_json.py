"""試運転⑤：日本語の指示文を、Gemini で JSON（箱・置き場所）に変換する"""
import json
import os
import time
import urllib.error
import urllib.request

# ===== 設定 =====
MODEL = "gemini-3.5-flash-lite"   # 無料枠：1分15回・1日500回
PAUSE_SEC = 4                     # 1分に15回まで → 1回ごとに4秒あける
URL = f"https://generativelanguage.googleapis.com/v1beta/models/{MODEL}:generateContent"

# 答えの形（スキーマ）。この形でしか答えられないようにする
SCHEMA = {
    "type": "object",
    "properties": {
        "box": {"type": "string", "enum": ["red", "blue", "green", "unknown"],
                "description": "運ぶ箱の色。指示から分からなければ unknown"},
        "place": {"type": "string", "enum": ["left", "right", "front", "unknown"],
                  "description": "置き場所。指示から分からなければ unknown"},
    },
    "required": ["box", "place"],
}

PROMPT = """あなたはロボットアームへの指示を読み取る係です。
次の指示から、運ぶ箱の色と置き場所を取り出してください。
箱の色は red / blue / green、置き場所は left / right / front のどれかです。
指示から分からないとき、または当てはまらないときは unknown にしてください。
指示: {instruction}"""


def ask_gemini(instruction, retries=3):
    """指示文を Gemini に送り、{"box": ..., "place": ...} の辞書を返す。失敗したら None。"""
    body = {
        "contents": [{"parts": [{"text": PROMPT.format(instruction=instruction)}]}],
        "generationConfig": {
            "responseMimeType": "application/json",
            "responseJsonSchema": SCHEMA,
        },
    }
    request = urllib.request.Request(
        URL,
        data=json.dumps(body).encode("utf-8"),
        headers={"x-goog-api-key": os.environ["GEMINI_API_KEY"],
                 "Content-Type": "application/json"},
        method="POST",
    )
    for attempt in range(1, retries + 1):
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                reply = json.loads(response.read().decode("utf-8"))
            break
        except urllib.error.HTTPError as error:
            print(f"  エラー {error.code}（{attempt}回目）: {error.read().decode('utf-8')[:300]}")
            if error.code not in (429, 503) or attempt == retries:
                return None
            time.sleep(20 if error.code == 429 else 2 * attempt)   # 回数制限は長めに待つ
        except (urllib.error.URLError, TimeoutError) as error:
            print(f"  通信エラー（{attempt}回目）: {error}")
            if attempt == retries:
                return None
            time.sleep(2 * attempt)
    print(f"  答えたモデル: {reply.get('modelVersion')}")
    parts = reply["candidates"][0]["content"]["parts"]
    text = "".join(p.get("text", "") for p in parts)
    return json.loads(text)


def main():
    tests = [   # (指示文, 正解)
        ("赤い箱を右に置いて", {"box": "red", "place": "right"}),
        ("青いのを左へお願い", {"box": "blue", "place": "left"}),
        ("緑の箱、前に置いといて", {"box": "green", "place": "front"}),
        ("右に置いて", {"box": "unknown", "place": "right"}),
        ("コーヒーを入れて", {"box": "unknown", "place": "unknown"}),
    ]
    correct = 0
    for instruction, expected in tests:
        start = time.time()
        result = ask_gemini(instruction)
        seconds = time.time() - start
        if result is None:
            print(f"？ {instruction} → 答えなし（未測定。通信・混雑・回数制限）")
            continue
        ok = (result == expected)
        correct += ok
        mark = "○" if ok else "×"
        print(f"{mark} {instruction} → {result}（正解 {expected}、{seconds:.1f}秒）")
        time.sleep(PAUSE_SEC)
    print(f"正解 {correct} / {len(tests)}")


if __name__ == "__main__":
    main()