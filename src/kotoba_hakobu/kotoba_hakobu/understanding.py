"""理解ノード（の中身）：茶会の指示文を {茶碗, 席, 頼みの種類} に直す。Gemini ＋ キーワード規則の予備"""
import json
import os
import time
import urllib.error
import urllib.request

# ===== 設定 =====
MODEL = "gemini-3.5-flash-lite"   # モデル名はここだけに書く（無料枠：1分15回・1日500回）
URL = f"https://generativelanguage.googleapis.com/v1beta/models/{MODEL}:generateContent"
PAUSE_SEC = 4                     # テストで Gemini に続けて聞くときの間（1分15回まで）

BOWLS = ["red", "black", "unknown"]
SEATS = ["shokyaku", "jikyaku", "sankyaku", "unknown"]
INTENTS = ["serve", "other"]
SCHEMA = {
    "type": "object",
    "properties": {
        "bowl": {"type": "string", "enum": BOWLS},
        "seat": {"type": "string", "enum": SEATS},
        "intent": {"type": "string", "enum": INTENTS},
    },
    "required": ["bowl", "seat", "intent"],
}

PROMPT = """あなたは茶会で亭主を手伝うロボットの、指示を読み取る係です。
次の指示から「どの茶碗を」「どの席に」お出しするかを読み取ってください。

茶碗（bowl）:
- 赤いお茶碗・赤楽 → red
- 黒いお茶碗・黒楽 → black
- 指示に茶碗の言葉がない、または分からない → unknown

席（seat）。お客さまは3人で、呼び方は次のとおりです:
- お正客様・正客 → shokyaku
- お次客様・次客 → jikyaku
- お三客様・三客、お詰め様・お詰め（3人の席では、お詰めは三客と同じ方）→ sankyaku
- 指示に席の言葉がない、存在しない席、または分からない → unknown

頼みの種類（intent）:
- お茶・お茶碗をお出しする（運ぶ・置く）頼み → serve
- それ以外の頼み → other（このとき bowl と seat は unknown）

推測で埋めないでください。指示に書かれていないことは unknown にしてください。

指示: {text}"""

# キーワード規則（Gemini が答えないときの予備）
BOWL_WORDS = {"red": ["赤"], "black": ["黒"]}
SEAT_WORDS = {"shokyaku": ["正客"], "jikyaku": ["次客"], "sankyaku": ["三客", "詰め"]}
SERVE_WORDS = ["お茶", "茶碗", "出し", "運", "置"]


def ask_gemini(text, retries=3):
    """Gemini に聞く。成功したら（答えの辞書, 答えたモデル名）、失敗したら（None, None）を返す。"""
    body = {
        "contents": [{"parts": [{"text": PROMPT.format(text=text)}]}],
        "generationConfig": {"responseMimeType": "application/json", "responseJsonSchema": SCHEMA},
    }
    request = urllib.request.Request(
        URL, data=json.dumps(body).encode("utf-8"), method="POST",
        headers={"x-goog-api-key": os.environ["GEMINI_API_KEY"], "Content-Type": "application/json"})
    for attempt in range(1, retries + 1):
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                reply = json.loads(response.read().decode("utf-8"))
            break
        except urllib.error.HTTPError as error:
            if error.code not in (429, 503) or attempt == retries:
                return None, None
            time.sleep(20 if error.code == 429 else 2 * attempt)
        except (urllib.error.URLError, TimeoutError):
            if attempt == retries:
                return None, None
            time.sleep(2 * attempt)
    try:
        parts = reply["candidates"][0]["content"]["parts"]
        answer = json.loads("".join(p.get("text", "") for p in parts))
    except (KeyError, IndexError, json.JSONDecodeError):
        return None, None
    return answer, reply.get("modelVersion")


def is_valid(answer):
    """答えの形が決まりどおりかを確かめる（LLM の答えは、必ず確かめてから使う）。"""
    return (isinstance(answer, dict)
            and answer.get("bowl") in BOWLS
            and answer.get("seat") in SEATS
            and answer.get("intent") in INTENTS)


def find_one(text, words):
    """候補のうち、text に出てくるものが1つだけならそれを返す。0個、または2個以上なら unknown。"""
    found = [key for key, ws in words.items() if any(w in text for w in ws)]
    return found[0] if len(found) == 1 else "unknown"


def keyword_rules(text):
    """キーワード規則で読み取る（Gemini が答えないときの予備）。"""
    bowl = find_one(text, BOWL_WORDS)
    seat = find_one(text, SEAT_WORDS)
    serve = bowl != "unknown" or seat != "unknown" or any(w in text for w in SERVE_WORDS)
    return {"bowl": bowl, "seat": seat, "intent": "serve" if serve else "other"}


def understand(text, use_llm=True):
    """指示文を読み取る。Gemini が答えない・形がおかしいときは、キーワード規則で読む。
    戻り値には、どちらで読んだか（source）・答えたモデル名・かかった秒数も入れる。"""
    start = time.time()
    if use_llm:
        answer, model = ask_gemini(text)
        if answer is not None and is_valid(answer):
            return {**answer, "source": "llm", "model": model, "seconds": time.time() - start}
    answer = keyword_rules(text)
    return {**answer, "source": "keyword", "model": None, "seconds": time.time() - start}


# ===== 調整用のテスト（評価用の指示文は、別に用意する） =====
TESTS = [   # (指示文, 正解の bowl, seat, intent)
    ("赤いお茶碗をお次客様へお出しして", "red", "jikyaku", "serve"),
    ("黒楽をお正客様に", "black", "shokyaku", "serve"),
    ("赤楽をお詰め様にお願いします", "red", "sankyaku", "serve"),
    ("お三客様に黒いお茶碗を", "black", "sankyaku", "serve"),
    ("お次客様にお出しして", "unknown", "jikyaku", "serve"),
    ("赤いお茶碗をお出しして", "red", "unknown", "serve"),
    ("お茶をお出しして", "unknown", "unknown", "serve"),
    ("黒楽をお四客様へ", "black", "unknown", "serve"),
    ("お菓子を配って", "unknown", "unknown", "other"),
    ("コーヒーを入れて", "unknown", "unknown", "other"),
    ("赤くない方のお茶碗を、お次客様に", "black", "jikyaku", "serve"),
    ("お正客様のお隣の方に、赤楽を", "red", "jikyaku", "serve"),
    ("最後のお客様に黒楽を", "black", "sankyaku", "serve"),
    ("黒楽を…いえ、赤楽をお三客様に", "red", "sankyaku", "serve"),
    ("まだお茶はお出ししないで", "unknown", "unknown", "other"),
]


def main():
    score = {"llm": 0, "keyword": 0}
    for text, *expected in TESTS:
        results = {"llm": understand(text), "keyword": understand(text, use_llm=False)}
        line = f"{text}\n   正解 {expected}"
        for name, r in results.items():
            got = [r["bowl"], r["seat"], r["intent"]]
            ok = (got == expected)
            score[name] += ok
            label = "Gemini" if name == "llm" else "キーワード規則"
            note = "（Gemini が答えず、規則で読んだ）" if name == "llm" and r["source"] == "keyword" else ""
            line += f"\n   {'○' if ok else '×'} {label}: {got}{note}（{r['seconds']:.1f}秒）"
        print(line)
        time.sleep(PAUSE_SEC)
    print(f"\n正解数  Gemini {score['llm']}/{len(TESTS)}   キーワード規則 {score['keyword']}/{len(TESTS)}")


if __name__ == "__main__":
    main()