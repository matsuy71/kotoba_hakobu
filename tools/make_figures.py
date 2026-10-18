"""結果の図を作る：results/ の評価の記録から、レポート用の図（PNG）と表（Markdown）を作る。
日本語のフォントがあれば日本語、なければ英語で描く。"""
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")                     # 画面を開かずに、ファイルにだけ描く
import matplotlib.pyplot as plt          # noqa: E402
from matplotlib import font_manager      # noqa: E402

RESULTS = "/workspace/results"
OUT = f"{RESULTS}/figures"
BLUE, ORANGE = "#2a78d6", "#eb6834"      # 比べる2つの色（色覚の違いを考えた差 24.7 で確かめ済み）
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
CATS = "ABCDE"

JA_FONTS = ("Noto Sans CJK JP", "IPAexGothic", "IPAGothic", "TakaoGothic")
FONT = next((n for n in JA_FONTS if n in {f.name for f in font_manager.fontManager.ttflist}), None)
JA = FONT is not None
if JA:
    plt.rcParams["font.family"] = FONT

TEXT = {   # （日本語, 英語）
    "cat": {"A": ("はっきり", "Clear"), "B": ("足りない", "Missing info"), "C": ("言い換え", "Paraphrase"),
            "D": ("頼みではない", "Not a request"), "E": ("言い直し", "Self-correction")},
    "t1": ("読み取りの評価：ふるまいの正解率", "Instruction understanding: correct behavior"),
    "y1": ("正解率（%）", "Correct (%)"),
    "llm": ("Gemini", "Gemini"), "keyword": ("キーワード規則", "Keyword rules"),
    "t2": ("カメラの誤差は、つかむ動きで消える", "Camera error is absorbed by the grasp"),
    "y2": ("ずれ（mm）", "Error (mm)"),
    "black": ("黒楽", "Black raku bowl"), "red": ("赤楽", "Red raku bowl"),
    "before": ("つかむ前：カメラの誤差", "Before grasp: camera error"),
    "after": ("置いたあと：本当のずれ", "After placing: true error"),
    "mean": ("平均", "mean"),
}


def t(key, sub=None):
    pair = TEXT[key][sub] if sub else TEXT[key]
    return pair[0] if JA else pair[1]


def latest(pattern):
    files = sorted(glob.glob(f"{RESULTS}/{pattern}"))
    return files[-1] if files else None


def style(ax):
    """軸と目盛りを控えめにする（データを主役にするため）。"""
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def fig_understanding(path):
    """図1：種類ごとの、ふるまいの正解率（Gemini とキーワード規則）。"""
    data = json.load(open(path, encoding="utf-8"))
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    width = 0.38
    for key, color, dx in (("llm", BLUE, -width / 2 - 0.01), ("keyword", ORANGE, width / 2 + 0.01)):
        by = data["summary"][key]["by_category"]
        cats = [c for c in CATS if c in by]
        xs = [CATS.index(c) + dx for c in cats]
        rates = [100 * by[c]["behavior_ok"] / by[c]["n"] for c in cats]
        ax.bar(xs, rates, width=width, color=color, label=t(key))
        for x, c, r in zip(xs, cats, rates):
            ax.text(x, r + 1.5, f"{by[c]['behavior_ok']}/{by[c]['n']}", ha="center", va="bottom",
                    fontsize=8, color=INK2)
    ax.set_xticks(range(len(CATS)))
    ax.set_xticklabels([f"{c}\n{t('cat', c)}" for c in CATS])
    ax.set_ylim(0, 112)
    ax.set_ylabel(t("y1"), color=INK2)
    ax.set_title(t("t1"), loc="left", color=INK, fontsize=11)
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, fontsize=9)
    style(ax)
    fig.text(0.99, 0.01, os.path.basename(path), ha="right", fontsize=6, color=INK2)
    fig.tight_layout()
    out = f"{OUT}/fig1_understanding.png"
    fig.savefig(out, dpi=200)
    plt.close(fig)
    return out


def fig_camera(path):
    """図2：つかむ前のカメラの誤差と、置いたあとの本当のずれ（1回のお出しを1つの点で）。"""
    recs = [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    top = 0.0
    for i, bowl in enumerate(("black", "red")):
        for key, color, dx, label in (("camera_error_mm", BLUE, -0.2, t("before")),
                                      ("true_error_mm", ORANGE, 0.2, t("after"))):
            vals = [r[key] for r in recs if r["bowl"] == bowl and r[key] is not None]
            if not vals:
                continue
            n = len(vals)
            xs = [i + dx + ((j % 5) - 2) * 0.025 for j in range(n)]   # 横の位置に意味はない。重ならないよう5列に散らす
            ax.scatter(xs, vals, s=30, color=color, edgecolors="white", linewidths=0.8, zorder=3,
                       label=label if i == 0 else None)
            mean = sum(vals) / n
            ax.plot([i + dx - 0.09, i + dx + 0.09], [mean, mean], color=INK, linewidth=1.5, zorder=4)
            ax.text(i + dx + 0.11, mean, f"{t('mean')} {mean:.1f}mm", va="center", fontsize=8, color=INK2)
            top = max(top, max(vals))
    ax.set_xticks([0, 1])
    ax.set_xticklabels([t("black"), t("red")])
    ax.set_xlim(-0.6, 1.75)
    ax.set_ylim(0, top + 0.8)
    ax.set_ylabel(t("y2"), color=INK2)
    ax.set_title(t("t2"), loc="left", color=INK, fontsize=11)
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, fontsize=9)
    style(ax)
    fig.text(0.99, 0.01, os.path.basename(path), ha="right", fontsize=6, color=INK2)
    fig.tight_layout()
    out = f"{OUT}/fig2_camera_error.png"
    fig.savefig(out, dpi=200)
    plt.close(fig)
    return out


def serve_table():
    """お出しする評価の表（Markdown）：記録ファイルごとに、成功率・ずれ・傾き・時間。"""
    lines = ["| 記録 | 位置の読み方 | 成功 | 本当のずれ 平均/最大(mm) | カメラの誤差 平均/最大(mm) "
             "| 傾き 平均/最大(度) | 時間 平均(秒) |", "|---|---|---|---|---|---|---|"]
    for path in sorted(glob.glob(f"{RESULTS}/eval_serve_*.jsonl")):
        recs = [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]
        ok = [r for r in recs if r["success"]]

        def mm(key, rs):
            v = [r[key] for r in rs if r[key] is not None]
            return f"{sum(v) / len(v):.1f} / {max(v):.1f}" if v else "-"

        tilt = [r["max_tilt_deg"] for r in ok]
        secs = [r["elapsed_sec"] for r in ok]
        lines.append(f"| {os.path.basename(path)} | {recs[0]['locator']} | {len(ok)}/{len(recs)} | "
                     f"{mm('true_error_mm', ok)} | {mm('camera_error_mm', recs)} | "
                     f"{sum(tilt) / len(tilt):.2f} / {max(tilt):.2f} | {sum(secs) / len(secs):.1f} |")
    out = f"{OUT}/serve_summary.md"
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return out, lines


def main():
    os.makedirs(OUT, exist_ok=True)
    print(f"図の文字：{'日本語（' + FONT + '）' if JA else '英語（日本語のフォントがないため）'}")
    understanding = latest("eval_understanding_*.json")
    camera = latest("eval_serve_camera_*.jsonl")
    if understanding:
        print("図1：", fig_understanding(understanding), "（元の記録：", os.path.basename(understanding), "）")
    if camera:
        print("図2：", fig_camera(camera), "（元の記録：", os.path.basename(camera), "）")
    out, lines = serve_table()
    print("表：", out)
    print("\n".join(lines))


if __name__ == "__main__":
    main()