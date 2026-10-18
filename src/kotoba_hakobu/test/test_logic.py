"""計算と判断の部分のテスト（ロボット・シミュレータ・Gemini は使わない）"""
import math

import pytest

from kotoba_hakobu.camera_locator import CAM_X, CAM_Y, CX, CY, to_table
from kotoba_hakobu.eval_serve import BLACK_DEG, GAP_DEG, MIN_APART, R_RANGE, X_MIN, make_trials
from kotoba_hakobu.eval_understanding import behavior, grade
from kotoba_hakobu.measure_color import to_pixel
from kotoba_hakobu.serve_server import SEATS
from kotoba_hakobu.teishu_console import is_yes
from kotoba_hakobu.try_serve import AIM_FORWARD, AIM_LEFT, HELD_FORWARD, HELD_LEFT, wrist_for
from kotoba_hakobu.understanding import is_valid, keyword_rules


# ===== 手首の位置の計算（wrist_for） =====
TARGETS = [xy for _, xy in SEATS.values()] + [(0.0740, -0.1586), (-0.0453, -0.1690)]


@pytest.mark.parametrize("target", TARGETS)
@pytest.mark.parametrize("forward, left", [(HELD_FORWARD, HELD_LEFT), (AIM_FORWARD, AIM_LEFT)])
def test_wrist_for_reaches_target(target, forward, left):
    """計算した手首の位置と向きから、手の中の茶碗の位置を逆算すると、行き先にぴったり戻る。"""
    wx, wy, psi_deg = wrist_for(*target, forward, left)
    psi = math.radians(psi_deg)
    bx = wx + forward * math.cos(psi) - left * math.sin(psi)
    by = wy + forward * math.sin(psi) + left * math.cos(psi)
    assert bx == pytest.approx(target[0], abs=1e-9)
    assert by == pytest.approx(target[1], abs=1e-9)


def test_wrist_for_jikyaku():
    """お次客様の席（0.24, 0）：手首は土台から約14.9cm、向きは左の指の分だけ右へ約1.67度。"""
    wx, wy, psi_deg = wrist_for(0.24, 0.0, HELD_FORWARD, HELD_LEFT)
    reach = math.sqrt(0.24 ** 2 - HELD_LEFT ** 2)       # 土台から、手の中の茶碗の「前の方向」までの距離
    assert math.hypot(wx, wy) == pytest.approx(reach - HELD_FORWARD)
    assert psi_deg == pytest.approx(-math.degrees(math.atan2(HELD_LEFT, reach)))

# ===== カメラの画像と机の上の位置（to_table・to_pixel） =====
def test_camera_center_is_below_camera():
    """画像の真ん中は、カメラの真下。"""
    assert to_table(CX, CY) == pytest.approx((CAM_X, CAM_Y))


def test_camera_directions():
    """画像の右＝ロボットの右（y がマイナス）、画像の上＝ロボットの前（x がプラス）。"""
    assert to_table(CX + 100, CY)[1] < CAM_Y
    assert to_table(CX, CY - 100)[0] > CAM_X


@pytest.mark.parametrize("u, v", [(100, 50), (320, 240), (466, 264), (600, 400)])
def test_camera_round_trip(u, v):
    """画像→机→画像と行き来すると、元のピクセルに戻る。"""
    x, y = to_table(u, v)
    assert to_pixel(x, y, 0.04) == (u, v)


def test_camera_matches_measurement():
    """10/18 に measure_color で、本当の位置から計算して色を測ったピクセルと同じになる。"""
    assert to_pixel(0.0740, -0.1586, 0.04) == (465, 264)    # 黒楽
    assert to_pixel(-0.0453, -0.1690, 0.04) == (475, 373)   # 赤楽


# ===== Gemini の答えの形の確かめ（is_valid） =====
GOOD = {"bowl": "red", "seat": "jikyaku", "intent": "serve"}


def test_is_valid_accepts_good_answer():
    assert is_valid(GOOD)


@pytest.mark.parametrize("bad", [
    {**GOOD, "bowl": "blue"},                       # 決まりにない茶碗
    {**GOOD, "seat": "yonkyaku"},                   # 決まりにない席
    {"bowl": "red", "seat": "jikyaku"},             # intent が抜けている
    None, ["red", "jikyaku", "serve"], "red",       # 辞書ではない
])
def test_is_valid_rejects_bad_answer(bad):
    assert not is_valid(bad)


# ===== キーワード規則（調整用の文だけを使う。評価用の35文は使わない） =====
@pytest.mark.parametrize("text, bowl, seat, intent", [
    ("赤いお茶碗をお次客様へお出しして", "red", "jikyaku", "serve"),
    ("赤楽をお詰め様にお願いします", "red", "sankyaku", "serve"),
    ("お次客様にお出しして", "unknown", "jikyaku", "serve"),
    ("黒楽をお四客様へ", "black", "unknown", "serve"),
    ("お菓子を配って", "unknown", "unknown", "other"),
    ("黒楽を…いえ、赤楽をお三客様に", "unknown", "sankyaku", "serve"),   # 候補が2つ → 決めない
])
def test_keyword_rules(text, bowl, seat, intent):
    assert keyword_rules(text) == {"bowl": bowl, "seat": seat, "intent": intent}


# ===== 評価の採点（behavior・grade） =====
def test_behavior_order():
    """断る → 茶碗を聞き返す → 席を聞き返す → 復唱 の順に判断する（操作盤と同じ）。"""
    assert behavior({"bowl": "red", "seat": "jikyaku", "intent": "other"}) == "断る"
    assert behavior({"bowl": "unknown", "seat": "unknown", "intent": "serve"}) == "茶碗を聞き返す"
    assert behavior({"bowl": "red", "seat": "unknown", "intent": "serve"}) == "席を聞き返す"
    assert behavior(GOOD) == "復唱:red/jikyaku"


def test_grade_danger():
    """間違ったまま復唱に進むときだけ、危ない間違いとする。"""
    ask_seat = {"bowl": "red", "seat": "unknown", "intent": "serve"}
    refuse = {"bowl": "unknown", "seat": "unknown", "intent": "other"}
    assert grade(GOOD, ask_seat)["danger"]                        # 聞き返すべきなのに復唱
    assert grade({**GOOD, "seat": "sankyaku"}, GOOD)["danger"]    # 違う席で復唱
    assert not grade(ask_seat, refuse)["danger"]                  # 間違いだが、聞き返すので危なくない
    assert grade(GOOD, GOOD) == {"fields_ok": True, "behavior_ok": True, "danger": False,
                                 "behavior": "復唱:red/jikyaku", "expected_behavior": "復唱:red/jikyaku"}


# ===== 評価のくじ（make_trials） =====
def test_make_trials_same_seed_same_result():
    assert make_trials(1018, 10) == make_trials(1018, 10)
    assert make_trials(1018, 10) != make_trials(1019, 10)


@pytest.mark.parametrize("trial", make_trials(1018, 10))
def test_make_trials_rules(trial):
    """置き場所が、決めた条件（距離・向き・間の長さ・机の上・席）を守っている。"""
    black, red = trial["black"], trial["red"]
    for x, y in (black, red):
        assert R_RANGE[0] - 1e-9 <= math.hypot(x, y) <= R_RANGE[1] + 1e-9
        assert x >= X_MIN
    tb = math.degrees(math.atan2(black[1], black[0]))
    tr = math.degrees(math.atan2(red[1], red[0]))
    assert BLACK_DEG[0] <= tb <= BLACK_DEG[1]
    assert GAP_DEG[0] - 1e-6 <= tb - tr <= GAP_DEG[1] + 1e-6   # 赤楽は黒楽より右（黒楽が先に回っても上を通らない）
    assert math.dist(black, red) >= MIN_APART
    assert trial["seats"]["black"] != trial["seats"]["red"]
    assert set(trial["seats"].values()) <= set(SEATS)


# ===== 操作盤の返事（is_yes） =====
@pytest.mark.parametrize("text, expected", [
    ("はい", True), ("はい、お願いします", True), ("お願いします", True), ("y", True), ("YES", True),
    ("いいえ", False), ("やめて", False), ("", False), ("はいいろ", True),
])
def test_is_yes(text, expected):
    assert is_yes(text) == expected