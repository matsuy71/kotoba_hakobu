"""指の形のデータ（STL）から、手首（crane_plus_link4）から見た指の位置を測る。読むだけで何も動かさない。"""
import math
import sys
from pathlib import Path

import numpy as np

MESH_DIR = Path(sys.argv[1])
HAND_ORIGIN = np.array([0.0, -0.0145, 0.045])   # URDF：手首から見た、動く指の関節の位置（m）


def read_stl(path):
    """STL ファイルを読み、すべての頂点の座標（m）を (N, 3) の表で返す。"""
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    if 84 + count * 50 == len(data):    # バイナリ形式（三角形1つが50バイト）
        rows = np.frombuffer(data[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")]))
        return rows["v"].reshape(-1, 3).astype(float)
    lines = data.decode("ascii", errors="ignore").splitlines()   # 文字（ASCII）形式
    return np.array([[float(x) for x in s.split()[1:4]] for s in lines if s.strip().startswith("vertex")])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def y_range(pts, z_lo, z_hi):
    """z が z_lo〜z_hi（mm）の輪切りの中で、y の範囲（mm）を文字で返す。点がなければ「―」。"""
    band = pts[(pts[:, 2] * 1000 >= z_lo) & (pts[:, 2] * 1000 < z_hi)]
    if len(band) == 0:
        return f"{'―':>15s}"
    return f"{band[:, 1].min() * 1000:6.1f}〜{band[:, 1].max() * 1000:6.1f}"


def main():
    link4 = read_stl(next(MESH_DIR.glob("Link4_collision_*.stl"))) @ rot_x(math.pi / 2).T
    hand_raw = read_stl(next(MESH_DIR.glob("Hand_collision_*.stl"))) @ rot_y(-math.pi / 2).T
    hands = {deg: hand_raw @ rot_x(-math.radians(deg)).T + HAND_ORIGIN for deg in (-30, 10)}

    print("=== 手首から見た、1cm ごとの輪切り。各部品の y の範囲（mm）。y＝指が開閉する方向 ===")
    print(f"{'z（mm）':>10s} {'手首の部品':>15s} {'動く指(開-30°)':>15s} {'動く指(閉+10°)':>15s}")
    for z_lo in range(40, 140, 10):
        print(f"{z_lo:4d}〜{z_lo + 10:3d} "
              f"{y_range(link4, z_lo, z_lo + 10):>15s} "
              f"{y_range(hands[-30], z_lo, z_lo + 10):>15s} "
              f"{y_range(hands[10], z_lo, z_lo + 10):>15s}")


if __name__ == "__main__":
    main()