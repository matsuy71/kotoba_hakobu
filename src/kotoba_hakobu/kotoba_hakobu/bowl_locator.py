"""茶碗の位置を教える部品（真の値版）：Gazebo の /world/default/pose/info を読んで、茶碗の位置と傾きを返す"""
import math
import subprocess
import time

TABLE_Z = 1.015   # Gazebo の世界での机の天板の高さ（ロボットを置いた高さ）
COMMAND = ["ign", "topic", "-e", "-t", "/world/default/pose/info", "-n", "1"]
BOWL_NAMES = {"red": "chawan_red", "black": "chawan_black"}


def parse_pose_info(text):
    """ign topic の表示（文字）から、{物の名前: {"position.x": 値, ...}} の辞書を作る。"""
    poses = {}
    name, section = None, None
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("name:"):
            name = line.split('"')[1]
            poses[name] = {}
        elif line in ("position {", "orientation {"):
            section = line.split()[0]
        elif line == "}":
            section = None
        elif section and name and ":" in line:
            key, value = line.split(":")
            poses[name][f"{section}.{key.strip()}"] = float(value)
    return poses


class TruePoseLocator:
    """Gazebo の真の値から、茶碗の位置を教える。使い方：locate("red")"""

    def locate(self, color):
        """茶碗の位置（ロボットの座標、m）と傾き（度）を辞書で返す。読めなければ None。"""
        try:
            text = subprocess.run(COMMAND, capture_output=True, text=True, timeout=5).stdout
        except subprocess.TimeoutExpired:
            return None
        pose = parse_pose_info(text).get(BOWL_NAMES[color])
        if pose is None:
            return None
        g = lambda key: pose.get(key, 0.0)   # 0 の項目は表示されないので、ないときは 0 とする
        qx, qy = g("orientation.x"), g("orientation.y")
        up = 1.0 - 2.0 * (qx ** 2 + qy ** 2)   # 茶碗の上下の軸の、上向き成分
        return {
            "x": g("position.x"),
            "y": g("position.y"),
            "z": g("position.z") - TABLE_Z,
            "tilt_deg": math.degrees(math.acos(max(-1.0, min(1.0, up)))),
        }


def main():
    locator = TruePoseLocator()
    for color in BOWL_NAMES:
        start = time.time()
        result = locator.locate(color)
        seconds = time.time() - start
        if result is None:
            print(f"{color}: 読めませんでした")
        else:
            print(f"{color}: x={result['x']:.4f} y={result['y']:.4f} z={result['z']:.4f}  "
                  f"傾き={result['tilt_deg']:.2f}度  （{seconds:.2f}秒）")


if __name__ == "__main__":
    main()