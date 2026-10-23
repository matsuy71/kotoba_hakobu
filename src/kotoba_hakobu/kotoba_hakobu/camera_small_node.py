"""カメラの縮め係：元画像（640×480）を受け取り、手本の記録と同じ縮め方（4画素ごとに
1画素）で 160×120 にして /kotoba/camera_small に配信する。

大きい画像は配達の途中で欠けやすく、推論で忙しい ACT サーバーは受け取り損ねることがある
（2026-10-23 に判明した統合の欠陥3号）。縮め係を独立の部品にして、小さくて欠けにくい
画像を ACT サーバーに食べさせる（本番採用の試験のときと同じ構成に戻す）。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

RAW = "/chakai_camera/image"     # 元画像（640×480）
SMALL = "/kotoba/camera_small"   # 縮めた画像（160×120）
SCALE = 4                        # 縦横とも4画素ごとに1画素


class CameraSmall(Node):
    def __init__(self):
        super().__init__("camera_small_node")
        self.pub = self.create_publisher(Image, SMALL, qos_profile_sensor_data)
        self.create_subscription(Image, RAW, self.on_image, qos_profile_sensor_data)
        self.count = 0
        self.get_logger().info(f"縮め係を起動：{RAW}（640×480）→ {SMALL}（160×120）")

    def on_image(self, msg):
        if msg.encoding != "rgb8":
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        small = arr[::SCALE, ::SCALE]
        out = Image()
        out.header = msg.header
        out.height, out.width = small.shape[0], small.shape[1]
        out.encoding = "rgb8"
        out.step = out.width * 3
        out.data = small.tobytes()
        self.pub.publish(out)
        self.count += 1
        if self.count % 500 == 0:
            self.get_logger().info(f"{self.count} 枚目を縮めて配信")


def main():
    rclpy.init()
    node = CameraSmall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()