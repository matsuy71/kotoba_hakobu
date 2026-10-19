"""手本（茶碗をつかむ）のデータの形の決まり。
講義第5回 pai_training の configs/crane_features.py（松尾研究室）を参考に、自分で書いた。"""
features = {
    # 観測：ロボットの状態（関節4つと指の、今の角度。ラジアン）
    "observation.state": {
        "dtype": "float32",
        "shape": (5,),
        "names": ["joint1", "joint2", "joint3", "joint4", "gripper"],
    },
    # 観測：まわりの状態（つかむ茶碗の場所。ロボットの座標・メートル）
    "observation.environment_state": {
        "dtype": "float32",
        "shape": (2,),
        "names": ["bowl_x", "bowl_y"],
    },
    # 行動：腕は1コマあとの関節の角度、指は命令（開く −30度・閉じる 35度）。ラジアン
    "action": {
        "dtype": "float32",
        "shape": (5,),
        "names": ["joint1", "joint2", "joint3", "joint4", "gripper"],
    },
}
robot_type = "crane_plus"
repo_id = "crane_plus_grasp"
task_name = "grasp"