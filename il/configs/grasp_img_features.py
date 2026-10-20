"""画像つきの手本の、データの形の決まり（il/configs/grasp_features.py の画像版）。
茶碗の座標（environment_state)は入れない。場所は画像から読み取らせるのが、この実験の狙いのため。"""
robot_type = "crane_plus"
repo_id = "crane_plus_grasp_img"
task_name = "grasp"
features = {
    "observation.state": {"dtype": "float32", "shape": (5,),
                          "names": ["joint1", "joint2", "joint3", "joint4", "hand"]},
    "observation.images.top": {"dtype": "image", "shape": (120, 160, 3),
                               "names": ["height", "width", "channel"]},
    "action": {"dtype": "float32", "shape": (5,),
               "names": ["joint1", "joint2", "joint3", "joint4", "hand"]},
}