"""画像つきの手本の、データの形の決まり（grasp_img_features.py のねらい0.000版）。
茶碗の座標（environment_state)は入れない。場所は画像から読み取らせるのが、この実験の狙いのため。
手本のねらいは左寄せ0.000（指の真ん中）。左の指とのすき間を広げる実験（10/21）。"""
robot_type = "crane_plus"
repo_id = "crane_plus_grasp_img_center"
task_name = "grasp"
features = {
    "observation.state": {"dtype": "float32", "shape": (5,),
                          "names": ["joint1", "joint2", "joint3", "joint4", "hand"]},
    "observation.images.top": {"dtype": "image", "shape": (120, 160, 3),
                               "names": ["height", "width", "channel"]},
    "action": {"dtype": "float32", "shape": (5,),
               "names": ["joint1", "joint2", "joint3", "joint4", "hand"]},
}