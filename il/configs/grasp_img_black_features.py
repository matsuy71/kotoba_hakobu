"""画像つきの手本の、データの形の決まり（黒楽版・ねらい0.000＋立て直し混ぜ）。
ふつう60回（20261022_030954）＋わざとずらして立て直す30回（jitter 0.02・seed 3027）の計90回。
赤楽の優勝レシピ（真ん中ねらい・立て直し）をそのまま黒楽に適用。"""
robot_type = "crane_plus"
repo_id = "crane_plus_grasp_img_black"
task_name = "grasp"
features = {
    "observation.state": {"dtype": "float32", "shape": (5,),
                          "names": ["joint1", "joint2", "joint3", "joint4", "hand"]},
    "observation.images.top": {"dtype": "image", "shape": (120, 160, 3),
                               "names": ["height", "width", "channel"]},
    "action": {"dtype": "float32", "shape": (5,),
               "names": ["joint1", "joint2", "joint3", "joint4", "hand"]},
}