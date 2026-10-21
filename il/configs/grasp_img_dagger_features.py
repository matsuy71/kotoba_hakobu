"""画像つきの手本の、データの形の決まり（ねらい0.000＋立て直し混ぜ版）。
ふつう60回（20261021_015942）＋わざとずらして立て直す29回（jitter 0.02・seed 2027）の計89回。
ずれた位置の画像と、その直し方を教材に入れる（DAgger の発想。Ross ら 2011）。"""
robot_type = "crane_plus"
repo_id = "crane_plus_grasp_img_dagger"
task_name = "grasp"
features = {
    "observation.state": {"dtype": "float32", "shape": (5,),
                          "names": ["joint1", "joint2", "joint3", "joint4", "hand"]},
    "observation.images.top": {"dtype": "image", "shape": (120, 160, 3),
                               "names": ["height", "width", "channel"]},
    "action": {"dtype": "float32", "shape": (5,),
               "names": ["joint1", "joint2", "joint3", "joint4", "hand"]},
}