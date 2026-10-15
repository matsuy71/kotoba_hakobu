# ===== 1. 土台：ROS 2 Humble（Ubuntu 22.04）＋画面の道具（RViz など） =====
FROM osrf/ros:humble-desktop

# ===== 2. apt に「質問せずに進めて」と伝える =====
ENV DEBIAN_FRONTEND=noninteractive

# ===== 3. 開発に使う基本の道具を入れる =====
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      sudo \
    && rm -rf /var/lib/apt/lists/*

# ===== 4. 普通のユーザー dev を作る（root で作業しないため） =====
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

# ===== 4.5 CRANE+ V2 と pymoveit2 を用意する =====
# ① ワークスペースを作り、② ソースを取ってくる
WORKDIR /opt/crane_ws/src
RUN git clone --depth 1 -b humble https://github.com/rt-net/crane_plus.git \
    && git clone --depth 1 https://github.com/AndrejOrsula/pymoveit2.git

# ③ 必要な部品（依存関係）を rosdep で調べて入れる
WORKDIR /opt/crane_ws
RUN apt-get update \
    && rosdep update \
    && rosdep install -r -y -i --from-paths src --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

# ④ ビルドする
RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ===== 5. 端末を開いたら ROS 2 と CRANE+ が使える状態にする =====
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/crane_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && mkdir -p /home/${USERNAME}/.claude \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.claude

# ===== 6. ここから先は dev として動く =====
USER ${USERNAME}
WORKDIR /home/${USERNAME} 