# ===== 1. 土台：ROS 2 Humble（Ubuntu 22.04）＋画面の道具（RViz など） =====
# 版を固定する（2026-10-15 に docker buildx history で確かめた番号）
FROM osrf/ros:humble-desktop@sha256:fb07245b32187d74350be25323d8ad2f8ca5c25c325759911a1eff2267a49c1e

# ===== 2. apt に「質問せずに進めて」と伝える =====
ENV DEBIAN_FRONTEND=noninteractive

# ===== 3. 開発に使う基本の道具を入れる（fonts-noto-cjk：図に日本語を書くためのフォント） =====
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      sudo \
      fonts-noto-cjk \
    && rm -rf /var/lib/apt/lists/*

# ===== 4. 普通のユーザー dev を作る（root で作業しないため） =====
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

# ===== 4.5 CRANE+ V2 と pymoveit2 を用意する =====
# ① ワークスペースを作り、② ソースを取ってきて、③ 動作を確かめた版に切りかえる
ARG CRANE_PLUS_COMMIT=87486bb21abbdcba05aba4e06e377dae95d4b5ff
ARG PYMOVEIT2_COMMIT=33314af864f86403e33f33d0c47e0c303e81630b
WORKDIR /opt/crane_ws/src
RUN git clone -b humble https://github.com/rt-net/crane_plus.git \
    && git -C crane_plus checkout ${CRANE_PLUS_COMMIT} \
    && git clone https://github.com/AndrejOrsula/pymoveit2.git \
    && git -C pymoveit2 checkout ${PYMOVEIT2_COMMIT}

# ④ 必要な部品（依存関係）を rosdep で調べて入れる（apt の部品は、配布元が最新版のみを置くため固定しない）
WORKDIR /opt/crane_ws
RUN apt-get update \
    && rosdep update \
    && rosdep install -r -y -i --from-paths src --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

# ⑤ ビルドする
RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ===== 5. 端末を開いたら ROS 2・CRANE+・自分のワークスペース・Gemini の鍵が使える状態にする =====
# 鍵そのものは、ここには書かない。devcontainer.json のボリューム（~/.secrets）に、利用者が自分で置く
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/crane_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /workspace/install/setup.bash ] && . /workspace/install/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo '[ -f "$HOME/.secrets/gemini_key" ] && . "$HOME/.secrets/gemini_key"' >> /home/${USERNAME}/.bashrc \
    && mkdir -p /home/${USERNAME}/.claude /home/${USERNAME}/.secrets \
    && chmod 700 /home/${USERNAME}/.secrets \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.claude /home/${USERNAME}/.secrets

# ===== 6. ここから先は dev として動く =====
USER ${USERNAME}
WORKDIR /home/${USERNAME}