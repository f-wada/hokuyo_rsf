# --- Stage 1: Build Stage ---
FROM ubuntu:20.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# 必要なツールとROS 1リポジトリのセットアップ
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates git \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-rosdep python3-catkin-tools \
    ros-noetic-ros-base \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# ワークスペース構築
WORKDIR /ws/src
COPY . /ws/src/hokuyo_rsf
RUN git clone --depth 1 https://github.com/hokuyo-rd-release/nmea_msgs.git

# 依存関係の解決とビルド
WORKDIR /ws
RUN rosdep init || true && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO && \
    . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build hokuyo_rsf

# --- Stage 2: Runtime Stage (Minimal) ---
FROM ubuntu:20.04

ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1

# 実行環境でもROSリポジトリを登録し、必要なランタイムパッケージのみをインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    && apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*

# ビルド成果物をコピー
COPY --from=builder /ws/devel /ws/devel

# ROS環境設定
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /ws
