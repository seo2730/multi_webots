# 1. Webots 공식 이미지 사용 (이미 모든 그래픽 설정이 완료됨)
FROM osrf/ros:humble-desktop

# 2. ROS 2 Humble 설치를 위한 기본 설정
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 3. ROS 2 Humble 레포지토리 추가 및 설치
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release net-tools iputils-ping netcat git software-properties-common wget

# 4. ROS 2 및 Webots 연동 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-webots-ros2 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-nav2-msgs \
    ros-humble-nav2-costmap-2d \
    ros-humble-tf2-sensor-msgs ros-humble-tf2-geometry-msgs \
    ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-mapping \
    ros-humble-visualization-msgs ros-humble-pcl-conversions ros-humble-pcl-ros ros-humble-rosgraph-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Webots controller 라이브러리 설치 (버전을 Windows Webots와 맞출 것 : 현재는 2025a)
RUN wget -q https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb && \ 
    apt-get update && \
    apt-get install -y ./webots_2025a_amd64.deb && \
    rm webots_2025a_amd64.deb

ENV WEBOTS_HOME=/usr/local/webots
ENV PYTHONPATH=$WEBOTS_HOME/lib/controller/python:$PYTHONPATH
ENV LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH

RUN add-apt-repository ppa:borglab/gtsam-release-4.1
RUN apt install -y libgtsam-dev libgtsam-unstable-dev

# 5. 환경 설정
WORKDIR /ros2_ws
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Webots 실행을 위한 기본 환경 변수
ENV USER=root