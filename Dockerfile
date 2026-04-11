# 1. Webots 공식 이미지 사용
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 2. 필수 시스템 및 🌟 X11/Qt/OpenGL GUI 실행 필수 라이브러리 (VNC 제거)
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release net-tools iputils-ping netcat git software-properties-common wget \
    libxext6 libx11-6 libglvnd0 libgl1 libglx0 libegl1 mesa-utils x11-apps \
    libqt5gui5 libqt5x11extras5 \
    libxcb-xinerama0 libxcb-randr0 libxcb-render-util0 libxcb-shape0 libxcb-xfixes0 \
    libxcb-sync1 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    && rm -rf /var/lib/apt/lists/*

# 3. ROS 2 및 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-webots-ros2 ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-nav2-msgs ros-humble-nav2-costmap-2d ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-tf2-sensor-msgs ros-humble-tf2-geometry-msgs ros-humble-tf2-eigen \
    ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-mapping \
    ros-humble-visualization-msgs ros-humble-pcl-conversions ros-humble-pcl-ros ros-humble-pointcloud-to-laserscan \
    ros-humble-rosgraph-msgs ros-humble-cv-bridge ros-humble-image-transport ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 4. Webots 2025a 설치
RUN wget -q https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb && \
    apt-get update && apt-get install -y ./webots_2025a_amd64.deb && \
    rm webots_2025a_amd64.deb

ENV WEBOTS_HOME=/usr/local/webots
ENV PYTHONPATH=$WEBOTS_HOME/lib/controller/python:$PYTHONPATH
ENV LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH

RUN add-apt-repository ppa:borglab/gtsam-release-4.1 -y && \
    apt-get update && apt install -y libgtsam-dev libgtsam-unstable-dev

# 5. 환경 설정
WORKDIR /ros2_ws
# 호스트의 소스 코드를 도커 안으로 복사
COPY src /ros2_ws/src
# 🌟 [핵심] 이미지를 구울 때 colcon build를 아예 끝내버립니다!
RUN /bin/bash -c '. /opt/ros/humble/setup.bash && colcon build --symlink-install'
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source  /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
ENV USER=root

# 🌟 X11 화면 전송을 위한 환경 변수 (host.docker.internal을 통해 윈도우로 쏩니다)
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=host.docker.internal:0