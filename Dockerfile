# 1. Webots 공식 이미지 사용
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 2. 필수 시스템 및 GUI 라이브러리 (X11 기초)
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release net-tools iputils-ping netcat git software-properties-common wget \
    libxext6 libx11-6 libglvnd0 libgl1 libglx0 libegl1 mesa-utils x11-apps \
    && rm -rf /var/lib/apt/lists/*

# 3. 🌟 VNC, 가상 모니터(Xvfb), 가벼운 윈도우 매니저(Fluxbox) 설치
RUN apt-get update && apt-get install -y \
    xvfb x11vnc fluxbox novnc websockify \
    && rm -rf /var/lib/apt/lists/*

# 4. ROS 2 및 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-webots-ros2 ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-nav2-msgs ros-humble-nav2-costmap-2d ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-tf2-sensor-msgs ros-humble-tf2-geometry-msgs ros-humble-tf2-eigen \
    ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-mapping \
    ros-humble-visualization-msgs ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-rosgraph-msgs ros-humble-cv-bridge ros-humble-image-transport \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 5. Webots 2025a 설치
RUN wget -q https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb && \
    apt-get update && apt-get install -y ./webots_2025a_amd64.deb && \
    rm webots_2025a_amd64.deb

ENV WEBOTS_HOME=/usr/local/webots
ENV PYTHONPATH=$WEBOTS_HOME/lib/controller/python:$PYTHONPATH
ENV LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH

RUN add-apt-repository ppa:borglab/gtsam-release-4.1 -y && \
    apt-get update && apt install -y libgtsam-dev libgtsam-unstable-dev

# 6. 환경 설정 및 VNC 시작 스크립트 만들기
WORKDIR /ros2_ws
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENV USER=root

# 🌟 VNC 시작 스크립트 작성
RUN echo '#!/bin/bash\n\
export DISPLAY=:0\n\
# 1. 가상 모니터 켜기 (FHD 해상도)\n\
Xvfb :0 -screen 0 1920x1080x24 &\n\
sleep 2\n\
# 2. 창 관리자(드래그 등 가능하게) 켜기\n\
fluxbox &\n\
# 3. VNC 서버 켜기 (비밀번호 없음)\n\
x11vnc -display :0 -forever -nopw -bg -xkb &\n\
# 4. 웹브라우저용 Websocket 연결 켜기\n\
websockify --web=/usr/share/novnc/ 6080 localhost:5900 &\n\
echo "====================================================="\n\
echo "🚀 VNC 서버 부팅 완료!"\n\
echo "🌐 윈도우 크롬 창에서 http://localhost:6080 접속하세요!"\n\
echo "====================================================="\n\
exec bash' > /start_vnc.sh && chmod +x /start_vnc.sh

EXPOSE 5900 6080