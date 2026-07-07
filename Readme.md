# Webots를 활용한 다중 로봇 시뮬레이션

## 0. 사전 요구 사항 (Prerequisites)
- 호스트 장치 운영체제 : 윈도우, Mac, **Ubuntu(리눅스, 신규 지원)**
- 도커 베이스 이미지 : Ubuntu 22.04 (ROS 2 Humble)
- ROS : humble
- Webots : 2025a

## 1. Webots 설치
시뮬레이션을 원활하게 실행하기 위해 시스템에 Webots가 설치 필요
- **Webots 다운로드**: [Cyberbotics 공식 홈페이지](https://cyberbotics.com/)에서 운영체제에 맞는 **2025a** 버전을 다운로드하여 설치
- **Webots 클라우드 공유 및 정보**: [https://webots.cloud/](https://webots.cloud/)
  - 오픈되어 있는 asset를 쓸 수 있음 

## 2. 설치 및 구성 (Installation)

본 워크스페이스는 여러 ROS 2 패키지와 **Git 서브모듈(Submodule)**을 포함 

터미널에서 다음 명령어를 통해 저장소를 클론하고 서브모듈을 초기화

```bash
# 저장소 클론 (서브모듈 포함)
git clone https://github.com/seo2730/multi_webots.git
cd multi_webots

# (이미 클론한 상태라면) 서브모듈 초기화 및 업데이트
git submodule update --init --recursive
```

## 3. 시뮬레이션 실행 방법 (Usage)
1. 설치된 **Webots** 프로그램을 실행합니다.
2. 상단 메뉴에서 `File` -> `Open World...`를 클릭합니다.
3. `multi_webots/src/Webots-SummitXL/workspace/simulator/worlds` 디렉토리 내의 `worlds` 폴더에 있는 `my_world.wbt` 월드 파일(`.wbt`)을 선택하여 엽니다.
4. 상단의 **Play** 버튼(또는 `Step` 버튼)을 눌러 시뮬레이션을 시작하고 로봇들의 동작을 확인합니다.

## 4. 로봇 추가 방법 (향후 자동화 예정)
1. `my_world.wbt` 월드 파일에서 SummitXlSteel을 복사
2. SummitXlSteel 클릭하여 하위 트리에서 name 변경 (ROS2와 연동할 때 알아서 네임스페이스 생성해줌)

---

### 1. Ubuntu (우분투 / 리눅스) — 신규 지원, 테스트 완료
Docker Engine이 설치된 우분투 데스크탑에서 바로 동작합니다 (X11 네이티브, VNC 불필요).

1. **사전 준비 (최초 1회, 또는 재부팅할 때마다):** 호스트 X 서버가 컨테이너(root)의 화면 출력을 받아주도록 허용
   ```bash
   xhost +local:root
   ```
2. `echo $DISPLAY`로 현재 디스플레이 번호를 확인 (보통 `:0` 또는 `:1`). 대부분 이미 설정되어 있어서 별도 조치 불필요.
3. Docker Engine이 없다면 [공식 문서](https://docs.docker.com/engine/install/ubuntu/)를 따라 설치 (Docker Desktop이 아니어도 됩니다).

> 우분투용 컨테이너는 도커 기본 **bridge 네트워크**를 씁니다 (`network_mode: host`가 아님). 컨테이너와 호스트를 같은 네트워크로 묶으면 FastRTPS가 "같은 머신"으로 착각해서 공유메모리(SHM) 전송을 시도하다가, 컨테이너(root)와 호스트(일반 유저)의 권한이 안 맞아 호스트에서 `ros2 topic echo`가 안 되는 문제가 있었습니다. bridge로 컨테이너마다 별도 IP를 주면 이 문제가 해결됩니다.

---

### 2. Windows (윈도우)
윈도우는 기본적으로 X11을 지원하지 않기 때문에, X 서버 역할을 해줄 외부 프로그램을 설치해야 합니다. 

1. **설치:** [VcXsrv](https://github.com/marchaesen/vcxsrv)를 깃허브에 접속하여 Release를 클릭한 뒤 최신 exe 파일을 다운받아 설치한다.
2. **실행 (XLaunch):** 시작 메뉴에서 `XLaunch`를 실행한다.
3. **설정 단계 (매우 중요):**
   * **Display settings:** `Multiple windows` 선택, Display number에 `0` 입력
   * **Client startup:** `Start no client` 선택
   * **Extra settings:** * `Clipboard`, `Primary Selection` 체크
     * `Native opengl` **체크 해제** (3D 프로그램 충돌 방지)
     * 🌟 **`Disable access control` 체크 (필수!)** -> 도커의 화면 신호를 거부하지 않고 받기 위함입니다.
4. **마무리:** 다음을 눌러 실행합니다. (작업표시줄 우측 하단 트레이에 `X` 모양 아이콘이 떠 있으면 성공입니다.)
5. **호스트 장치를 킬 때마다 계속 작동시켜줘야함**

---

### 3. macOS (맥)
현재 맥에서 X11 - rviz2 연동이 상당히 불안한 관계로 vnc로 설치

1. 브라우저에서 **http://localhost:6080** 접속 후 **vnc.html** 클릭
2. 화면 한 가운데 Connect 클릭

---

## 5. 작동 명령어

도커 관련 파일은 전부 **`docker-configs/` 아래 OS별 폴더**로 정리되어 있습니다.
```
docker-configs/
├── ubuntu/   Dockerfile, docker-compose.yml   (신규, bridge 네트워크)
├── windows/  Dockerfile, docker-compose.yml   (network_mode: host)
├── mac/      Dockerfile, docker-compose.yml   (VNC, bridge 네트워크)
└── camera-lidar/  docker-compose.yml          (카메라-라이다 데이터 수집 전용, 현재 맥 기준)
```
(예전엔 저장소 루트에 `Dockerfile`, `docker-compose.yml`, `Dockerfile_mac`, `docker-compose-mac.yaml`이 흩어져 있었는데, 지금은 전부 여기로 옮겨졌습니다.)

### 0. 도커 컨테이너 추가
로봇을 추가하려면 `docker-configs/<OS>/docker-compose.yml`에 아래처럼 서비스를 하나 더 추가하면 됨
```yaml
  # 4. UGV3 독립 컨테이너 (예시)
  [compose 이름]:
    <<: *ros-common
    container_name: [container 이름]
    environment:
      - DISPLAY=${DISPLAY:-:0}
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_LOCALHOST_ONLY=0
      - ROS_DOMAIN_ID=30
      - ROBOT_ID=[Webots에서 설정한 이름]
    command: >
      bash -c "source /ros2_ws/install/setup.bash &&
               ros2 launch webots_python single_ugv.launch.py"
    depends_on:
      - master
```

### 1. Ubuntu (우분투)
```bash
# 사전 준비 (최초 1회 / 재부팅 후)
xhost +local:root

# 도커 컨테이너 전체 시작 (rviz용, ugv1, ugv2)
docker compose -f docker-configs/ubuntu/docker-compose.yml up --build -d
# 도커 컨테이너 전체 종료
docker compose -f docker-configs/ubuntu/docker-compose.yml down

# 목표점을 주면 자율주행 시작
ros2 topic pub -1 /ugv1/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv1/map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

ros2 topic pub -1 /ugv2/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv2/map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

**호스트에서 직접 `ros2 topic list`/`echo`로 확인하고 싶다면**, 호스트 쉘의 ROS 2 환경변수도 컨테이너와 맞춰줘야 합니다 (`~/.bashrc`에 추가):
```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

### 2. Windows (윈도우)
```bash
# 도커 컨테이너 전체 시작 (rviz용, ugv1, ugv2)
docker compose -f docker-configs/windows/docker-compose.yml up --build -d
# 도커 컨테이너 전체 종료
docker compose -f docker-configs/windows/docker-compose.yml down

# visual code로 도커 컨테이너 접속하여 목표점 주면 자율주행 시작
ros2 topic pub -1 /ugv1/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv1/map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

ros2 topic pub -1 /ugv2/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv2/map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 3. macOS (맥)
```bash
# 도커 컨테이너 전체 시작
docker compose -f docker-configs/mac/docker-compose.yml up --build -d
# 도커 컨테이너 전체 종료
docker compose -f docker-configs/mac/docker-compose.yml down

# visual code로 도커 컨테이너 접속하여 목표점 주면 자율주행 시작
ros2 topic pub -1 /ugv1/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv1/map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

ros2 topic pub -1 /ugv2/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv2/map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 4. 카메라-라이다 데이터 수집 (맥 전용, 현재)
`webots_data_collection` 패키지가 담당하며, `docker-configs/camera-lidar/docker-compose.yml`을 씁니다 (같은 `docker-configs/mac/Dockerfile` 이미지를 재사용).
```bash
docker compose -f docker-configs/camera-lidar/docker-compose.yml up --build -d
```
수집된 원본 데이터는 `src/webots_data_collection/dataset_output/`에, KITTI 포맷 변환 결과는 `src/webots_data_collection/training/`에 쌓입니다. 변환은 컨테이너 안이 아니라 아래처럼 직접 실행합니다:
```bash
cd src/webots_data_collection
python3 scripts/webots2kitti.py
```

---

## 6. 외부 연동 (웹 목표점 / Gemini)
`webots_goal_bridge` 패키지가 담당합니다.
1. `web_goal_relay.py` — 웹에서 지도 클릭으로 보낸 목표점(`/web/goal_point`)을 로봇의 `goal_pose`로 중계 (동작 중)
2. `gemini_goal_assigner.py` — Gemini와 연동해 지도/위치 기반으로 다음 목표를 자동 할당 (**아직 연동 완료 안됨**, `setup.py`의 entry_point도 주석 처리되어 있음)
   - gemini api는 google ai studio에서 생성 가능 (gemini api 생성 방법은 구글링하면 나와있음)

## 7. 파이썬 파일을 추가 시 해야할 것
패키지가 목적별로 나뉘어 있으니, 새 노드가 어디에 속하는지 먼저 정하세요.

| 패키지 | 용도 |
|---|---|
| `webots_python` | 로봇 플랫폼 제어/통제 (텔레옵, 시계 브릿지 등) |
| `webots_goal_bridge` | 외부(웹, Gemini)에서 들어오는 목표점 연동 |
| `webots_data_collection` | 카메라-라이다 데이터 수집 및 변환 |

새 파일을 넣을 패키지를 고른 뒤, 그 패키지의 `setup.py`에서 `entry_points`에 아래처럼 한 줄 추가하면 됩니다.
```python
    entry_points={
        'console_scripts': [
            # 기존에 있던 노드들이 있다면 유지하고, 아래 줄을 추가하세요.
            'my_new_node = <패키지_이름>.my_new_node:main',
        ],
    },
```
완전히 새로운 목적(예: 새로운 센서 파이프라인)이라면, 기존 패키지 중 하나에 억지로 끼워넣기보다 `webots_data_collection`과 같은 구조로 새 ament_python 패키지를 하나 만드는 것을 추천합니다.

## 향후 계획
- Gemini api 연동
- Spot 추가
- Drone 추가
- 지도 생성 및 로봇 생성 자동화
- 윈도우 환경도 bridge 네트워크로 전환 테스트 (우분투에서 확인된 SHM 통신 문제가 윈도우에도 있는지 확인 필요)

## 참고 문서 (References)
- Webots 공식 사용자 가이드 (User Guide)
- Webots 공식 레퍼런스 매뉴얼 (Reference Manual)
