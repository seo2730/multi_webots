# Webots를 활용한 다중 로봇 시뮬레이션

## 목차
- [0. 사전 요구 사항 (Prerequisites)](#0-사전-요구-사항-prerequisites)
- [1. Webots 설치](#1-webots-설치)
- [2. 설치 및 구성 (Installation)](#2-설치-및-구성-installation)
- [3. 시뮬레이션 실행 방법 (Usage)](#3-시뮬레이션-실행-방법-usage)
- [4. 로봇 추가 방법](#4-로봇-추가-방법)
- [5. 도커 컨테이너 화면(GUI) 띄우기 — OS별 사전 준비](#5-도커-컨테이너-화면gui-띄우기--os별-사전-준비)
  - [5-1. Ubuntu (우분투 / 리눅스)](#5-1-ubuntu-우분투--리눅스--신규-지원-테스트-완료)
  - [5-2. Windows (윈도우)](#5-2-windows-윈도우)
  - [5-3. macOS (맥)](#5-3-macos-맥)
- [6. 실행 명령어 (Docker Compose)](#6-실행-명령어-docker-compose)
  - [6-1. OS별 실행 명령어](#6-1-os별-실행-명령어)
  - [6-2. 공통 명령어 (모든 OS 동일)](#6-2-공통-명령어-모든-os-동일)
- [7. 로봇 위치 및 맵 데이터](#7-로봇-위치-및-맵-데이터)
- [8. 외부 연동 (웹 목표점 / Gemini)](#8-외부-연동-웹-목표점--gemini)
  - [8-1. 맵/목표점 데이터 규격 (웹 개발자용)](#8-1-맵목표점-데이터-규격-웹-개발자용)
  - [8-2. Windows 네트워킹 참고사항 (웹 개발자용)](#8-2-windows-네트워킹-참고사항-웹-개발자용)
- [9. 파이썬 파일을 추가 시 해야할 것](#9-파이썬-파일을-추가-시-해야할-것)
- [10. Spot (사족보행 로봇)](#10-spot-사족보행-로봇)
  - [10-1. 사전 준비 (서브모듈 + 월드 설정)](#10-1-사전-준비-서브모듈--월드-설정)
  - [10-2. 실행](#10-2-실행)
  - [10-3. 센서 구성 (라이다 없음 → 뎁스카메라 5개 병합)](#10-3-센서-구성-라이다-없음--뎁스카메라-5개-병합)
  - [10-4. cmd_vel 사용법 (UGV와 다름, 주의)](#10-4-cmd_vel-사용법-ugv와-다름-주의)
  - [10-5. 자세 제어 서비스](#10-5-자세-제어-서비스)
  - [10-6. 해결된 이슈 (트러블슈팅 기록)](#10-6-해결된-이슈-트러블슈팅-기록)
  - [10-7. UGV와 다른 점 / Spot에서 고려해야 할 사항](#10-7-ugv와-다른-점--spot에서-고려해야-할-사항)
- [11. Drone (중형급 쿼드콥터)](#11-drone-중형급-쿼드콥터)
  - [11-1. 실행](#11-1-실행)
  - [11-2. cmd_vel 사용법 (UGV와 다름, 주의)](#11-2-cmd_vel-사용법-ugv와-다름-주의)
  - [11-3. 기체 구성 (Mavic 2 Pro 개조)](#11-3-기체-구성-mavic-2-pro-개조)
  - [11-4. 해결된 이슈 (트러블슈팅 기록)](#11-4-해결된-이슈-트러블슈팅-기록)
  - [11-5. 고도 회피 (2.5D 레이어드)](#11-5-고도-회피-25d-레이어드)
  - [11-6. 알려진 한계 / 다음 작업](#11-6-알려진-한계--다음-작업)
- [12. 로봇 소환 (Runtime Spawn)](#12-로봇-소환-runtime-spawn)
  - [12-1. 소환하기](#12-1-소환하기)
  - [12-2. 편대 매니페스트](#12-2-편대-매니페스트)
  - [12-2-1. 새 월드를 만들어 편대를 올리기까지](#12-2-1-새-월드를-만들어-편대를-올리기까지)
  - [12-3. 구조 요약 (몸 / 뇌 / 컨테이너)](#12-3-구조-요약-몸--뇌--컨테이너)
  - [12-4. 자주 걸리는 것](#12-4-자주-걸리는-것)
- [향후 계획](#향후-계획)
- [참고 문서 (References)](#참고-문서-references)

### 📖 문서 지도

이 Readme는 **설치·실행·전체 그림**을 다룬다. 주제별 깊은 내용은 아래 문서에 있다.

| 문서 | 다루는 것 | 언제 보나 |
|---|---|---|
| **[INTERFACES.md](INTERFACES.md)** | 토픽·서비스·프레임·환경변수 **총람** | "무엇을 보내면 무엇이 나오나"가 궁금할 때. 웹/외부 연동 개발자 1순위 |
| [WORLD_GEN.md](WORLD_GEN.md) | 월드(환경) 만들기 4종 + OS별 명령어 | 새 지형이 필요할 때 |
| [SPAWNER.md](SPAWNER.md) | 로봇 소환 구축 기록 (몸/뇌 분리, 기동 순서, 잔여 몸) | 소환이 뜻대로 안 될 때, 새 로봇 종류를 더할 때 |
| [MAP_MERGE.md](MAP_MERGE.md) | 여러 로봇 지도를 `/map_merged`로 합치기 | 관제 화면·전역 좌표를 다룰 때 |
| [ugv_setup.md](ugv_setup.md) | **UGV(SummitXL)** 구성 — 메카넘·라이다·SLAM·Nav2 | 기준 로봇의 동작을 알아야 할 때 |
| [spot_driver_functions.md](spot_driver_functions.md) | **Spot** 드라이버 함수 설명서 + 자율주행 테스트 절차 | 사족보행 제어를 손볼 때 |
| [SPOT_NAV.md](SPOT_NAV.md) | **Spot 자율주행 튜닝 기록** (cmd_vel 단위·보행 케이던스·측정 함정) | Spot이 경로를 못 따라갈 때 |
| [drone_setup.md](drone_setup.md) | **드론** 기체 구성과 비행 제어 | 비행 거동·게인을 손볼 때 |
| [DRONE_NAV.md](DRONE_NAV.md) | **드론 자율비행** 구조 + 직접 테스트 방법 | 목표점을 주거나 고도 회피를 손볼 때 |
| [SPOT_NAV.md](SPOT_NAV.md) | **Spot 자율주행** 튜닝 기록 + 이슈/해결 모음 | Spot 속도·보행·Nav2 파라미터를 손볼 때 |
| [DATA_COLLECTION.md](DATA_COLLECTION.md) | 카메라-라이다 수집 → KITTI 변환 | 학습용 데이터셋을 만들 때 |

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
1. 설치된 **Webots** 프로그램 실행
2. 상단 메뉴에서 `File` -> `Open World...` 클릭
3. `multi_webots/src/Webots-SummitXL/workspace/simulator/worlds` 디렉토리 내의 `worlds` 폴더에 있는 `my_world.wbt` 월드 파일(`.wbt`) 선택해서 열기
4. 상단의 **Play** 버튼(또는 `Step` 버튼)을 눌러 시뮬레이션 시작 후 로봇들의 동작 확인

## 4. 로봇 추가 방법
**월드 파일을 편집하지 않는다.** 실행 중인 Webots에 서비스 호출로 소환한다.

```bash
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'ugv', random: true}"
```

`my_world.wbt`에는 로봇이 하나도 없다 — 환경(아레나·벽·가구)과 소환 전담 노드
(`spawn_supervisor`)만 있다. 편대는 yaml로 정의한다.
자세한 내용은 [12. 로봇 소환](#12-로봇-소환-runtime-spawn) 참고.

---

## 5. 도커 컨테이너 화면(GUI) 띄우기 — OS별 사전 준비

rviz2 같은 GUI 프로그램은 컨테이너 안에서 실행되기 때문에, **호스트 화면으로 그 화면을 받아오는 방법**이 OS마다 다름. 아래는 "화면을 띄우기 위한 사전 준비"만 다루고, 실제 `docker compose` 실행 명령어는 [6. 실행 명령어](#6-실행-명령어-docker-compose)에 따로 모아둠.

### 5-1. Ubuntu (우분투 / 리눅스) — 신규 지원, 테스트 완료
Docker Engine이 설치된 우분투 데스크탑에서 바로 동작 (X11 네이티브, VNC 불필요)

1. **사전 준비 (최초 1회, 또는 재부팅할 때마다):** 호스트 X 서버가 컨테이너(root)의 화면 출력을 받아주도록 허용
   ```bash
   xhost +local:root
   ```
2. `echo $DISPLAY`로 현재 디스플레이 번호 확인 (보통 `:0` 또는 `:1`). 대부분 이미 설정되어 있어서 별도 조치 불필요.
3. Docker Engine이 없다면 [공식 문서](https://docs.docker.com/engine/install/ubuntu/)를 따라 설치 (Docker Desktop이 아니어도 됨)

> 우분투용 컨테이너는 도커 기본 **bridge 네트워크**를 씀 (`network_mode: host`가 아님). 컨테이너와 호스트를 같은 네트워크로 묶으면 FastRTPS가 "같은 머신"으로 착각해서 공유메모리(SHM) 전송을 시도하다가, 컨테이너(root)와 호스트(일반 유저)의 권한이 안 맞아 호스트에서 `ros2 topic echo`가 안 되는 문제가 있었음. bridge로 컨테이너마다 별도 IP를 주면 이 문제가 해결됨.

### 5-2. Windows (윈도우)
윈도우는 기본적으로 X11을 지원하지 않기 때문에, X 서버 역할을 해줄 외부 프로그램 설치 필요

1. **설치:** [VcXsrv](https://github.com/marchaesen/vcxsrv)를 깃허브에 접속하여 Release를 클릭한 뒤 최신 exe 파일을 다운받아 설치
2. **실행 (XLaunch):** 시작 메뉴에서 `XLaunch` 실행
3. **설정 단계 (매우 중요):**
   * **Display settings:** `Multiple windows` 선택, Display number에 `0` 입력
   * **Client startup:** `Start no client` 선택
   * **Extra settings:**
     * `Clipboard`, `Primary Selection` 체크
     * `Native opengl` **체크 해제** (3D 프로그램 충돌 방지)
     * 🌟 **`Disable access control` 체크 (필수!)** -> 도커의 화면 신호를 거부하지 않고 받기 위함
4. **마무리:** 다음을 눌러 실행. (작업표시줄 우측 하단 트레이에 `X` 모양 아이콘이 떠 있으면 성공)
5. **호스트 장치를 킬 때마다 계속 작동시켜줘야함**

> Windows용 컨테이너도 우분투와 동일하게 도커 기본 **bridge 네트워크**(`windows_ros_bridge`)를 씀. 원래는 `network_mode: host`였으나, Docker Desktop for Windows는 host 모드를 줘도 실제 Windows 네트워크가 아니라 Docker Desktop 내부 VM 네트워크에 격리되어 호스트(크롬 등)에서 컨테이너 포트로 아예 접속이 안 되는 문제가 있어 bridge로 전환함. 자세한 배경은 [8-2. Windows 네트워킹 참고사항 (웹 개발자용)](#8-2-windows-네트워킹-참고사항-웹-개발자용)에 정리.

### 5-3. macOS (맥)
현재 맥에서 X11 - rviz2 연동이 상당히 불안한 관계로 VNC로 설치

1. 브라우저에서 **http://localhost:6080** 접속 후 **vnc.html** 클릭
2. 화면 한 가운데 Connect 클릭

---

## 6. 실행 명령어 (Docker Compose)

도커 관련 파일은 전부 **`docker-configs/` 아래 OS별 폴더**로 정리되어 있음.
```
docker-configs/
├── ubuntu/   Dockerfile, docker-compose.yml   (bridge 네트워크)
├── windows/  Dockerfile, docker-compose.yml   (bridge 네트워크)
├── mac/      Dockerfile, docker-compose.yml   (VNC, bridge 네트워크)
└── camera-lidar/  docker-compose.yml          (카메라-라이다 데이터 수집 전용, 현재 맥 기준)
```
(예전엔 저장소 루트에 `Dockerfile`, `docker-compose.yml`, `Dockerfile_mac`, `docker-compose-mac.yaml`이 흩어져 있었는데, 지금은 전부 여기로 옮김.)

### 6-1. OS별 실행 명령어

**Ubuntu**
```bash
# 사전 준비 (최초 1회 / 재부팅 후) - 5-1 참고
xhost +local:root

docker compose -f docker-configs/ubuntu/docker-compose.yml up --build -d   # 전체 시작
docker compose -f docker-configs/ubuntu/docker-compose.yml down             # 전체 종료
```

**Windows**
```bash
docker compose -f docker-configs/windows/docker-compose.yml up --build -d   # 전체 시작
docker compose -f docker-configs/windows/docker-compose.yml down             # 전체 종료
```
(화면 띄우기는 5-2의 VcXsrv 사전 준비가 먼저 되어 있어야 함)

**macOS**
```bash
docker compose -f docker-configs/mac/docker-compose.yml up --build -d   # 전체 시작
docker compose -f docker-configs/mac/docker-compose.yml down             # 전체 종료
```
(화면 확인은 5-3의 http://localhost:6080 접속)

**카메라-라이다 데이터 수집 (맥 전용, 현재)**
`webots_data_collection` 패키지가 담당하며, `docker-configs/camera-lidar/docker-compose.yml`을 씀 (같은 `docker-configs/mac/Dockerfile` 이미지 재사용)
```bash
docker compose -f docker-configs/camera-lidar/docker-compose.yml up --build -d
```
수집된 원본 데이터는 `src/webots_data_collection/dataset_output/`에, KITTI 포맷 변환 결과는 `src/webots_data_collection/training/`에 쌓임. 변환은 컨테이너 안이 아니라 아래처럼 직접 실행:
```bash
cd src/webots_data_collection
python3 scripts/webots2kitti.py
```

> 📖 이 compose는 **컨테이너만 띄우고 런치는 직접 실행**하는 구조다. 수집 토픽·출력 포맷·
> 캘리브레이션 상수, 그리고 **학습에 쓰기 전 확인해야 할 것들**(`.bin`이 3채널인 점 등)은
> 별도 문서로 정리해뒀다 → **[DATA_COLLECTION.md](DATA_COLLECTION.md)**

### 6-2. 공통 명령어 (모든 OS 동일)

**로봇 추가**: 서비스 호출 한 번. Webots를 멈추거나 재시작하지 않고, 월드 파일이나
compose를 고치지 않는다. 종류는 `ugv` / `spot` / `drone`.
```bash
# 맵의 빈 공간에 알아서 배치 (이름은 ugv3, ugv4 ... 로 자동 채번)
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'ugv', random: true}"

# 원하는 자리에
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'drone', x: 3.0, y: -2.0, yaw: 1.57}"
```

편대 전체를 바꾸려면 매니페스트를 고른다
([src/webots_robot_spawner/config/fleet/](src/webots_robot_spawner/config/fleet/)):
`default.yaml`(기본 4대) / `random_squad.yaml`(UGV3+Spot1+드론2 무작위) / `ugv_only.yaml`.
compose의 `fleet` 서비스 command에서 `fleet:=` 를 바꾸면 된다.

소환된 로봇의 로그는 `fleet` 컨테이너 안 `/tmp/spawned_robots/{robot_id}.log`에 로봇별로
분리돼 쌓인다. 자세한 내용은 [로봇 소환 문서](#12-로봇-소환-runtime-spawn) 참고.

**목표점을 주면 자율주행 시작** (OS 상관없이 동일한 명령어; 컨테이너 안에서 실행하거나, 호스트 ROS 2 환경변수를 맞췄다면 호스트에서 바로 실행 가능):
```bash
ros2 topic pub -1 /ugv1/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv1/map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

ros2 topic pub -1 /ugv2/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv2/map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

**호스트에서 직접 `ros2 topic list`/`echo`로 확인하고 싶다면**, 호스트 쉘의 ROS 2 환경변수도 컨테이너와 맞춰줘야 함 (`~/.bashrc` 등에 추가):
```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

---

## 7. 로봇 위치 및 맵 데이터

- **정적 맵 파일은 없음.** `map_server`/`amcl`을 쓰지 않고, 각 로봇이 SLAM Toolbox(`mode: mapping`, [config/mapper_params_online_async.yaml](src/webots_python/config/mapper_params_online_async.yaml))로 **자기 맵을 실시간으로 직접 생성**. Nav2도 이 실시간 맵을 그대로 사용.
- **로봇마다 맵이 완전히 독립적.** 프레임 이름 규칙:
  | 항목 | 프레임/토픽 |
  |---|---|
  | 맵 프레임 | `{ns}/map` (예: `ugv1/map`, `ugv2/map`) |
  | 오돔 프레임 | `{ns}/odom` |
  | 로봇 기준 프레임 | `{ns}/base_link` |
  | 맵 토픽 | `/{ns}/map` |
  | 목표점 토픽 | `/{ns}/goal_pose` (`geometry_msgs/msg/PoseStamped`) |

  즉 `ugv1`과 `ugv2`는 서로 다른 좌표계를 쓰며, 한 로봇의 좌표를 다른 로봇에 그대로 써도 안 맞음.
- **목표점 좌표는 각 로봇 자신의 map 프레임 기준.** 다만 이 프로젝트에서 **각 로봇의 `{ns}/map` 원점은 사실상 Webots 월드 원점과 같다.** Webots 드라이버가 `odom → base_link`를 GPS 원값(= 월드 절대좌표)으로 발행하기 때문 ([robot_driver.py:117-119](src/Webots-SummitXL/workspace/simulator/simulator/robot_driver.py#L117-L119)). 즉 좌표 **값**은 로봇끼리 사실상 호환되지만, **프레임 이름은 여전히 로봇별로 달라서** `frame_id`는 정확히 맞춰야 함.
  > ⚠️ "map 프레임 원점 = 로봇 스폰 위치"로 오해하기 쉬운데 **아님.** 실제로 이 오해 때문에 맵 병합에서 좌표가 정확히 두 배로 어긋나는 버그가 났었음 ([맵 병합 문서 2장](MAP_MERGE.md#2-정렬-설계--world-앵커-프레임)).
- **초기 스폰 위치는 이제 월드가 아니라 편대 매니페스트에 있다** (world 절대좌표).
  `my_world.wbt` 기준값은 [`config/fleet/default.yaml`](src/webots_robot_spawner/config/fleet/default.yaml):
  | 로봇 | world x, y |
  |---|---|
  | ugv1 | -6.159, 1.263 |
  | ugv2 | 8.376, 1.373 |
  | spot1 | -0.840, -0.340 |
  | drone1 | -6.500, 5.500 |

  다른 월드를 쓰면 그 월드의 매니페스트(`config/fleet/{월드이름}.yaml`)를 본다.
  무작위 생성 월드는 방·출입구 좌표까지 `config/doorways/`에 함께 나온다
  ([WORLD_GEN.md](WORLD_GEN.md#3-3-출입구-yaml)).
- 웹에서 지도 클릭으로 목표점을 보낼 때는 `/web/goal_point`(`geometry_msgs/msg/PointStamped`)로 발행하되, **`frame_id`가 정확히 `{ns}/map`이어야** [web_goal_relay.py](src/webots_goal_bridge/webots_goal_bridge/web_goal_relay.py)가 해당 로봇의 `goal_pose`로 중계함 (다른 frame_id는 무시됨).
- **전역 병합 맵도 있음.** 마스터 관제 컨테이너가 로봇별 맵을 공통 `world` 프레임 기준으로 합쳐 `/map_merged`(`nav_msgs/msg/OccupancyGrid`)로 발행. 새 로봇이 추가되면 마스터 수정·재시작 없이 자동으로 합류함.

> 💡 병합 설계 근거(왜 직접 짰는지), 로봇 자동 발견 구조, 알고리즘, 검증 결과, 트러블슈팅 전체 기록은 별도 문서로 정리해둠 → **[다중 로봇 맵 병합 구축 기록](MAP_MERGE.md)**

---

## 8. 외부 연동 (웹 목표점 / Gemini)
`webots_goal_bridge` 패키지가 담당.
1. `web_goal_relay.py` — 웹에서 지도 클릭으로 보낸 목표점(`/web/goal_point`)을 로봇의 `goal_pose`로 중계 (동작 중)
2. `gemini_goal_assigner.py` — Gemini와 연동해 지도/위치 기반으로 다음 목표를 자동 할당 (**아직 연동 완료 안됨**, `setup.py`의 entry_point도 주석 처리되어 있음)
   - gemini api는 google ai studio에서 생성 가능 (gemini api 생성 방법은 구글링하면 나와있음)

### 8-1. 맵/목표점 데이터 규격 (웹 개발자용)
웹에서 지도를 그리거나 목표점을 보낼 때 참고할 규격은 [7. 로봇 위치 및 맵 데이터](#7-로봇-위치-및-맵-데이터) 참고. 요약:
- 로봇별로 맵/좌표계가 완전히 분리되어 있으니, UI에서 "어느 로봇의 맵을 보고 있는지"를 항상 구분해서 표시해야 함
- 클릭한 좌표를 보낼 때 `frame_id`를 `{선택된 로봇}/map`으로 정확히 채워야 함

### 8-2. Windows 네트워킹 참고사항 (웹 개발자용)

Windows 환경에서 Docker로 ROS 2를 돌리면서 실측으로 확인한 내용. 웹 중계 서비스(웹소켓/rosbridge 등)를 어디에 배치할지 정할 때 참고.

**결론**
- **컨테이너끼리는 ROS 2 DDS 통신이 정상 동작** (`ugv1` ↔ `ugv2` 목표점 전달 실측 확인됨).
- **컨테이너가 아닌 네이티브 Windows 프로세스에서는 DDS discovery(멀티캐스트)가 안 됨.** Docker Desktop이 WSL2 미러링 네트워킹(`.wslconfig`의 `networkingMode=mirrored`)과 "Host Networking" 옵션을 모두 켜도, 컨테이너는 여전히 Docker Desktop 내부 VM 네트워크(`192.168.65.x`)에 격리되어 있어 실제 Windows에서 보낸 UDP 멀티캐스트 패킷이 컨테이너까지 도달하지 못하는 것을 직접 테스트로 확인함. (DDS 벤더를 FastDDS 대신 OpenDDS 등으로 바꿔도 이건 네트워크 계층 문제라 동일하게 막힘.)
- **HTTP/WebSocket 같은 단순 TCP 서비스는 얘기가 다름.** 컨테이너를 **bridge 네트워크**에 놓고 `docker-compose.yml`의 `ports:`에 포트를 명시적으로 publish하면, Docker Desktop이 자동으로 `localhost:<포트>`를 통해 Windows 호스트(크롬 포함)에서 접속 가능. 이것도 실측으로 확인함.
- 그래서 지금 `docker-configs/windows/docker-compose.yml`을 `network_mode: host`에서 **bridge 네트워크(`windows_ros_bridge`)**로 전환해둠. 우분투가 이미 (다른 이유로) bridge를 쓰고 있던 것과 같은 방향.

**웹 중계 서비스를 추가할 때 권장 방식**
1. 이 프로젝트의 `docker-configs/windows/docker-compose.yml`에 웹 중계 서비스를 **컨테이너로 추가**하고, `ugv1`/`ugv2`처럼 같은 `x-ros-common`(bridge 네트워크)에 태우기. 이러면 ROS 2 topic 구독/발행(DDS)이 별도 설정 없이 됨.
2. 브라우저(크롬)에서 접속해야 하는 포트(HTTP/WebSocket)는 그 서비스에 `ports:`로 명시적으로 publish. (예: mac 설정의 `6080:6080`과 동일한 방식)
3. **네이티브 Windows 프로세스로 직접 ROS 2 노드를 돌리는 방식은 지금 구조로는 지원 안 됨.** 꼭 필요하다면 멀티캐스트 대신 Fast DDS Discovery Server(unicast) 같은 별도 설정이 추가로 필요하니 미리 공유 필요.

## 9. 파이썬 파일을 추가 시 해야할 것
패키지가 목적별로 나뉘어 있으니, 새 노드가 어디에 속하는지 먼저 결정.

| 패키지 | 용도 |
|---|---|
| `webots_python` | 로봇 플랫폼 제어/통제 (텔레옵, 시계 브릿지 등) |
| `webots_goal_bridge` | 외부(웹, Gemini)에서 들어오는 목표점 연동 |
| `webots_data_collection` | 카메라-라이다 데이터 수집 및 변환 |

새 파일을 넣을 패키지를 고른 뒤, 그 패키지의 `setup.py`에서 `entry_points`에 아래처럼 한 줄 추가하면 됨.
```python
    entry_points={
        'console_scripts': [
            # 기존에 있던 노드들이 있다면 유지하고, 아래 줄 추가
            'my_new_node = <패키지_이름>.my_new_node:main',
        ],
    },
```
완전히 새로운 목적(예: 새로운 센서 파이프라인)이라면, 기존 패키지 중 하나에 억지로 끼워넣기보다 `webots_data_collection`과 같은 구조로 새 ament_python 패키지를 하나 만드는 것을 추천.

> 🚗 **UGV(SummitXL Steel)** 는 이 프로젝트의 **기준 로봇**이라, 아래 10·11장이
> "UGV와 무엇이 다른가"로 쓰여 있다. UGV 자체의 구성 — 메카넘 `cmd_vel`,
> 라이다 → 스캔 → SLAM → Nav2 사슬, Nav2 파라미터, `/clock`을 `ugv1`만 발행하는 함정 —
> 은 별도 문서로 정리해뒀다 → **[ugv_setup.md](ugv_setup.md)**

## 10. Spot (사족보행 로봇)

Boston Dynamics Spot을 [seo2730/webots_ros2_spot](https://github.com/seo2730/webots_ros2_spot) (MASKOR/webots_ros2_spot 포크)로 연동. UGV(SummitXL)와 별개 흐름이라 여기 따로 정리.

### 10-1. 사전 준비 (서브모듈 + 월드 설정)
- 서브모듈 2개 추가됨: `src/webots_ros2_spot`(포크, 다리 제어 코드), `src/webots_spot_msgs`(커스텀 메시지). [2. 설치 및 구성](#2-설치-및-구성-installation)의 `git submodule update --init --recursive`에 이미 포함되어 있어서 별도 조치 불필요.
- 🔄 **이 설정은 이제 월드가 아니라 래퍼 PROTO에 들어 있다.**
  [`protos/SpotSensorized.proto`](src/Webots-SummitXL/workspace/simulator/protos/SpotSensorized.proto)가
  아래 구성을 통째로 품고 있고, 월드에는 `IMPORTABLE EXTERNPROTO` 선언만 있으면 된다.
  소환기가 이 PROTO로 몸을 주입한다. 예전처럼 월드에 인라인 40줄을 박아 둘 필요가 없다.
  아래는 **그 PROTO가 무엇을 담고 있는지** 보여주는 참고용이다:
  ```
  EXTERNPROTO "../../../../webots_ros2_spot/protos/Spot.proto"
  ...
  Spot {                      # SpotSensorized 안에서 이렇게 감싼다
    name "spot1"              # 소환기가 매니페스트의 id 를 넣는다
    controller "<extern>"
    supervisor TRUE
    middleExtension [
      DistanceSensor {
        translation -0.05 -0.21 -0.3
        rotation -0.7071 0.7071 0 3.14159
        name "front_left_dist"
        lookupTable [ 0 0 0, 1000 1000 0 ]
      }
      DistanceSensor {
        translation 0.05 -0.21 -0.3
        rotation -0.7071 0.7071 0 3.14159
        name "front_right_dist"
        lookupTable [ 0 0 0, 1000 1000 0 ]
      }
      DistanceSensor {
        translation -0.05 -0.21 0.3
        rotation 0 0 1 -1.57079
        name "rear_left_dist"
        lookupTable [ 0 0 0, 1000 1000 0 ]
      }
      DistanceSensor {
        translation 0.05 -0.21 0.3
        rotation 0 0 1 -1.57079
        name "rear_right_dist"
        lookupTable [ 0 0 0, 1000 1000 0 ]
      }
    ]
  }
  ```
  - **`middleExtension`이 뭔가?** `Spot.proto`가 노출하는 확장 슬롯(`frontExtension`/`middleExtension`/`rearExtension`) 중 하나로, proto 파일을 수정하지 않고 로봇 몸통(등 중앙부)에 장치를 추가 장착하는 통로. 위의 하향 거리센서 4개는 `float_mode`(제자리 호버링)가 바닥까지의 거리를 재는 데 필요한데, 포크 `Spot.proto`엔 이 센서가 없어서 MASKOR 원본 월드와 동일한 방식·배치로 꽂아줌. 센서가 없으면 드라이버가 자동 감지해서 `float_mode`만 비활성화되고 나머지(걷기/SLAM)는 정상 동작함. 여기에 **더** 얹고 싶으면 `SpotSensorized`의 `extraMiddleSlot` 필드를 쓴다 (거리센서 4개는 그대로 유지됨).
  - **디바이스 이름은 절대 바꾸지 말 것** — `spot_driver.py`가 `front_left_dist` / `front_right_dist` / `rear_left_dist` / `rear_right_dist`를 이름으로 찾는다.
  - `EXTERNPROTO`는 **로컬 상대경로**여야 함. GitHub raw URL로 참조하면 `Spot.proto` 내부의 `EXTERNPROTO "SpotLeg.proto"`(상대경로)가 "공식 Webots 에셋 아니면 상대경로 추론 안 해줌" 정책에 걸려서 다리가 하나도 안 뜸.
  - 🚨 **커밋 전 이 줄을 항상 확인할 것.** Webots에서 월드를 저장(`Ctrl+S`)할 때마다 이 줄이 `D:/Document/...` 같은 **절대경로로 자동 변경됨**. 그대로 커밋하면 다른 컴퓨터에서 월드가 안 열림. 원인은 Webots가 "월드의 프로젝트 폴더(`simulator/`) 바깥"에 있는 proto만 절대경로로 정규화하기 때문 (`simulator/protos/` 안에 있는 `VelodyneVLP-16.proto` 등은 상대경로가 유지됨). Webots 옵션으로 끌 수 없으니 `git diff`에서 `D:/`가 보이면 위의 상대경로로 되돌리고 커밋.
  - `supervisor TRUE` 필수 — `spot_driver.py`가 `getFromDef()` 같은 Supervisor 전용 API를 씀. 빠지면 `init()`이 조용히 실패하고 이상한 곳(`touch_fl` 등)에서 크래시남. `SpotSensorized`는 이 값이 기본 TRUE다. `getFromDef()`를 쓰기 때문에 소환기도 Spot만은 `DEF` 이름을 붙여 주입한다(`needs_def`).
  - 🚨 **Webots 씬트리에서 "Spot"을 Add Node로 다시 검색해서 추가하지 말 것.** Webots 기본 내장(스톡) proto가 잡혀서 위 설정이 통째로 날아감. 어차피 지금은 몸을 손으로 넣지 않으니 이럴 일 자체가 없지만, 옛 월드를 손볼 때는 텍스트 에디터로 `.wbt`를 직접 고치고 `Ctrl+Shift+R`로 리로드하는 방식만 쓴다.

### 10-2. 실행
기본 편대([`config/fleet/default.yaml`](src/webots_robot_spawner/config/fleet/default.yaml))에
`spot1`이 들어 있어서 [6. 실행 명령어](#6-실행-명령어-docker-compose)의 `up --build -d`로
다른 로봇들과 같이 소환됨. Spot만 한 대 더 띄우려면:
```bash
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'spot', random: true}"
```
매니페스트에 있는 로봇은 자기 컨테이너를 가지므로 로그도 거기서 본다:
```bash
docker logs -f spot1_brain_windows
```
반면 위처럼 **런타임에 소환한** 로봇(spot2 등)은 `fleet` 컨테이너가 뇌를 띄우므로
`docker exec fleet_spawner_windows tail -f /tmp/spawned_robots/spot2.log`.
런치 파일은 `ros2 launch webots_spot single_spot_launch.py` (namespace는 `ROBOT_ID` 환경변수, 기본값 `spot1`).

> 💡 `spot_driver.py`에 사족보행 관련 함수가 많아서 별도 문서로 정리해둠 → **[spot_driver.py 함수 설명서](spot_driver_functions.md)** (동작 모드 3종, 보행/자세/호버링/상태발행 함수별 역할)

### 10-3. 센서 구성 (라이다 없음 → 뎁스카메라 5개 병합)
Spot에는 UGV의 Velodyne 같은 2D 라이다가 없고, 뎁스카메라 5개(`left_flank_depth`, `right_flank_depth`, `left_head_depth`, `right_head_depth`, `rear_depth`)만 있음. 그래서:
1. `pointcloud_to_laserscan` 노드 5개가 각 뎁스카메라의 3D 포인트클라우드를 `{ns}/base_link` 기준으로 변환 후 **z 높이 필터**(`min_height: -0.35` = 지면 위 ~0.17m부터만 장애물 인정)를 거쳐 개별 `LaserScan`으로 변환
   - 🚨 `depthimage_to_laserscan`을 쓰면 안 됨 — 카메라 수평 가정이라, 아래로 기울어진 Spot 카메라가 바닥을 장애물로 읽어 로봇 주변에 가짜 원형 벽이 생김 (10-6 참고)
2. `webots_spot` 패키지의 커스텀 노드 `multi_scan_merger`([multi_scan_merger.py](src/webots_ros2_spot/webots_spot/multi_scan_merger.py))가 tf2로 5개를 `{ns}/base_link` 기준 하나의 360도 스캔으로 합쳐서 `/spot1/scan`으로 발행
3. SLAM Toolbox/Nav2는 이 `/spot1/scan`을 UGV와 완전히 동일한 방식(`navigation` 패키지의 `nav2.launch.py` 그대로 재사용)으로 사용

### 10-4. cmd_vel 사용법 (UGV와 다름, 주의)
Spot의 `/spot1/cmd_vel`은 **UGV와 같은 진짜 단위(m/s, rad/s)** 다. 내부적으로 Bezier 보행의 걸음 크기(StepLength)로 환산되는데, **전진은 비선형**이다 — 보폭이 작을수록 효율이 높아서 계수 하나로는 저속이 33% 초과속했다. 회귀로 얻은 `v = SPEED_COEF·L^0.81`을 역산해 쓴다. 회전은 선형(`YAW_PER_RADPS = 8.11`).

> 🔄 **2026-08-16 변경.** 예전에는 `linear.x`가 보폭 배율(`×0.15`)이었다. Nav2의 DWB가 그것을 m/s로 알고 궤적을 예측해 **회전이 예측의 절반만 돌면서 경로를 못 따라갔다.** 실측 근거와 환산 계수는 [spot_driver_functions.md](spot_driver_functions.md#cmd_vel-단위), 튜닝 전 과정은 [SPOT_NAV.md](SPOT_NAV.md)에 있다.

보폭 상한(`MAX_STEP_LENGTH 0.090`)과 회전 상한(`MAX_YAW_RATE 2.0`)으로 클램프되어, 현재 케이던스 운용점에서 **0.195 m/s / 0.247 rad/s**가 최대다(시뮬 시각 기준 실측). 그 이상을 줘도 잘린다. 반대로 **약 0.046 m/s 미만은 못 낸다** — 보폭 하한 때문에 그 속도로 나간다.

> 🚨 **Nav2 운용점은 0.15 m/s 다.** 최고속 0.195 는 직진에서 잰 값이라, 그대로 쓰면
> 회전이 얹히는 순간 여유가 0 이 되어 간헐적으로 넘어진다
> ([SPOT_NAV.md 3장 ⑫](SPOT_NAV.md#-주행-중-간헐적으로-넘어진다--임계값이-아니라-확률이었다)).

> ⚠️ 위 **절대 속도값은 재측정 대기 중이다.** 시뮬 속도를 23% 고정으로 가정하고 쟀는데 실제로는 부하에 따라 23~90%로 변한다 ([SPOT_NAV.md 4장](SPOT_NAV.md#-속도를-벽시계로-쟀다)). 단위가 m/s라는 것과 상한·하한이 존재한다는 사실은 유효하다.
```bash
ros2 topic pub /spot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

### 10-5. 자세 제어 서비스
| 서비스 | 타입 | 기능 |
|---|---|---|
| `/spot1/stand_up` | `webots_spot_msgs/srv/SpotMotion` | 일어서기 |
| `/spot1/sit_down` | `webots_spot_msgs/srv/SpotMotion` | 앉기 |
| `/spot1/lie_down` | `webots_spot_msgs/srv/SpotMotion` | 눕기 |
| `/spot1/shake_hand` | `webots_spot_msgs/srv/SpotMotion` | 악수(재롱) |
| `/spot1/set_height` | `webots_spot_msgs/srv/SpotHeight` | 몸높이 조절 |
| `/spot1/float_mode` | `std_srvs/srv/SetBool` | 제자리 호버링 (월드의 `middleExtension` 거리센서 4개 사용) |

예: `ros2 service call /spot1/stand_up webots_spot_msgs/srv/SpotMotion "{override: true}"`

### 10-6. 해결된 이슈 (트러블슈팅 기록)
- ~~**`spot1/odom` TF가 안 올라옴**~~ → **해결.** 원인은 Webots 시뮬레이션이 Play 상태가 아니었던 것. 일시정지 상태면 `step()`이 호출되지 않아 TF/odom이 전혀 발행되지 않음. **시뮬레이션 Play(▶) 상태 확인이 항상 1순위 점검 항목.**
- ~~**`spot1/map`이 안 나옴 (SLAM 맵 생성 실패)**~~ → **해결.** `multi_scan_merger`와 `depthimage_to_laserscan` 노드에 `use_sim_time: True`가 빠져 있어서, 병합 스캔이 벽시계 시간으로 스탬프됨 → 시뮬레이션 시간 기반 TF와 영원히 매칭 안 됨 → slam_toolbox가 "Message Filter dropping message ... queue is full"을 찍으며 스캔을 전부 버림. launch 파일에 `use_sim_time` 추가로 해결 (`/spot1/map` 발행 실측 확인). **새 센서 처리 노드를 추가할 땐 `use_sim_time: True`를 잊지 말 것.**
- ~~**`float_mode` 서비스가 항상 비활성화됨**~~ → **해결.** MASKOR 원본은 거리 센서 4개(`front_left_dist` 등)를 proto가 아니라 자기 월드 파일에서 Spot의 `middleExtension` 슬롯에 꽂아주고 있었음. 같은 배치를 `my_world.wbt`의 Spot 인스턴스에 추가해서 해결 (드라이버가 자동 감지). 지금은 그 배치가 월드가 아니라 [`SpotSensorized.proto`](src/Webots-SummitXL/workspace/simulator/protos/SpotSensorized.proto) 안에 들어 있다.
- ~~**맵이 로봇 주변 반경 1.5m 감옥처럼 나옴**~~ → **해결.** `depthimage_to_laserscan`이 아래로 기울어진 뎁스카메라의 "1~2m 앞 바닥"을 장애물로 읽은 것. `pointcloud_to_laserscan` + z 높이 필터로 교체해서 해결 (10-3 참고).
- ~~**주행할수록 위치가 틀어지고 빈 공간에 유령 장애물이 생김**~~ → **해결.** MASKOR 원본 `spot_driver.py`의 odom 계산에 "로봇이 180도 돌아서 스폰"을 전제한 마이너스 부호가 하드코딩되어 있었음. 우리 월드(정방향 스폰)에서는 odom이 이동 방향과 반대로 나와, SLAM이 매 스캔 잘못된 사전 추정에서 출발 → 맵 오염 + 이동량 비례 드리프트. **판별법**: Spot의 odom은 supervisor 정답 좌표 기반이라 원리상 드리프트 0이어야 하므로, `map→odom` 보정량(tf2_echo)이 수십 cm 이상이면 무조건 좌표 변환 버그.
- ~~**맵이 실제 세계와 180도 뒤집혀 그려짐**~~ → **해결.** 위 버그를 "접속 시점 자세 기준 상대좌표"로 고쳤더니, 드라이버가 재접속하던 순간 로봇이 이전 주행 자리에서 ~185도 돌아서 있어서 그 방향이 맵의 기준축이 되어버림 (IMU 정답 yaw와 odom yaw를 대조해 184.8° 차이로 확정). 최종적으로 **UGV `robot_driver.py`와 동일하게 월드 절대좌표를 odom으로 그대로 발행**하도록 변경 → 시작 자세·재시작 순서와 무관하게 맵이 항상 월드와 정렬됨.
- ~~**Nav2 리커버리(spin/backup)가 전부 Abort됨**~~ → **해결.** behavior_server가 네임스페이스 없는 기본값(`odom`/`base_link`)을 찾다 실패한 것. 처음엔 `local_frame` 키를 넣어 고쳤다고 봤는데 **Humble의 `nav2_behaviors`에 그런 파라미터는 없어서 조용히 무시되고 있었다.** 실제 이름은 `global_frame`이고, RewrittenYaml의 치환값(`{ns}/map`)이 behavior_server에는 맞지 않아 `nav2.launch.py`가 그 노드에서만 `{ns}/odom`으로 덮어쓴다. 자세한 증상·근거는 [ugv_setup.md 5장](ugv_setup.md#5-nav2-파라미터에서-실제로-중요한-값들). **UGV·드론에도 잠재해 있던 버그라 셋 다 함께 고쳐짐.**

### 10-7. UGV와 다른 점 / Spot에서 고려해야 할 사항

| 항목 | UGV (SummitXL) | Spot |
|---|---|---|
| 이동 방식 | 바퀴 (메카넘) | 다리 (Bezier 보행) |
| `cmd_vel` 의미 | 진짜 속도(m/s) | 진짜 속도(m/s) — 약 0.149~0.58 m/s 구간만 (10-4 참고) |
| 넘어짐 | 불가능 | **가능** — 큰 cmd_vel, 충돌, 급회전에 넘어질 수 있음 |
| 주 센서 | Velodyne 라이다 (360도, ~50m) | 뎁스카메라 5개 합성 (10m, 카메라 사이 사각지대 있음) |
| odom 출처 | GPS+IMU 장치값 (월드 절대좌표) | supervisor 정답값 (월드 절대좌표, 동일 컨벤션으로 통일함) |

**운용할 때 주의할 것들:**
1. **넘어지면 복구가 안 됨** — 넘어진 뒤에는 stand_up으로도 못 일어나는 경우가 많고, odom은 정답값이라 넘어진 자세를 그대로 반영해 SLAM/Nav2가 이상해짐. 넘어지면 Webots 월드 리로드(`Ctrl+Shift+R`) + `docker compose restart spot1`이 가장 빠른 복구.
2. **Nav2 파라미터는 Spot 전용이 따로 있음** — `navigation/param/nav2_spot.yaml`. cmd_vel 단위를 m/s로 고치고 속도·가속·footprint를 Spot에 맞췄다. 근거와 실측표는 그 파일 머리말에 있다. ⚠️ 스캔 `range_min`이 0.5 m인데 footprint 반길이가 0.55 m라 **자기 몸 끝에 닿은 장애물은 코스트맵에 안 보인다** — 접촉 사고의 주원인.
3. **footprint는 길이만 늘렸다** — `nav2_spot.yaml`에서 1.1×0.5m(몸통 실측). **폭은 일부러 UGV와 같게 뒀다** — 1.2×0.6으로 키웠더니 내접 반경이 0.30m가 되어 전역 플래너가 가구 사이를 통행 불가로 보고 `failed to create plan`을 냈다 ([SPOT_NAV.md 3장 ④](SPOT_NAV.md#-footprint-폭을-키웠더니-경로가-아예-안-나왔다)).
4. **뎁스카메라 사각지대** — 카메라 5개가 대부분 방향을 커버하지만 카메라 FOV 사이 틈이 있어, 정확히 사각에 있는 얇은 장애물은 스캔에 안 잡힐 수 있음.
5. **새 센서 노드 추가 시 `use_sim_time: True` 필수** — 빠뜨리면 벽시계 스탬프 때문에 SLAM이 데이터를 전부 버림 (10-6의 사례).
6. **Spot 여러 대는 아직 미지원** — 월드의 `DEF Spot` 이름과 드라이버의 `robot_def` 기본값이 1대 기준. 2대 이상은 DEF 이름 분리 + xacro `robot_def` 인자 전달 작업이 필요.
7. **실제 로봇 이식 시** — 지금 odom은 시뮬레이션 정답값이라 드리프트가 0임. 실기에서는 센서 기반 추정 odom(드리프트 있음)으로 바뀌므로 SLAM 파라미터(보정 강도 등)를 다시 튜닝해야 함.

## 11. Drone (중형급 쿼드콥터)

DJI Mavic 2 Pro를 중형급(6.35kg)으로 개조한 `Mavic2ProMedium`. UGV·Spot과 동일하게 **`<extern>` 컨트롤러 + ROS 2 드라이버** 구조로 동작한다.

> 💡 기체 개조 방식, 2단 제어 구조와 게인 근거, 검증 방법, 트러블슈팅 전체 기록은 별도 문서로 정리해둠 → **[드론 구축 기록](drone_setup.md)** (여기 11장은 사용법 요약)

### 11-1. 실행

다른 로봇과 동일하다. 월드에서 `controller "<extern>"`이므로 컨테이너를 띄워야 드론이 움직인다 (안 띄우면 Webots가 컨트롤러를 기다리며 멈춘다).

```bash
# 코드가 바뀌었으면 먼저 빌드 (Dockerfile이 COPY로 코드를 넣기 때문)
docker compose -f docker-configs/windows/docker-compose.yml build
docker compose -f docker-configs/windows/docker-compose.yml up drone1
```

띄워지는 것: `robot_state_publisher` + `webots_ros2_driver`([drone_driver.py](src/Webots-SummitXL/workspace/simulator/simulator/drone_driver.py) 플러그인).

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/drone1/cmd_vel` | 입력 | 속도 명령 (아래 11-2) |
| `/drone1/odom` | 출력 | 위치·자세·속도 (GPS + IMU) |
| `/tf` | 출력 | `drone1/odom → drone1/base_link` |
| `/drone1/camera/image_color` | 출력 | 짐벌 카메라 영상 |
| `/drone1/Velodyne_VLP_16/point_cloud` | 출력 | 라이다 3D 클라우드 |
| `/drone1/scan` | 출력 | 2D 스캔 (클라우드에서 변환) |
| `/drone1/map` | 출력 | SLAM 맵 (맵 병합 참여) |

| `/drone1/navigate_to_pose` | 액션 | Nav2 목표점 (UGV와 동일, 고도 고정) |
| `/drone1/goal_pose_3d` | 입력 | **층을 골라서** 가는 목표 (아래 11-6) |
| `/drone1/map_active` | 출력 | 현재 순항 고도 한 층 (Nav2가 이걸 본다) |
| `/drone1/map_layer_{0,1,2}` | 출력 | 후보 층별 지도 |
| `/drone1/altitude_status` | 출력 | 층 선택 근거 로그 |

> **Nav2는 UGV 파라미터를 그대로 공유한다.** 지상 로봇 전제라 못 쓸 줄 알았는데 실측해
> 보니 그대로 동작했다 — Nav2는 `linear.x`/`angular.z`만 쓰고, 드라이버는 `linear.z`가 0이면
> 목표 고도를 유지하기 때문이다. 그래서 2D 내비게이션이 정속 수평 비행으로 번역된다.
> 4 m 목표 3회 SUCCEEDED, 최종 오차 0.13 / 0.20 / 0.15 m, 고도는 2.00 m 고정 유지.
>
> ⚠️ **2D 내비게이션이지 3D 경로계획이 아니다.** 고도는 계획 대상이 아니라 드라이버가
> 붙잡고 있는 상수다. 장애물 위로 넘어가거나 아래로 지나가는 경로는 만들지 못한다.

### 11-2. cmd_vel 사용법 (UGV와 다름, 주의)

드론은 모터 4개로 6자유도를 다루는 **underactuated** 시스템이라, UGV처럼 Twist를 바퀴 속도로 바로 변환할 수 없다. 드라이버가 2단으로 처리한다.

```
cmd_vel ──▶ [속도 외부 루프] ──▶ 자세 목표 ──▶ [자세/고도 내부 루프] ──▶ 모터 4개
```

| 필드 | 의미 | 비고 |
|---|---|---|
| `linear.x` | 전후 속도 (m/s) | 기체 기준 |
| `linear.y` | 좌우 속도 (m/s) | 기체 기준. **UGV(메카넘)와 달리 기체가 기울어서 이동** |
| `linear.z` | **상승 속도** (m/s) | 목표 고도를 적분해서 바꾼다 (위치 아님) |
| `angular.z` | 선회 각속도 (rad/s) | |

```bash
# 전진 1 m/s
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
# 상승 (0.5 m/s로 목표 고도 상승)
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}"
# 정지 -> 제자리 유지 (속도 0을 능동 추종)
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{}"
```

**키보드 조종**: UGV의 `simulator keyboard`에 대응하는 드론용 텔레옵이 있다. 고도(`R`/`F`) 축이 추가됐고, 좌우가 조향이 아니라 평행이동인 점이 UGV와 다르다.

```bash
docker exec -it drone1_brain_windows bash -c \
  "source /ros2_ws/install/setup.bash && \
   ros2 run simulator drone_teleop --ros-args -r __ns:=/drone1"
```

```
        W : 전진              Q / E : 좌 / 우 선회
   A         D : 좌 / 우 평행이동   R / F : 상승 / 하강
        S : 후진              Space : 정지(호버)   = / - : 속도 증감
```

**Nav2 호환**: Nav2는 `linear.x` / `angular.z`만 쓰므로 고도 고정 상태로 그대로 붙일 수 있다. `linear.y`·`linear.z`는 안 쓰면 0이 들어올 뿐이라 무해하다.

실측 추종 성능 (Webots 헤드리스, 30초):

| 명령 | 실제 | 오차 |
|---|---|---|
| 정지 (0 m/s) | 0.009 m/s 드리프트 | — |
| 전진 1.0 m/s | 1.01 m/s | 1% |
| 선회 0.5 rad/s | 0.50 rad/s | ~1% |
| 목표 고도 2.0 m | 1.98 m, 오버슈트 없음 | 1% |

### 11-3. 기체 구성 (Mavic 2 Pro 개조)

순정 `Mavic2Pro.proto` 기준으로 **크기 ×2, 질량·추력/토크 상수 ×7** (0.907kg → 6.35kg). 질량과 추력을 같은 비율로 키웠기 때문에 추력 대 중량비가 보존되고, 순정 제어 법칙이 그대로 유효하다.

- PROTO: `workspace/simulator/protos/Mavic2ProMedium.proto` (기체)
- 래퍼 PROTO: `workspace/simulator/protos/Mavic2ProMediumSensorized.proto` (기체 + 라이다) ← **소환은 이쪽**
- 메시·텍스처: `workspace/simulator/protos/Mavic2Pro/` (아래 11-4 참고)
- 월드 배치: 이제 월드에 정적으로 두지 않는다. 편대 매니페스트가 소환한다 → [SPAWNER.md](SPAWNER.md)

센서 구성:

| 디바이스 | Webots 이름 | 어디에 | 용도 |
|---|---|---|---|
| GPS | `gps` | 기체 PROTO | 위치·속도 → odom |
| InertialUnit | `inertial unit` | 기체 PROTO | roll/pitch/yaw |
| Gyro | `gyro` | 기체 PROTO | 각속도 (자세 D항) |
| Compass | `compass` | 기체 PROTO | 선언만 — 드라이버는 IMU yaw를 쓴다 |
| 짐벌 카메라 400×240 | `camera` | 기체 PROTO `cameraSlot` | 영상 |
| **Velodyne VLP-16** | `Velodyne_VLP_16` | **래퍼 PROTO `bodySlot`** | **SLAM** |

라이다는 동체 위 0.12 m에 얹었다. 그 높이는 아래쪽 광선(±15°)이 동체·랜딩기어·프로펠러를 스치지 않도록 계산해서 고른 값이고, 헤드리스로 재서 확인했다(자기 몸 반사 0개, 수평 광선이 10 m 벽을 10.002 m로 읽음). 근거는 래퍼 PROTO 주석에 있다.

> **라이다에 질량을 주지 않았다** (`lidarPhysics FALSE`). VLP-16 실물은 0.83 kg이라 켜면 본체(2.8 kg) 대비 +30%고, `K_VERTICAL_THRUST`(68.5)부터 다시 잡아야 한다. 탑재 중량 영향까지 보려면 TRUE로 켜되 호버 추력을 재측정할 것.

> **1 m 이내는 안 보인다.** VelodynePuck의 `minRange`가 1 m다. 좁은 복도나 벽 가까이에서 벽이 사라지므로, 자율 비행을 붙일 때 안전 여유를 그 이상으로 잡아야 한다.

**왜 뎁스카메라가 아니라 라이다인가** — UGV와 파이프라인이 100% 같아진다(`Velodyne → pointcloud_to_laserscan → slam_toolbox`). 뎁스는 Spot처럼 시야를 채우려면 5개 + `multi_scan_merger`가 필요해 로봇당 노드가 6개 늘고, 하나당 FOV가 90° 남짓이라 제자리 요잉이 잦은 드론에 불리하다.

### 11-4. 해결된 이슈 (트러블슈팅 기록)

1. **드론이 투명하게 보임** — Webots R2025a는 메시·텍스처 에셋을 설치본에 포함하지 않는다(`projects/robots/dji/mavic/` 폴더 자체가 없음). `webots://` 경로 참조가 조용히 실패해 셰이프가 하나도 안 그려졌다. 물리는 정상 동작해서 "안 보이는데 날아다니는" 상태가 됐다. → 메시 14개 + 텍스처를 `protos/Mavic2Pro/`에 로컬 포함하고 상대 경로로 참조.

2. **고도가 0.2~4.2m로 진동** — 순정 Mavic 데모 월드는 `WorldInfo.defaultDamping`(linear/angular 0.5)에 의존해 안정화되는데 `my_world.wbt`에는 그 설정이 없었다. 격리 테스트로 원인을 분리한 결과 **`basicTimeStep`은 무관**(8ms로 낮춰도 동일하게 진동), **댐핑이 원인**. → 전역으로 켜면 UGV/Spot 물리까지 바뀌므로 **드론 PROTO의 `Physics.damping`에만** 동일 값을 걸었다. 시뮬레이션 속도 손해 없음.

3. **고도 39% 오버슈트** — 순정 제어 법칙은 고도를 **P항만으로** 제어한다. 이중적분기에 P 제어만 걸면 준안정이라, 위 2번의 댐핑이 사실상 D항을 대신하고 있었다. → 컨트롤러에 `k_vertical_d`(수직 속도 D항, `wb_gps_get_speed_vector()` 사용)를 추가. 2.79m 오버슈트가 사라지고 2.000m에 단조 수렴한다.

4. **Webots GUI에서 월드를 저장하면 드론 컨트롤러가 `"<none>"`으로 바뀜** — Spot.proto 절대경로 변형과 같은 부류의 현상. 저장 후 `controller "mavic2pro_medium"`인지 확인할 것.

### 11-5. 고도 회피 (2.5D 레이어드)

> 📘 구조 전체와 **직접 테스트하는 명령어**는 [DRONE_NAV.md](DRONE_NAV.md)에 있다.

Nav2는 2D라 고도를 계획하지 않는다. 그 **한 축만 바깥에서** 담당해 "장애물을 넘어간다"를 얻는 구조다. Nav2 자체는 손대지 않는다.

```
Velodyne(수평) ─┐
                ├─▶ drone_layer_mapper ─┬─▶ /map          층 합집합 ─▶ 맵 병합기
down_depth(하향)┘                        ├─▶ /map_active  현재 층   ─▶ Nav2
                                         └─▶ /map_layer_k          ─▶ altitude_selector
                                                                          │
                      /goal_pose_3d ─▶ 층 선택 ─▶ 고도 이동 ─▶ /goal_pose ─▶ Nav2
```

```bash
# 층을 골라서 가는 목표 (고도 회피 O)
ros2 topic pub -1 /drone1/goal_pose_3d geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'drone1/map'}, pose: {position: {x: -2.9, y: 4.3}, orientation: {w: 1.0}}}"
# 기존처럼 고도 고정으로 가려면 /drone1/goal_pose 를 그대로 쓰면 된다
```

**왜 3D 플래너가 아닌가** — 계획 비용이 군집에서 대수만큼 곱해진다. 이 구조는 계획을 지금과 똑같은 2D A*로 유지하고, 늘어나는 비용은 층당 회랑 검사(수백 칸)뿐이다. 오히려 드론 1대당 노드가 **2개→1개로 줄었다**(`pointcloud_to_laserscan` + `slam_toolbox` → `drone_layer_mapper`).

실측 (my_world, 층 1/2/3 m):

| 상황 | 층 판정 | 결과 |
|---|---|---|
| 현재 층 막힘, 위가 열림 | `1m:X(장애물22.3%) 2m:X(3.2%) 3m:OK(0.0%)` | 3.0 m로 **상승**, 고도 2.00→2.97 (오차 0.03 m) |
| 현재 층 막힘, 아래가 열림 | `1m:OK(0.5%) 2m:X(5.9%) 3m:X` | 1.0 m로 **하강**, 목표까지 3.00→0.60 m |
| 현재 층 열림 | `2m:OK(0.0%)` | 고도 유지, 3.1 m 주행 |

### 11-6. 알려진 한계 / 다음 작업

- **여전히 3D 경로계획은 아니다** — 11-6의 고도 회피는 **이산적인 층 선택**이지 연속 3D 경로가 아니다. 상승과 수평이동이 섞이지 않고 순차로 일어나고, 층 사이 높이의 장애물은 표현하지 못한다. 아래는 그 배경. Nav2의 플래너는 `nav_msgs/OccupancyGrid`(2D 격자) 위에서 돌고, 고도는 계획 변수가 아니라 드라이버가 잡고 있는 상수다(실측 3회 모두 1.99~2.00 m로 고정). 장애물 위를 넘거나 아래로 지나는 경로는 원리적으로 나올 수 없다. 3D로 가려면 ① 3D 점유 지도(`octomap_server` — 의존성은 Dockerfile에 이미 들어 있다) ② 3D 플래너(Nav2에는 없다. MoveIt+OMPL이나 UAV 전용 플래너) ③ `linear.z`를 쓰는 컨트롤러(드라이버는 이미 지원)가 필요하다.
- **드론의 맵은 비행 고도의 수평 단면** — `pointcloud_to_laserscan`의 높이 필터가 기체에 붙은 `base_link` 기준이라 2 m로 날면 2 m 단면이 찍힌다. 덕분에 바닥 반사는 자동으로 걸러지지만, 고도를 크게 바꾸면 다른 단면이 같은 맵에 겹친다. **매핑 중에는 고도를 유지하는 편이 좋다.** 지상 로봇의 맵과 병합할 때도 이 점이 차이를 만든다.
- **1 m 이내 사각** — 위 11-3 참고. 장애물 회피를 붙이려면 하향/근접 센서를 추가해야 한다 (`bodySlot`의 `extraBodySlot` 필드가 그 자리다).
- **속도 제어이지 위치 제어가 아님** — `cmd_vel`이 0이면 속도 0을 능동 유지하지만(드리프트 0.009 m/s), 바람 같은 외란에 밀린 뒤 원위치로 돌아가지는 않는다. 웨이포인트 비행에는 위치 루프가 추가로 필요하며, Nav2를 붙이면 Nav2가 그 역할을 한다.
- **짐벌이 각속도 댐핑만 함** — 순항 중 기체가 5~15° 기울면 카메라도 같이 기운다. 정찰 용도로는 자세 자체를 상쇄하도록(`-roll`/`-pitch`) 바꾸는 편이 낫다.

> 참고: 초기에는 Webots 내장 C 컨트롤러(`controllers/mavic2pro_medium/`)로 키보드 조종을 했으나, OS별 컴파일이 필요하고 ROS 2 미션 스택에 붙일 수 없어 폐기했다. 11-4의 물리 이슈들은 그 컨트롤러로 규명한 것이며 결론은 그대로 유효하다. 필요하면 `git log -- workspace/simulator/controllers/`에서 복원할 수 있다.

## 12. 로봇 소환 (Runtime Spawn)

실행 중인 Webots에 로봇을 추가하는 기능. 담당 패키지:
[src/webots_robot_spawner/](src/webots_robot_spawner/) + [src/webots_spawner_msgs/](src/webots_spawner_msgs/)

**`my_world.wbt`에는 로봇이 없다.** 환경(아레나·벽 6개·가구 40개)과 소환 전담 노드
`spawn_supervisor` 하나만 있고, 로봇은 전부 소환으로 들어온다. 예전에는 로봇 4대가
월드 파일에 박혀 있어서 한 대 늘릴 때마다 월드 편집 + compose 서비스 추가 + Webots
재시작이 필요했다.

### 12-1. 소환하기

```bash
# 맵(/map_merged)의 빈 공간에 알아서 배치. 이름은 자동 채번(ugv3, ugv4 ...)
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'ugv', random: true}"

# 원하는 자리에
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'drone', x: 3.0, y: -2.0, yaw: 1.57}"

# 이름을 직접 정하고, 빈 공간 검사에 실패해도 강행
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'spot', robot_id: 'scout1', x: 0.0, y: 5.0, force: true}"
```

| 필드 | 뜻 |
|---|---|
| `type` | `ugv` / `spot` / `drone` |
| `robot_id` | 비우면 씬 트리를 보고 자동 채번 |
| `random` | true면 x/y/yaw 무시하고 맵의 빈 자리를 고름 |
| `min_clearance` | 주변에 요구할 여유 반경(m). 0이면 종류별 기본값 |
| `force` | 빈 공간 검사 실패에도 그 자리에 놓음 |

응답에 실제 부여된 이름과 좌표, 실패 시 사유가 담긴다.

### 12-2. 편대 매니페스트

편대는 yaml로 정의한다 ([config/fleet/](src/webots_robot_spawner/config/fleet/)):

| 파일 | 내용 |
|---|---|
| `default.yaml` | ugv1 / ugv2 / spot1 / drone1 — 예전 월드와 같은 좌표 |
| `random_squad.yaml` | UGV 3 + Spot 1 + 드론 2, 무작위 배치 |
| `ugv_only.yaml` | UGV 2대만 (맵 작업용 경량 편대) |

```yaml
fleet:
  - {type: ugv,   id: ugv1, x: -6.159, y: 1.263, yaw: -2.910}
  - {type: ugv,   count: 3, random: true}       # 3대를 알아서
  - {type: drone, count: 2, random: true, clearance: 1.0}
spawn_area: [-9.0, -6.0, 9.0, 7.0]              # random 배치 영역
```

`fleet` 컨테이너가 기동하면서 매니페스트대로 소환한다. 바꾸려면 compose의 `fleet`
서비스 command에서 `fleet:=` 값을 바꾼다 (`fleet:=''` 면 자동 소환 없이 서비스만 받음).

> `spawn_area`는 **항상** 지켜진다. 맵이 있으면 그 영역 안에서 장애물까지 피하고,
> 월드가 비어 있는 냉시동(SLAM 맵이 존재할 수 없음)에서는 로봇 간 간격만 보고 고른다.

### 12-2-1. 새 월드를 만들어 편대를 올리기까지

월드를 얻는 방법이 세 가지다. 어느 쪽이든 **`spawn_supervisor` 노드와 래퍼 PROTO의
`IMPORTABLE` 선언**이 들어가야 소환이 되는데, 세 스크립트가 같은 `prepare()` 로직을
공유하므로 신경 쓸 필요는 없다.

| 스크립트 | 용도 |
|---|---|
| [gen_world_random.py](src/webots_robot_spawner/scripts/gen_world_random.py) | **시드마다 다른 건물** — 복도 + 방 + 장애물 산포 |
| [gen_world.py](src/webots_robot_spawner/scripts/gen_world.py) | 넓은 작전 지역을 처음부터 생성 (창고형, 결정적) |
| [gen_world_from_map.py](src/webots_robot_spawner/scripts/gen_world_from_map.py) | SLAM 맵·건물 도면(점유격자) → 월드 |
| [prepare_world.py](src/webots_robot_spawner/scripts/prepare_world.py) | 밖에서 가져온 `.wbt`를 소환 가능 상태로 |

> 📖 옵션·알고리즘·OS별 명령·트러블슈팅은 **[WORLD_GEN.md](WORLD_GEN.md)** 에 따로 정리했다.
> 아래는 편대를 올리기까지의 최단 경로만 적는다.

**① 월드 생성** — 편대 매니페스트가 **같이** 나온다. 생성기는 어디가 비었는지 알기
때문에 좌표까지 써준다 (손으로 고르면 선반 안에 로봇을 놓기 쉽다).

```powershell
docker run --rm -v "${PWD}:/w" -w /w windows-master python3 `
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3
```
→ `worlds/arena_s3.wbt` + `config/fleet/arena_s3.yaml` + `config/doorways/arena_s3.yaml`

무작위 생성이 만드는 것은 **건물 하나가 놓인 부지**다. 실내에만 갇히지 않게
건물을 안쪽으로 물리고 둘레를 바깥 땅으로 남긴다:

```
┌─────────────────────────────┐  ← 부지 울타리 (2 m)
│   마당                       │
│   ┌───────────────────┐     │
│   │ 복도 + 방          │     │
│   ╡ ← 외부 출입구      │     │
│   └───────────────────┘     │
└─────────────────────────────┘
```

- **방마다** 로봇이 지나갈 출입구가 있고, **바깥에서도** 건물로 들어올 수 있다.
  외부 출입구는 복도 끝에만 내서 진입 즉시 동선에 붙는다
- 문짝은 달지 않는다. 벽이 끊긴 구간이 곧 출입구이고, 그 틈의 중앙 좌표를
  `config/doorways/`에 남긴다 (복도 중심선도 함께 — 순찰 경로 짤 때 쓴다)
- 개수는 지정할 수 있다: `--corridors` `--links` `--rooms` `--entrances`.
  크기에 안 들어가면 줄이고 알려준다
- 마당 폭은 `--yard`(기본 12 m), `--yard 0`이면 건물이 부지를 꽉 채운다

자세한 것은 [WORLD_GEN.md 3-2-1](WORLD_GEN.md#3-2-1-부지와-외부-출입구)과
[3-3](WORLD_GEN.md#3-3-출입구-yaml) 참고.

**② compose를 그 편대에 맞추기** — 로봇 서비스와 `fleet:=` 값이 한꺼번에 맞춰진다.

```powershell
docker run --rm -v "${PWD}:/w" -w /w windows-master python3 `
  src/webots_robot_spawner/scripts/gen_fleet_compose.py --fleet arena_s3.yaml
```

> 🚨 ②를 건너뛰면 **조용히 어긋난다.** 소환기는 새 편대를, 로봇 컨테이너는 옛 이름을
> 쓰게 된다. 그래서 생성기가 두 곳을 한 번에 고친다.

**③ Webots에서 월드 열기** — `File > Open World...` → `worlds/arena_s3.wbt`

**④ 컨테이너 기동**
```bash
docker compose -f docker-configs/windows/docker-compose.yml up -d
```

편대가 뜨기까지 30초쯤 걸린다. `fleet_start_delay`가 20초라 느린 게 아니라
[기동 순서](SPAWNER.md#7-기동-순서-왜-드론만-다른가)를 지키는 중이다.

**재빌드가 필요한 때 / 아닌 때**

| 바꾼 것 | 필요한 조치 |
|---|---|
| 월드 `.wbt` | 없음 — Webots가 호스트에서 직접 읽는다 |
| 편대 매니페스트 | 컨테이너 재시작 (마운트되어 있다) |
| compose 서비스 구성 | `up -d` |
| 파이썬 소스 (드라이버·소환기) | `build` 후 `up -d` |

### 12-3. 구조 요약 (몸 / 뇌 / 컨테이너)

세 층으로 나뉜다. **몸과 뇌는 1:1이어야 하지만, 뇌와 컨테이너는 그럴 이유가 없다** —
이 구분이 구조의 핵심이다.

```
[Webots — 호스트]                  [fleet 컨테이너]        [로봇별 컨테이너]

spawn_supervisor (유령 로봇) ←TCP→  spawn_supervisor
  supervisor TRUE                    ├─ /spawn_robot 서비스
  synchronization FALSE              ├─ 매니페스트대로 몸 주입
                                     └─ /map_merged 로 빈 자리 찾기
ugv1   (몸)  ←────────────────────────────────────TCP────→ ugv1_brain_*
spot1  (몸)  ←────────────────────────────────────TCP────→ spot1_brain_*    driver
drone1 (몸)  ←────────────────────────────────────TCP────→ drone1_brain_*   + SLAM
                                                                            + Nav2
ugv3 (런타임 소환된 몸) ←TCP→ 이 뇌만 fleet 컨테이너가 띄운다               + registrar
```

- **몸**은 fleet 컨테이너의 소환기가 매니페스트대로 Webots에 주입한다
- **뇌(제어·경로)** 는 로봇별 컨테이너가 담당한다. 컨테이너 경계가 있어야
  `cpuset`으로 코어를 고정하거나 `cpus`로 상한을 걸 수 있다
- **매니페스트에 없는 런타임 소환**만 fleet 컨테이너가 뇌까지 띄운다.
  그 로그는 `/tmp/spawned_robots/{robot_id}.log` (fleet 컨테이너 안)

**compose는 매니페스트에서 생성한다** (위 12-2-1의 ②). 손으로 유지하면 이중 관리가 되고,
어긋나도 아무 에러 없이 조용히 틀린다. `# >>> FLEET GENERATED` 마커 사이만 갈아 끼우므로
`master`/`fleet` 서비스와 주석은 그대로 남고, `--check`를 붙이면 고치지 않고 최신인지만
확인한다(CI용). 셸별 문법과 이미지 이름(우분투 `ubuntu-master`, 맥 `mac-master`)은
[WORLD_GEN.md 2장](WORLD_GEN.md#2-os별-실행-방법-중요)에 표로 있다.

> 📖 **왜 이렇게 나눴는지, 로봇 종류 정의표, 소환 한 번에 일어나는 일, 빈 자리를 고르는
> 규칙, 기동 순서 교착, 잔여 몸 정책, 파라미터 전체는 별도 문서로 정리해뒀다 →
> [로봇 소환 구축 기록](SPAWNER.md)**

### 12-4. 자주 걸리는 것

깊은 배경은 [SPAWNER.md 10장](SPAWNER.md#10-트러블슈팅)에 있고, 여기는 목록만 둔다.

| 증상 | 원인 / 조치 |
|---|---|
| `In order to import the PROTO 'X' ...` | 월드에 `IMPORTABLE EXTERNPROTO` 선언이 없다. `prepare_world.py --check` |
| 월드는 열리는데 로봇이 안 나온다 | 매니페스트 이름과 compose의 `fleet:=` 불일치. `gen_fleet_compose.py --check` |
| 편대가 뜨는 데 30초 걸린다 | 정상이다. `fleet_start_delay` 20초 + 기동 순서를 지키는 중 ([SPAWNER.md 7장](SPAWNER.md#7-기동-순서-왜-드론만-다른가)) |
| 월드를 재로드했더니 로봇이 죽었다 | `driver` 프로세스가 종료되고 `ros2 launch`가 되살리지 않는다. `docker compose restart` |
| `compose down` 했더니 몸이 월드에 남았다 | `stale_body_policy`(기본 `recreate`)가 다음 기동 때 정리한다 ([SPAWNER.md 8장](SPAWNER.md#8-잔여-몸-정책-stale_body_policy)) |
| 소환한 드론이 이륙을 못 한다 | 동기화가 TRUE로 복원되지 않았다. 드론만 매 물리 스텝 제어 루프가 돌아야 한다 |
| `ros2 topic hz`가 "not published yet" | 노드 100개 넘으면 CLI가 거짓말을 한다. rclpy로 직접 구독해 확인 |

**despawn은 없다.** 스폰 실패 시 롤백만 한다 — 뇌가 유예 시간 안에 죽으면 몸을 씬
트리에서 되돌려 조종 불가능한 유령 로봇이 쌓이지 않게 한다.

## 향후 계획
- Gemini api 연동
- ~~Drone 추가~~ → 완료 (기체 개조·2단 비행 제어·ROS 2 연동 → [drone_setup.md](drone_setup.md))
- ~~드론 자율비행~~ → 완료 (라이다 탑재 + 층별 지도 + 전역 층 선택 + 지역 고도 회피.
  목표점 5개 중 4개 도달, 리밋 사이클 0회. [DRONE_NAV.md](DRONE_NAV.md) 참고)
- 드론 3D 경로계획(연속) — 미착수. 지금은 **2.5D 이산 층 선택**이다
  ([DRONE_NAV.md 8장](DRONE_NAV.md#8-알려진-한계))
- ~~Spot 자율주행 튜닝~~ → 완료. **도달률 0~1/3 → 3/3, 목표 오차 0.36~0.38 m,
  `map`→`odom` 표류 0, 넘어짐 없음.** cmd_vel을 m/s로 환산, Spot 전용 Nav2
  파라미터, 보행 케이던스 노출, SLAM TF 제거, 카메라별 스캔 하한, 운용 속도
  0.15 m/s. 남은 것은 배회 1.9~2.2배 ([SPOT_NAV.md](SPOT_NAV.md))
- ~~Spot 절대 속도 재측정~~ → 완료 (시뮬 시각 기준 최고 안정속 **0.195 m/s**,
  벽시계로 잰 옛 값은 최대 4배 틀렸다. 제자리 회전은 상한을 0.5→2.0으로 올려 0.079→**0.247 rad/s**)
- Spot `map`→`odom` 표류 — 해결. slam_toolbox 가 **정답 odom 에 보정을 얹어**
  목표 오차를 키우고 있었다. SLAM 은 지도만 만들고 TF 는 항등 정적 변환을 쓴다
  ([SPOT_NAV.md 3장 ⑩](SPOT_NAV.md#-mapodom-표류가-목표-오차로-그대로-나왔다))
- Spot 리커버리 `time_allowance` — **남음.** BT XML 속성이라 파라미터로 못 바꾼다
  ([SPOT_NAV.md 5장](SPOT_NAV.md#5-미해결--다음-작업))
- ~~로봇 생성 자동화~~ → 완료 (서비스 호출로 UGV/Spot/드론 런타임 소환, 편대는 yaml. [12](#12-로봇-소환-runtime-spawn) 참고)
- ~~여러 월드 지원 / 점유격자에서 월드 자동 생성~~ → 완료 (창고형·무작위 건물·점유격자
  변환·외부 월드 반입 4종. 무작위 생성은 부지에 건물을 앉히고, 방마다 출입구를,
  바깥에서 들어올 외부 출입구를 보장하며 좌표를 yaml로 남긴다.
  [WORLD_GEN.md](WORLD_GEN.md) 참고)
- 자율 탐사 (`explore_lite` 연동) — 미착수
- 실제 SLAM 지도 → 월드 왕복 시험 (`gen_world_from_map.py`는 합성 지도로만 검증) — 미착수
- ~~다중 로봇 지도 병합~~ → 완료 (마스터 관제 컨테이너에서 `/map_merged` 발행, 로봇 자동 합류/이탈까지 확인. [맵 병합 구축 기록](MAP_MERGE.md) 참고)
- ~~윈도우 환경도 bridge 네트워크로 전환 테스트~~ → 완료 ([8-2](#8-2-windows-네트워킹-참고사항-웹-개발자용) 참고)
- ~~Spot 추가~~ → 완료 (다리 제어 + 뎁스카메라 SLAM 맵 생성까지 확인, [10-6](#10-6-해결된-이슈-트러블슈팅-기록) 참고)

## 참고 문서 (References)

이 저장소의 문서 (위 [문서 지도](#-문서-지도)와 같은 목록, 성격별로 묶은 것):

**규격 — 무엇을 주고받나**

| 문서 | 다루는 것 |
|---|---|
| [INTERFACES.md](INTERFACES.md) | 토픽·서비스·프레임·QoS·환경변수 총람. 세 로봇의 `cmd_vel` 의미 차이 표 포함 |

**만들기 — 환경과 편대**

| 문서 | 다루는 것 |
|---|---|
| [WORLD_GEN.md](WORLD_GEN.md) | 월드(환경) 만들기 — 무작위 방/복도, 창고형, 점유격자 변환, 외부 반입. **OS별 명령어 정리 포함** |
| [SPAWNER.md](SPAWNER.md) | 로봇 소환 구축 기록 — 몸/뇌/컨테이너 분리, 기동 순서 교착, 잔여 몸 정책, 새 로봇 종류 추가 |

**로봇별**

| 문서 | 다루는 것 |
|---|---|
| [ugv_setup.md](ugv_setup.md) | UGV(SummitXL) — 메카넘 역기구학, 라이다 → 스캔 → SLAM → Nav2 사슬, `/clock` 함정 |
| [spot_driver_functions.md](spot_driver_functions.md) | Spot 드라이버 함수 설명서 (보행·자세·호버링·상태발행) + 자율주행 직접 테스트하는 법 |
| [SPOT_NAV.md](SPOT_NAV.md) | Spot 자율주행 튜닝 기록 — cmd_vel 단위 환산, footprint·리커버리 함정, 측정 방법론 |
| [drone_setup.md](drone_setup.md) | 드론 기체 구성과 2단 비행 제어, 게인 근거 |
| [DRONE_NAV.md](DRONE_NAV.md) | 드론 자율비행(층별 지도·전역 층 선택·지역 고도 회피)과 테스트 명령어 |
| [SPOT_NAV.md](SPOT_NAV.md) | Spot 자율주행 튜닝 기록 — cmd_vel 단위·보행 케이던스·Nav2 파라미터, 겪은 이슈와 해결 |

**관제·데이터**

| 문서 | 다루는 것 |
|---|---|
| [MAP_MERGE.md](MAP_MERGE.md) | 여러 로봇의 SLAM 지도를 `/map_merged`로 합치는 부분 |
| [DATA_COLLECTION.md](DATA_COLLECTION.md) | 카메라-라이다 수집 → KITTI 변환, 학습에 쓰기 전 확인할 것 |

서브모듈 쪽 문서: [src/Webots-SummitXL/README.md](src/Webots-SummitXL/README.md) (원본 프로젝트 안내),
[src/webots_ros2_spot/README.md](src/webots_ros2_spot/README.md) (MASKOR 포크)

바깥 자료:
- Webots 공식 사용자 가이드 (User Guide)
- Webots 공식 레퍼런스 매뉴얼 (Reference Manual)
