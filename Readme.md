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
  - [11-5. 알려진 한계 / 다음 작업](#11-5-알려진-한계--다음-작업)
- [12. 로봇 소환 (Runtime Spawn)](#12-로봇-소환-runtime-spawn)
  - [12-1. 소환하기](#12-1-소환하기)
  - [12-2. 편대 매니페스트](#12-2-편대-매니페스트)
  - [12-3. 구조 (몸 / 뇌 / 컨테이너)](#12-3-구조-몸--뇌--컨테이너)
  - [12-4. 주의사항 / 트러블슈팅](#12-4-주의사항--트러블슈팅)
- [향후 계획](#향후-계획)
- [참고 문서 (References)](#참고-문서-references)

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
- **Webots 월드(`my_world.wbt`) 상의 초기 스폰 위치** (참고용, world 절대좌표):
  | 로봇 | world x, y |
  |---|---|
  | ugv1 | -6.16, 1.26 |
  | ugv2 | 8.38, 1.37 |
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

## 10. Spot (사족보행 로봇)

Boston Dynamics Spot을 [seo2730/webots_ros2_spot](https://github.com/seo2730/webots_ros2_spot) (MASKOR/webots_ros2_spot 포크)로 연동. UGV(SummitXL)와 별개 흐름이라 여기 따로 정리.

### 10-1. 사전 준비 (서브모듈 + 월드 설정)
- 서브모듈 2개 추가됨: `src/webots_ros2_spot`(포크, 다리 제어 코드), `src/webots_spot_msgs`(커스텀 메시지). [2. 설치 및 구성](#2-설치-및-구성-installation)의 `git submodule update --init --recursive`에 이미 포함되어 있어서 별도 조치 불필요.
- `my_world.wbt`에 이미 아래처럼 세팅되어 있어야 함 (새 월드로 옮기거나 처음부터 구성할 때 참고):
  ```
  EXTERNPROTO "../../../../webots_ros2_spot/protos/Spot.proto"
  ...
  DEF Spot Spot {
    translation -0.84 -0.34 0.624
    rotation 0 0 1 0
    name "spot1"
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
  - **`middleExtension`이 뭔가?** `Spot.proto`가 노출하는 확장 슬롯(`frontExtension`/`middleExtension`/`rearExtension`) 중 하나로, proto 파일을 수정하지 않고 월드에서 로봇 몸통(등 중앙부)에 장치를 추가 장착하는 통로. 위의 하향 거리센서 4개는 `float_mode`(제자리 호버링)가 바닥까지의 거리를 재는 데 필요한데, 포크 `Spot.proto`엔 이 센서가 없어서 MASKOR 원본 월드와 동일한 방식·배치로 여기에 꽂아줌. 센서가 없으면 드라이버가 자동 감지해서 `float_mode`만 비활성화되고 나머지(걷기/SLAM)는 정상 동작함.
  - `EXTERNPROTO`는 **로컬 상대경로**여야 함. GitHub raw URL로 참조하면 `Spot.proto` 내부의 `EXTERNPROTO "SpotLeg.proto"`(상대경로)가 "공식 Webots 에셋 아니면 상대경로 추론 안 해줌" 정책에 걸려서 다리가 하나도 안 뜸.
  - 🚨 **커밋 전 이 줄을 항상 확인할 것.** Webots에서 월드를 저장(`Ctrl+S`)할 때마다 이 줄이 `D:/Document/...` 같은 **절대경로로 자동 변경됨**. 그대로 커밋하면 다른 컴퓨터에서 월드가 안 열림. 원인은 Webots가 "월드의 프로젝트 폴더(`simulator/`) 바깥"에 있는 proto만 절대경로로 정규화하기 때문 (`simulator/protos/` 안에 있는 `VelodyneVLP-16.proto` 등은 상대경로가 유지됨). Webots 옵션으로 끌 수 없으니 `git diff`에서 `D:/`가 보이면 위의 상대경로로 되돌리고 커밋.
  - `supervisor TRUE` 필수 — `spot_driver.py`가 `getFromDef()` 같은 Supervisor 전용 API를 씀. 빠지면 `init()`이 조용히 실패하고 이상한 곳(`touch_fl` 등)에서 크래시남.
  - 🚨 **Webots 씬트리에서 "Spot"을 Add Node로 다시 검색해서 추가하지 말 것.** Webots 기본 내장(스톡) proto가 잡혀서 위 설정이 통째로 날아감. 텍스트 에디터로 `.wbt` 파일을 직접 고치고 `Ctrl+Shift+R`로 리로드하는 방식으로만 수정.

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
Spot의 `/spot1/cmd_vel`은 UGV처럼 진짜 속도(m/s)가 아니라, Bezier 보행의 **걸음 크기(StepLength) 배율**로 쓰임 (`linear.x * 0.15`). UGV 감각으로 `linear.x: 1.0` 이상을 주면 보폭이 너무 커져서 넘어짐 — 그래서 `spot_driver.py`에 `MAX_STEP_LENGTH(0.05)`/`MAX_YAW_RATE(0.5)` 상한 클램프를 걸어둠. 그래도 **권장 입력 범위는 `linear.x`/`angular.z` 둘 다 -0.5~0.5 정도**.
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
- ~~**`float_mode` 서비스가 항상 비활성화됨**~~ → **해결.** MASKOR 원본은 거리 센서 4개(`front_left_dist` 등)를 proto가 아니라 자기 월드 파일에서 Spot의 `middleExtension` 슬롯에 꽂아주고 있었음. 같은 배치를 `my_world.wbt`의 Spot 인스턴스에 추가해서 해결 (드라이버가 자동 감지).
- ~~**맵이 로봇 주변 반경 1.5m 감옥처럼 나옴**~~ → **해결.** `depthimage_to_laserscan`이 아래로 기울어진 뎁스카메라의 "1~2m 앞 바닥"을 장애물로 읽은 것. `pointcloud_to_laserscan` + z 높이 필터로 교체해서 해결 (10-3 참고).
- ~~**주행할수록 위치가 틀어지고 빈 공간에 유령 장애물이 생김**~~ → **해결.** MASKOR 원본 `spot_driver.py`의 odom 계산에 "로봇이 180도 돌아서 스폰"을 전제한 마이너스 부호가 하드코딩되어 있었음. 우리 월드(정방향 스폰)에서는 odom이 이동 방향과 반대로 나와, SLAM이 매 스캔 잘못된 사전 추정에서 출발 → 맵 오염 + 이동량 비례 드리프트. **판별법**: Spot의 odom은 supervisor 정답 좌표 기반이라 원리상 드리프트 0이어야 하므로, `map→odom` 보정량(tf2_echo)이 수십 cm 이상이면 무조건 좌표 변환 버그.
- ~~**맵이 실제 세계와 180도 뒤집혀 그려짐**~~ → **해결.** 위 버그를 "접속 시점 자세 기준 상대좌표"로 고쳤더니, 드라이버가 재접속하던 순간 로봇이 이전 주행 자리에서 ~185도 돌아서 있어서 그 방향이 맵의 기준축이 되어버림 (IMU 정답 yaw와 odom yaw를 대조해 184.8° 차이로 확정). 최종적으로 **UGV `robot_driver.py`와 동일하게 월드 절대좌표를 odom으로 그대로 발행**하도록 변경 → 시작 자세·재시작 순서와 무관하게 맵이 항상 월드와 정렬됨.
- ~~**Nav2 리커버리(spin/backup)가 전부 Abort됨**~~ → **해결.** `nav2.yaml`의 behavior_server에 `local_frame`/`robot_base_frame` 키가 없어서 네임스페이스 없는 기본값(`odom`/`base_link`)을 찾다 실패한 것. 키 추가 + `nav2.launch.py` 재작성 규칙에 `local_frame` 포함으로 해결. **UGV에도 잠재해 있던 버그라 UGV 리커버리도 함께 고쳐짐.**

### 10-7. UGV와 다른 점 / Spot에서 고려해야 할 사항

| 항목 | UGV (SummitXL) | Spot |
|---|---|---|
| 이동 방식 | 바퀴 (메카넘) | 다리 (Bezier 보행) |
| `cmd_vel` 의미 | 진짜 속도(m/s) | **걸음 크기 배율** (10-4 참고, ±0.5 권장) |
| 넘어짐 | 불가능 | **가능** — 큰 cmd_vel, 충돌, 급회전에 넘어질 수 있음 |
| 주 센서 | Velodyne 라이다 (360도, ~50m) | 뎁스카메라 5개 합성 (10m, 카메라 사이 사각지대 있음) |
| odom 출처 | GPS+IMU 장치값 (월드 절대좌표) | supervisor 정답값 (월드 절대좌표, 동일 컨벤션으로 통일함) |

**운용할 때 주의할 것들:**
1. **넘어지면 복구가 안 됨** — 넘어진 뒤에는 stand_up으로도 못 일어나는 경우가 많고, odom은 정답값이라 넘어진 자세를 그대로 반영해 SLAM/Nav2가 이상해짐. 넘어지면 Webots 월드 리로드(`Ctrl+Shift+R`) + `docker compose restart spot1`이 가장 빠른 복구.
2. **Nav2 속도 튜닝 여지** — Nav2는 cmd_vel을 진짜 속도로 알고 보내는데 Spot은 걸음 배율로 해석 + 상한 클램프 때문에 Nav2 기대보다 느리게 이동함. 경로 추종이 답답하거나 리커버리가 자주 돌면 `nav2.yaml`의 속도/가속 상한을 Spot용으로 낮추는 튜닝 필요 (현재는 UGV와 같은 파라미터 공유).
3. **footprint 미조정** — `nav2.yaml`의 로봇 footprint가 UGV 기준(0.7×0.5m)임. Spot은 다리 벌림 폭이 달라서 좁은 통로 통과/충돌 여유 판단이 부정확할 수 있음.
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

> SLAM/Nav2는 아직 붙이지 않았다. 드론에 거리 측정 센서가 없어서다 (단계 3에서 추가 예정).

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

- PROTO: `workspace/simulator/protos/Mavic2ProMedium.proto`
- 메시·텍스처: `workspace/simulator/protos/Mavic2Pro/` (아래 11-4 참고)
- 월드 배치: `DEF DRONE1 Mavic2ProMedium { translation -6.5 5.5 0.13, name "drone1" }`

내장 센서는 **GPS, InertialUnit, Gyro, Compass, 짐벌 카메라**(3축, `cameraSlot`). 거리 측정 센서는 없음. 추가하려면 `bodySlot`(동체 고정) 또는 `cameraSlot`(짐벌 장착)을 쓴다 — SummitXL에 Velodyne 다는 방식과 동일.

### 11-4. 해결된 이슈 (트러블슈팅 기록)

1. **드론이 투명하게 보임** — Webots R2025a는 메시·텍스처 에셋을 설치본에 포함하지 않는다(`projects/robots/dji/mavic/` 폴더 자체가 없음). `webots://` 경로 참조가 조용히 실패해 셰이프가 하나도 안 그려졌다. 물리는 정상 동작해서 "안 보이는데 날아다니는" 상태가 됐다. → 메시 14개 + 텍스처를 `protos/Mavic2Pro/`에 로컬 포함하고 상대 경로로 참조.

2. **고도가 0.2~4.2m로 진동** — 순정 Mavic 데모 월드는 `WorldInfo.defaultDamping`(linear/angular 0.5)에 의존해 안정화되는데 `my_world.wbt`에는 그 설정이 없었다. 격리 테스트로 원인을 분리한 결과 **`basicTimeStep`은 무관**(8ms로 낮춰도 동일하게 진동), **댐핑이 원인**. → 전역으로 켜면 UGV/Spot 물리까지 바뀌므로 **드론 PROTO의 `Physics.damping`에만** 동일 값을 걸었다. 시뮬레이션 속도 손해 없음.

3. **고도 39% 오버슈트** — 순정 제어 법칙은 고도를 **P항만으로** 제어한다. 이중적분기에 P 제어만 걸면 준안정이라, 위 2번의 댐핑이 사실상 D항을 대신하고 있었다. → 컨트롤러에 `k_vertical_d`(수직 속도 D항, `wb_gps_get_speed_vector()` 사용)를 추가. 2.79m 오버슈트가 사라지고 2.000m에 단조 수렴한다.

4. **Webots GUI에서 월드를 저장하면 드론 컨트롤러가 `"<none>"`으로 바뀜** — Spot.proto 절대경로 변형과 같은 부류의 현상. 저장 후 `controller "mavic2pro_medium"`인지 확인할 것.

### 11-5. 알려진 한계 / 다음 작업

- **거리 센서 없음** — 자율 비행·장애물 회피에 필요. 뎁스카메라(`cameraSlot`, 짐벌 안정화를 공짜로 받음) + 하향 거리센서 조합이 유력. 이게 붙어야 SLAM/Nav2를 연결할 수 있다.
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

### 12-3. 구조 (몸 / 뇌 / 컨테이너)

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
  이름과 자리가 런타임에 정해져 기다려 줄 컨테이너가 없기 때문이다.
  그 로그는 `/tmp/spawned_robots/{robot_id}.log` (fleet 컨테이너 안)

> ⚠️ 컨테이너를 나눠도 **CPU 총량이 늘지는 않는다.** compose에 `cpus`/`memory` 제한이
> 없으면 컨테이너는 호스트 코어를 그냥 공유하고, OS는 컨테이너가 아니라 프로세스를
> 스케줄한다. 컨테이너 경계가 주는 것은 **코어 고정·자원 상한·장애 격리·개별 재시작**이다.

compose는 매니페스트에서 생성한다. 손으로 유지하면 매니페스트와 이중 관리가 된다:

```bash
docker run --rm -v "$PWD:/w" -w /w windows-master \
  python3 src/webots_robot_spawner/scripts/gen_fleet_compose.py --fleet default.yaml
```

`# >>> FLEET GENERATED` 마커 사이만 갈아 끼우므로 `master`/`fleet` 서비스와 주석은
그대로 남는다. `--check`를 붙이면 고치지 않고 최신인지만 확인한다(CI용).

- 맵 병합·RViz 표시는 손댈 것이 없다. 어떻게 태어난 로봇이든 `robot_registrar`로
  등록해서 마스터 입장에선 구분되지 않는다 ([MAP_MERGE.md](MAP_MERGE.md) 참고)
- Docker 소켓을 쓰는 방식은 **일부러 만들지 않았다.** 소켓 경로가 플랫폼마다 달라
  (Linux 유닉스 소켓 / Windows 네임드 파이프 / Mac Desktop VM) 크로스 플랫폼 전제가 깨진다.

### 12-3-1. 기동 순서 (왜 드론만 다른가)

`compose up` 할 때 순서가 중요하다. 세 가지가 얽혀 있다.

| 서비스 | 기다리는 것 | 이유 |
|---|---|---|
| ugv / spot | `fleet`의 **healthcheck** (`service_healthy`) | 소환기가 몸을 다 확정한 뒤 드라이버가 붙어야 한다 |
| **drone** | `fleet`의 **기동만** (`service_started`) | 아래 교착 때문에 먼저 떠야 한다 |

소환기는 몸을 다 확정하면 `/tmp/fleet_ready`를 만들고, fleet의 healthcheck가 그걸 본다.
이게 없으면 **드라이버가 옛 몸에 붙은 직후 소환기가 그 몸을 잔여물로 지워서 드라이버가
끊기고 종료한다**(실측: 접속 t=26s, 제거 t=38s).

드론만 예외인 이유는 교착이다:

```
드론 몸은 비행을 위해 synchronization TRUE
   → compose down 하면 그 몸만 월드에 남는다
   → Webots 가 기다려 줄 컨트롤러가 없어 시뮬을 멈춘다
   → 소환기의 step() 이 막혀 편대 처리를 못 한다
   → /tmp/fleet_ready 가 안 생겨 healthcheck 실패
   → 드론 컨테이너가 안 뜬다 → 처음으로 되돌아감
```

소환기가 스스로 풀 수 없다. 시작할 때 잔여 몸의 `synchronization`을 내려도
**supervisor의 필드 쓰기는 스텝이 돌아야 반영되는데 그 스텝에서 막혀 있다**
(두 번 연속 기동해도 같은 몸이 계속 TRUE로 보이는 것으로 확인). 이 고리를 끊을 수
있는 것은 그 드라이버가 붙는 것뿐이라, 드론 컨테이너만 먼저 띄운다.

대신 `fleet_start_delay`가 20초다. 먼저 뜬 드론이 `robot_registrar`까지 올라올 시간을
줘야 살아있는 드론을 잔여물로 오판해 몸을 지우지 않는다.

### 12-4. 주의사항 / 트러블슈팅

**PROTO는 `IMPORTABLE EXTERNPROTO`로 선언해야 한다.** 일반 `EXTERNPROTO`로는 런타임
주입이 실패한다:
```
ERROR: In order to import the PROTO 'X', first it must be declared in the IMPORTABLE EXTERNPROTO list.
```

**소환된 로봇은 `synchronization FALSE`로 주입된다.** TRUE면 Webots가 뇌 접속까지
시뮬레이션 전체를 멈추는데, 소환기 자신도 같은 시뮬에서 스텝을 밟으므로 같이 멈춰
교착에 빠진다. 단 **드론만** 뇌 접속 확인 후 TRUE로 되돌린다 — 자세 루프가 매 물리
스텝 돌지 않으면 뒤집혀 추락한다 ([drone_setup.md](drone_setup.md) 참고). 그래서
드론의 뇌가 죽으면 시뮬이 멈춘다.

**월드를 재로드하면 뇌들이 죽는다.** `driver` 프로세스가 종료되는데 `ros2 launch`가
되살리지 않는다. `docker compose restart` 로 다시 띄운다. `fleet` 컨테이너는
`restart: unless-stopped`라 스스로 돌아온다.

**Webots를 켠 채 `compose down` 하면 로봇의 몸이 월드에 남는다.** 다시 올릴 때
`stale_body_policy`(기본 `recreate`)가 처리한다:

| 몸의 상태 | 처리 | 이유 |
|---|---|---|
| 뇌가 살아 있음 (`/robot_registry`에 보임) | **그대로 둔다** | 지우면 정상 동작 중인 드라이버가 끊기고 `ros2 launch`가 되살리지 않는다 |
| 뇌가 없음 (잔여물) | **지우고 새로 소환** | 지난 세션 잔여물을 물려받지 않는다 |

생사 판단에 `/robot_registry`를 쓰는 이유는 QoS가 `TRANSIENT_LOCAL`이라 **늦게 구독해도
살아있는 registrar의 명함은 받고, 죽은 것의 명함은 안 오기** 때문이다. 그대로 두고
싶으면 `stale_body_policy: adopt`.

> 뇌만 다시 붙이는 방식(`attach`)은 폐기했다. 장치가 disabled 상태로 남아 센서가
> 죽는다 — 실측: 뇌만 붙인 드론은 `wb_gps_get_values() called for a disabled device`가
> 4726건, 같은 시점에 새로 소환한 드론은 0건이었다.

**despawn은 없다.** 스폰 실패 시 롤백만 한다 — 뇌가 유예 시간 안에 죽으면 몸을 씬
트리에서 되돌려 조종 불가능한 유령 로봇이 쌓이지 않게 한다.

**`ros2 topic hz`를 믿지 말 것.** 로봇이 늘어 노드가 100개를 넘으면 있는 토픽도
"does not appear to be published yet"으로 나온다(CLI가 매번 새 참여자로 discovery를
처음부터 함). rclpy로 직접 구독해 확인한다.

## 향후 계획
- Gemini api 연동
- Drone 추가 → 기체 구성·비행 검증 완료, ROS 2 연동 남음 ([11](#11-drone-중형급-쿼드콥터) 참고)
- ~~로봇 생성 자동화~~ → 완료 (서비스 호출로 UGV/Spot/드론 런타임 소환, 편대는 yaml. [12](#12-로봇-소환-runtime-spawn) 참고)
- 여러 월드 지원 / 점유격자에서 월드 자동 생성 (검토 완료, 착수 전)
- ~~다중 로봇 지도 병합~~ → 완료 (마스터 관제 컨테이너에서 `/map_merged` 발행, 로봇 자동 합류/이탈까지 확인. [맵 병합 구축 기록](MAP_MERGE.md) 참고)
- ~~윈도우 환경도 bridge 네트워크로 전환 테스트~~ → 완료 ([8-2](#8-2-windows-네트워킹-참고사항-웹-개발자용) 참고)
- ~~Spot 추가~~ → 완료 (다리 제어 + 뎁스카메라 SLAM 맵 생성까지 확인, [10-6](#10-6-해결된-이슈-트러블슈팅-기록) 참고)

## 참고 문서 (References)
- Webots 공식 사용자 가이드 (User Guide)
- Webots 공식 레퍼런스 매뉴얼 (Reference Manual)
