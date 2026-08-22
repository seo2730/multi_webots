# Webots를 활용한 다중 로봇 시뮬레이션

ROS 2 Humble + Docker 위에서 **UGV · 사족보행(Spot) · 드론**을 한 월드에 띄우고, 각자
SLAM을 돌리며 자율주행하고, 그 지도를 하나로 합쳐 관제하는 시뮬레이션 환경.

세 가지가 이 프로젝트의 뼈대다.

| | 뜻 |
|---|---|
| **월드에는 로봇이 없다** | 환경 + 소환 노드만 있고, 로봇은 **실행 중에 서비스 호출로** 들어온다 |
| **몸과 뇌가 분리돼 있다** | 몸(Webots 노드)은 `fleet` 컨테이너가 주입하고, 뇌(driver + SLAM + Nav2)는 로봇별 컨테이너가 돌린다 |
| **로봇 3종은 겉만 같다** | `cmd_vel`은 셋 다 m/s지만 내부 환산이 전혀 다르다. 한 로봇에서 통한 값을 다른 로봇에 그대로 옮기면 안 된다 |

---

## 📖 목차

> ### 🚀 처음이라면 → **[00. 빠른 시작](00_QUICKSTART.md)**
>
> 나머지를 하나도 안 읽은 상태에서 **시뮬레이션을 띄우고, 로봇이 스스로 목표점까지 가는
> 것을 보고, 로봇을 한 대 더 늘리는** 데까지 20~30분. 아래 목차는 그다음에 봐도 된다.

이 문서는 **설치 · 실행 · 빠른 사용법**을 다룬다. 그 뒤의 **12개 장**이 각 주제의 전체 기록
(왜 그렇게 만들었는지, 어떤 함정이 있었는지, 어떻게 검증했는지)이다.

### 안내서 — 이 문서

| 절 | 내용 |
|---|---|
| [0. 사전 요구 사항](#0-사전-요구-사항-prerequisites) · [1. Webots 설치](#1-webots-설치) · [2. 설치 및 구성](#2-설치-및-구성-installation) | 준비 |
| [3. 시뮬레이션 실행 방법](#3-시뮬레이션-실행-방법-usage) · [4. 로봇 추가 방법](#4-로봇-추가-방법) | 첫 실행 |
| [5. 화면(GUI) 띄우기 — OS별](#5-도커-컨테이너-화면gui-띄우기--os별-사전-준비) · [6. 실행 명령어](#6-실행-명령어-docker-compose) | 도커 |
| [7. 로봇 위치 및 맵 데이터](#7-로봇-위치-및-맵-데이터) · [8. 외부 연동 (웹/Gemini)](#8-외부-연동-웹-목표점--gemini) | 좌표계와 연동 |
| [9. 파이썬 파일 추가 시](#9-파이썬-파일을-추가-시-해야할-것) | 개발 |
| [10. Spot](#10-spot-사족보행-로봇) · [11. Drone](#11-drone-중형급-쿼드콥터) · [12. 로봇 소환](#12-로봇-소환-runtime-spawn) | 로봇별 빠른 사용법 |
| [13. 문제가 생겼을 때](#13-문제가-생겼을-때) | **막히면 여기부터** |
| [14. 남은 일](#14-남은-일) | 미착수·튜닝 잔여·정리 부채 |

### 0부. 따라 하기

| 장 | 다루는 것 | 언제 보나 |
|---|---|---|
| **[00. 빠른 시작](00_QUICKSTART.md)** | 내려받기 → 화면 준비 → 월드 열기 → 컨테이너 → **자율주행 → 로봇 늘리기** 까지 한 줄기 | **맨 처음.** 원리는 빼고 결과부터 본다 |

### 1부. 규격 — 무엇을 주고받나

| 장 | 다루는 것 | 언제 보나 |
|---|---|---|
| **[01. 인터페이스 총람](01_INTERFACES.md)** | 토픽·서비스·프레임·QoS·환경변수 **전체 색인**. 세 로봇의 `cmd_vel` 의미 차이표 | "무엇을 보내면 무엇이 나오나". **웹/외부 연동 개발자 1순위** |

### 2부. 무대와 배우 — 환경과 편대

| 장 | 다루는 것 | 언제 보나 |
|---|---|---|
| **[02. 월드 생성](02_WORLD_GEN.md)** | 월드 만들기 4종 (무작위 건물·창고형·점유격자 변환·외부 반입) + OS별 명령어 | 새 지형이 필요할 때 |
| **[03. 로봇 소환](03_SPAWNER.md)** | 몸/뇌/컨테이너 분리, 기동 순서 교착, 빈 자리 고르기, 잔여 몸 정책 | 소환이 뜻대로 안 될 때, **새 로봇 종류를 더할 때** |

### 3부. 로봇 3종

> 🚗 **UGV가 기준 로봇이다.** Spot과 드론은 "UGV와 무엇이 다른가"로 설명되고,
> Nav2·SLAM 설정도 UGV 것을 재사용한다. 04장을 먼저 읽으면 나머지가 빨라진다.

| 장 | 다루는 것 | 언제 보나 |
|---|---|---|
| **[04. UGV 구성](04_UGV_SETUP.md)** | 메카넘 역기구학, 라이다 → 스캔 → SLAM → Nav2 사슬, `/clock` 함정 | **기준 로봇을 알아야 할 때** |
| **[05. Spot 구축](05_SPOT_SETUP.md)** | 래퍼 PROTO, 라이다 없이 뎁스카메라 5개로 스캔 만들기, 해결한 이슈 7건 | Spot의 몸·센서를 손볼 때 |
| **[06. Spot 드라이버 함수](06_SPOT_DRIVER.md)** | `spot_driver.py` 함수별 설명 — 동작 모드 3종, 보행/자세/호버링 | 사족보행 제어 코드를 읽을 때 |
| **[07. Spot 자율주행](07_SPOT_NAV.md)** | 튜닝 기록 — cmd_vel 단위 환산, 보행 케이던스, **측정 방법론의 함정** | Spot이 경로를 못 따라갈 때 |
| **[08. 드론 구축](08_DRONE_SETUP.md)** | Mavic 2 Pro 개조, 2단 비행 제어와 게인 근거, 해결한 이슈 6건 | 비행 거동·게인을 손볼 때 |
| **[09. 드론 자율비행](09_DRONE_NAV.md)** | 층별 지도, 전역 층 선택, 지역 고도 회피 + **직접 테스트하는 명령어** | 목표점을 주거나 고도 회피를 손볼 때 |

### 4부. 관제와 활용

| 장 | 다루는 것 | 언제 보나 |
|---|---|---|
| **[10. 다중 로봇 맵 병합](10_MAP_MERGE.md)** | 로봇별 지도를 `world` 앵커로 `/map_merged`에 합치기, 자동 합류/이탈 | 관제 화면·전역 좌표를 다룰 때 |
| **[11. 데이터 수집 (KITTI)](11_DATA_COLLECTION.md)** | 카메라-라이다 수집 → KITTI 변환, 학습 전 확인할 것 | 학습용 데이터셋을 만들 때 |

---

### 처음 오셨다면

| 목적 | 읽는 순서 |
|---|---|
| **일단 돌려보고 싶다** | **[00. 빠른 시작](00_QUICKSTART.md) 하나면 된다** (이 문서 2·5·6절을 한 줄기로 엮은 것) |
| **웹/외부 연동을 붙인다** | [01장](01_INTERFACES.md) → [7. 좌표계](#7-로봇-위치-및-맵-데이터) → [8. 외부 연동](#8-외부-연동-웹-목표점--gemini) → [10장](10_MAP_MERGE.md) |
| **로봇을 손본다** | [04장(UGV)](04_UGV_SETUP.md) → 해당 로봇 장 |
| **새 로봇 종류를 더한다** | [03장 11절](03_SPAWNER.md#11-새-로봇-종류를-추가하려면) → [05장](05_SPOT_SETUP.md)(래퍼 PROTO 예시) |
| **새 지형이 필요하다** | [02장](02_WORLD_GEN.md) → [12-2-1](#12-2-1-새-월드를-만들어-편대를-올리기까지) |


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
3. `src/Webots-SummitXL/workspace/simulator/worlds/` 에서 월드 파일(`.wbt`) 선택해서 열기 —
   **지금 compose에 맞는 것은 `oneroom.wbt`** (아래 표 참고)
4. 상단의 **Play** 버튼(또는 `Step` 버튼)을 눌러 시뮬레이션 시작

> ⚠️ **월드를 열어도 로봇은 안 보인다.** 정상이다 — 로봇은 컨테이너가 뜬 뒤 소환으로
> 들어온다([4절](#4-로봇-추가-방법)). 그리고 **Play 상태가 아니면 아무것도 안 나온다.**

### 들어 있는 월드

각 월드는 짝이 되는 **편대 매니페스트**가 있다 (좌표가 그 월드 기준이라서).
**월드와 편대가 어긋나면 로봇이 벽 속이나 월드 밖에 소환된다** — 에러는 안 난다.

> 🚨 **지금 커밋된 compose 3벌은 `fleet:=oneroom.yaml` 이다.** 그래서 아무것도 안 고치고
> 띄우려면 **`oneroom.wbt`** 를 열어야 한다. 다른 월드를 쓰려면 `gen_fleet_compose.py`로
> compose를 그 편대에 맞춰 다시 생성한다 → [12-2-1 ②](#12-2-1-새-월드를-만들어-편대를-올리기까지).

| 월드 | 크기 | 어떤 곳 | 짝 편대 |
|---|---|---|---|
| **`oneroom.wbt`** | 76 m | 원룸 하나 + 마당. 안이 텅 비어 있다. **지금 compose 기본값** | **`oneroom.yaml`** |
| `my_world.wbt` | 소형 | 기본 아레나 — 벽 + 가구 40개 | `default.yaml` |
| `arena_s3.wbt` | 100 m | 무작위 생성 건물 (복도 + 방 + 외부 출입구), seed 3 | `arena_s3.yaml` |
| `arena150.wbt` | 150 m | 〃 더 넓은 부지 | `arena150.yaml` |
| `warehouse100.wbt` | 100 m | 창고형 (결정적 생성) | `warehouse100.yaml` |

새 월드를 만드는 법은 [02. 월드 생성](02_WORLD_GEN.md), 만들어서 편대까지 올리는
최단 경로는 [12-2-1](#12-2-1-새-월드를-만들어-편대를-올리기까지).

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
> 별도 문서로 정리해뒀다 → **[11_DATA_COLLECTION.md](11_DATA_COLLECTION.md)**

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
([src/webots_robot_spawner/config/fleet/](src/webots_robot_spawner/config/fleet/) — 7종,
[12-2](#12-2-편대-매니페스트)에 표로 있다). **현재 compose 값은 `oneroom.yaml`이다.**
바꿀 때는 compose의 `fleet:=`를 손으로 고치지 말고 생성기를 돌린다
([12-2-1 ②](#12-2-1-새-월드를-만들어-편대를-올리기까지)) — 로봇 서비스 목록까지 같이 맞춰야
하기 때문이다.

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
  > ⚠️ "map 프레임 원점 = 로봇 스폰 위치"로 오해하기 쉬운데 **아님.** 실제로 이 오해 때문에 맵 병합에서 좌표가 정확히 두 배로 어긋나는 버그가 났었음 ([맵 병합 문서 2장](10_MAP_MERGE.md#2-정렬-설계--world-앵커-프레임)).
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
  ([02_WORLD_GEN.md](02_WORLD_GEN.md#3-3-출입구-yaml)).
- 웹에서 지도 클릭으로 목표점을 보낼 때는 `/web/goal_point`(`geometry_msgs/msg/PointStamped`)로 발행하되, **`frame_id`가 정확히 `{ns}/map`이어야** [web_goal_relay.py](src/webots_goal_bridge/webots_goal_bridge/web_goal_relay.py)가 해당 로봇의 `goal_pose`로 중계함 (다른 frame_id는 무시됨).
- **전역 병합 맵도 있음.** 마스터 관제 컨테이너가 로봇별 맵을 공통 `world` 프레임 기준으로 합쳐 `/map_merged`(`nav_msgs/msg/OccupancyGrid`)로 발행. 새 로봇이 추가되면 마스터 수정·재시작 없이 자동으로 합류함.

> 💡 병합 설계 근거(왜 직접 짰는지), 로봇 자동 발견 구조, 알고리즘, 검증 결과, 트러블슈팅 전체 기록은 별도 문서로 정리해둠 → **[다중 로봇 맵 병합 구축 기록](10_MAP_MERGE.md)**

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

> 🚗 **UGV(SummitXL Steel)** 는 이 프로젝트의 **기준 로봇**이라, 아래 10·11절이
> "UGV와 무엇이 다른가"로 쓰여 있다. UGV 자체의 구성 — 메카넘 `cmd_vel`,
> 라이다 → 스캔 → SLAM → Nav2 사슬, Nav2 파라미터, `/clock`을 `ugv1`만 발행하는 함정 —
> 은 별도 문서로 정리해뒀다 → **[04_UGV_SETUP.md](04_UGV_SETUP.md)**

## 10. Spot (사족보행 로봇)

Boston Dynamics Spot을 [seo2730/webots_ros2_spot](https://github.com/seo2730/webots_ros2_spot)
(MASKOR 포크)로 연동. 여기는 **빠른 사용법만** 두고, 나머지는 장으로 나눠 뒀다.

| 알고 싶은 것 | 어디 |
|---|---|
| 몸 구성 · 센서 · 해결된 이슈 7건 | **[05. Spot 구축 기록](05_SPOT_SETUP.md)** |
| 드라이버 함수별 동작 (보행/자세/호버링) | **[06. Spot 드라이버 함수](06_SPOT_DRIVER.md)** |
| 자율주행 튜닝 (속도·보행 케이던스·Nav2) | **[07. Spot 자율주행](07_SPOT_NAV.md)** |

### 10-1. 실행

`ugv_only.yaml`을 뺀 편대 매니페스트 6종에 `spot1`이 들어 있어서(현재 compose 값은
[`oneroom.yaml`](src/webots_robot_spawner/config/fleet/oneroom.yaml))
[6. 실행 명령어](#6-실행-명령어-docker-compose)의 `up --build -d`로 다른 로봇들과 같이
소환된다. Spot만 한 대 더 띄우려면:

```bash
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'spot', random: true}"
```

로그 보는 곳이 두 가지다 — 매니페스트에 있는 로봇은 자기 컨테이너를 가진다:

```bash
docker logs -f spot1_brain_windows                                        # 매니페스트 로봇
docker exec fleet_spawner_windows tail -f /tmp/spawned_robots/spot2.log   # 런타임 소환 로봇
```

런치 파일은 `ros2 launch webots_spot single_spot_launch.py`
(namespace는 `ROBOT_ID` 환경변수, 기본값 `spot1`).

### 10-2. cmd_vel 사용법 (UGV와 다름, 주의)

Spot의 `/spot1/cmd_vel`은 **UGV와 같은 진짜 단위(m/s, rad/s)** 다. 내부적으로 Bezier 보행의
걸음 크기(StepLength)로 환산되는데, **전진은 비선형**이다 — 보폭이 작을수록 효율이 높아서
계수 하나로는 저속이 33% 초과속했다. 회귀로 얻은 `v = 1.371·L^0.81`을 역산해 쓴다.
회전은 선형(`YAW_PER_RADPS = 8.11`).

| | 값 | |
|---|---|---|
| 최고 안정속 | **0.195 m/s** | `MAX_STEP_LENGTH 0.090`에서 포화 |
| 최저 속도 | **약 0.045 m/s** | `MIN_STEP_LENGTH 0.015`. 더 작게 줘도 이 속도로 나간다 |
| 제자리 회전 | **0.247 rad/s** | `MAX_YAW_RATE 2.0`에서 포화 |
| **Nav2 운용점** | **0.15 m/s** | 아래 경고 참고 |

```bash
ros2 topic pub /spot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10
```

> 🚨 **Nav2 운용점은 0.15 m/s 다.** 최고속 0.195는 직진에서 잰 값이라, 그대로 쓰면 회전이
> 얹히는 순간 여유가 0이 되어 간헐적으로 넘어진다
> ([07장 3절 ⑫](07_SPOT_NAV.md#-주행-중-간헐적으로-넘어진다--임계값이-아니라-확률이었다)).

> 🔄 **2026-08-16 변경.** 예전에는 `linear.x`가 보폭 배율(`×0.15`)이었다. Nav2의 DWB가 그것을
> m/s로 알고 궤적을 예측해 **회전이 예측의 절반만 돌면서 경로를 못 따라갔다.** 환산 계수의
> 근거는 [06장](06_SPOT_DRIVER.md#cmd_vel-단위), 튜닝 전 과정은 [07장](07_SPOT_NAV.md).

> ✅ 위 속도값은 **재측정을 마친 확정치다.** 옛 값들은 벽시계로 재면서 시뮬 속도를 23%
> 고정으로 가정해 최대 4배까지 틀렸었다 ([07장 4절](07_SPOT_NAV.md#-속도를-벽시계로-쟀다)).

### 10-3. 자세 제어 서비스

| 서비스 | 기능 |
|---|---|
| `/spot1/stand_up` · `/spot1/sit_down` · `/spot1/lie_down` | 일어서기 / 앉기 / 눕기 |
| `/spot1/shake_hand` | 악수(재롱) |
| `/spot1/set_height` | 몸높이 조절 |
| `/spot1/float_mode` | 제자리 호버링 (거리센서 4개 사용) |

```bash
ros2 service call /spot1/stand_up webots_spot_msgs/srv/SpotMotion "{override: true}"
```

타입과 전체 규격은 [05장 3절](05_SPOT_SETUP.md#3-자세-제어-서비스)과
[01장 4절](01_INTERFACES.md#4-서비스).

> ⚠️ **넘어지면 복구가 안 된다.** `stand_up`으로도 못 일어나는 경우가 많다. 월드 리로드
> (`Ctrl+Shift+R`) + `docker compose restart spot1`이 가장 빠르다. 넘어짐을 포함한 운용
> 주의사항은 [05장 5절](05_SPOT_SETUP.md#5-ugv와-다른-점--운용할-때-주의할-것).

## 11. Drone (중형급 쿼드콥터)

DJI Mavic 2 Pro를 중형급(6.35 kg)으로 개조한 `Mavic2ProMedium`. UGV·Spot과 동일하게
**`<extern>` 컨트롤러 + ROS 2 드라이버** 구조다. 여기는 **빠른 사용법만** 둔다.

| 알고 싶은 것 | 어디 |
|---|---|
| 기체 개조 · 센서 · 2단 제어 구조 · 해결된 이슈 6건 | **[08. 드론 구축 기록](08_DRONE_SETUP.md)** |
| 자율비행 · 고도 회피 · 직접 테스트하는 법 | **[09. 드론 자율비행](09_DRONE_NAV.md)** |

### 11-1. 실행

다른 로봇과 동일하다. 월드에서 `controller "<extern>"`이므로 **컨테이너를 띄워야 드론이
움직인다** (안 띄우면 Webots가 컨트롤러를 기다리며 멈춘다).

```bash
# 코드가 바뀌었으면 먼저 빌드 (Dockerfile이 COPY로 코드를 넣는다)
docker compose -f docker-configs/windows/docker-compose.yml build
docker compose -f docker-configs/windows/docker-compose.yml up drone1
```

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/drone1/cmd_vel` | 입력 | 속도 명령 (아래 11-2) |
| `/drone1/goal_pose` | 입력 | Nav2 목표점 (고도 고정) |
| `/drone1/goal_pose_3d` | 입력 | **층을 골라서** 가는 목표 (아래 11-3) |
| `/drone1/odom` | 출력 | 위치·자세·속도 (GPS + IMU) |
| `/drone1/camera/image_color` | 출력 | 짐벌 카메라 영상 |
| `/drone1/Velodyne_VLP_16/point_cloud` | 출력 | 라이다 3D 클라우드 |
| `/drone1/map` | 출력 | 층 합집합 지도 (맵 병합 참여) |
| `/drone1/map_active` | 출력 | 현재 순항 고도 한 층 (Nav2가 이걸 본다) |
| `/drone1/altitude_status` | 출력 | 층 선택 근거 로그 |

전체 토픽 규격은 [01장](01_INTERFACES.md).

> **Nav2는 UGV 파라미터를 그대로 공유한다.** 지상 로봇 전제라 못 쓸 줄 알았는데 실측해 보니
> 그대로 동작했다 — Nav2는 `linear.x`/`angular.z`만 쓰고, 드라이버는 `linear.z`가 0이면 목표
> 고도를 유지하기 때문이다. 4 m 목표 3회 SUCCEEDED, 최종 오차 0.13 / 0.20 / 0.15 m,
> 고도는 2.00 m 고정 유지.

### 11-2. cmd_vel 사용법 (UGV와 다름, 주의)

드론은 모터 4개로 6자유도를 다루는 **underactuated** 시스템이라, UGV처럼 Twist를 바퀴 속도로
바로 변환할 수 없다. 드라이버가 2단으로 처리한다.

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
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"   # 전진 1 m/s
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}"   # 상승 0.5 m/s
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{}"                    # 정지(호버)
```

**키보드 조종** — 고도(`R`/`F`) 축이 추가됐고, 좌우가 조향이 아니라 평행이동인 점이 UGV와 다르다.

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

실측 추종 성능 (Webots 헤드리스, 30초): 정지 0.009 m/s 드리프트 · 전진 1.0 → 1.01 m/s(1%) ·
선회 0.5 → 0.50 rad/s(~1%) · 목표 고도 2.0 → 1.98 m(오버슈트 없음). 검증 방법은
[08장 4절](08_DRONE_SETUP.md#4-검증-방법).

### 11-3. 고도 회피 (2.5D 레이어드)

Nav2는 2D라 고도를 계획하지 않는다. 그 **한 축만 바깥에서** 담당해 "장애물을 넘어간다"를
얻는 구조다. Nav2 자체는 손대지 않는다.

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
# 고도 고정으로 가려면 /drone1/goal_pose 를 그대로 쓴다
```

> ⚠️ **2D 내비게이션이지 3D 경로계획이 아니다.** 고도는 **이산적인 층 선택**(1/2/3 m)이고,
> 상승과 수평이동이 섞이지 않는다. 왜 연속 3D로 가지 않았는지와 알려진 한계 전체는
> [09장 8절](09_DRONE_NAV.md#8-알려진-한계).

> 📘 경로계획 모드 3종(`2d` / `2.5d_local` / `2.5d`) 고르는 법과 **직접 테스트하는 명령어**는
> [09장](09_DRONE_NAV.md)에 있다.

## 12. 로봇 소환 (Runtime Spawn)

실행 중인 Webots에 로봇을 **월드 편집 없이** 추가하는 기능. 담당 패키지:
[src/webots_robot_spawner/](src/webots_robot_spawner/) + [src/webots_spawner_msgs/](src/webots_spawner_msgs/)

**월드에는 로봇이 없다.** 환경(아레나·벽·가구)과 소환 전담 노드 `spawn_supervisor` 하나만
있고, 로봇은 전부 소환으로 들어온다. 예전에는 로봇 4대가 월드 파일에 박혀 있어서 한 대 늘릴
때마다 월드 편집 + compose 서비스 추가 + Webots 재시작이 필요했다.

> 📖 **왜 이렇게 나눴는지, 로봇 종류 정의표, 소환 한 번에 일어나는 일, 빈 자리를 고르는 규칙,
> 기동 순서 교착, 잔여 몸 정책, 파라미터 전체 →
> [03. 로봇 소환 구축 기록](03_SPAWNER.md)**

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
필드별 의미와 실패 사유 전체는 [03장 9절](03_SPAWNER.md#9-파라미터-표).

**despawn은 없다.** 스폰 실패 시 롤백만 한다 — 뇌가 유예 시간 안에 죽으면 몸을 씬 트리에서
되돌려 조종 불가능한 유령 로봇이 쌓이지 않게 한다.

### 12-2. 편대 매니페스트

편대는 yaml로 정의한다 ([config/fleet/](src/webots_robot_spawner/config/fleet/)).
**월드와 짝을 이룬다** — 좌표가 그 월드 기준이기 때문이다.

| 매니페스트 | 짝 월드 | 내용 |
|---|---|---|
| **`oneroom.yaml`** ⬅ 현재 compose 값 | `oneroom.wbt` | ugv1 / ugv2 / spot1 / drone1. Spot 튜닝에 쓴 편대 |
| `default.yaml` | `my_world.wbt` | ugv1 / ugv2 / spot1 / drone1 — 예전 월드와 같은 좌표 |
| `random_squad.yaml` | `my_world.wbt` | UGV 3 + Spot 1 + 드론 2, 전부 무작위 배치 |
| `ugv_only.yaml` | `my_world.wbt` | UGV 2대만 — **Spot·드론 없음** (맵 작업용 경량 편대) |
| `arena_s3.yaml` | `arena_s3.wbt` | 무작위 생성 월드 — 생성기가 좌표까지 써준다 |
| `arena150.yaml` | `arena150.wbt` | 〃 (150 m 부지) |
| `warehouse100.yaml` | `warehouse100.wbt` | 창고형 100 m |

```yaml
fleet:
  - {type: ugv,   id: ugv1, x: -6.159, y: 1.263, yaw: -2.910}
  - {type: ugv,   count: 3, random: true}       # 3대를 알아서
  - {type: drone, count: 2, random: true, clearance: 1.0}
spawn_area: [-9.0, -6.0, 9.0, 7.0]              # random 배치 영역
```

`fleet` 컨테이너가 기동하면서 매니페스트대로 소환한다. 바꾸려면 compose의 `fleet` 서비스
command에서 `fleet:=` 값을 바꾼다 (`fleet:=''` 면 자동 소환 없이 서비스만 받음).

> `spawn_area`는 **항상** 지켜진다. 맵이 있으면 그 영역 안에서 장애물까지 피하고, 월드가 비어
> 있는 냉시동(SLAM 맵이 존재할 수 없음)에서는 로봇 간 간격만 보고 고른다.

### 12-2-1. 새 월드를 만들어 편대를 올리기까지

월드를 얻는 방법이 네 가지다. 어느 쪽이든 **`spawn_supervisor` 노드와 래퍼 PROTO의
`IMPORTABLE` 선언**이 들어가야 소환이 되는데, 네 스크립트가 같은 `prepare()` 로직을
공유하므로 신경 쓸 필요는 없다.

| 스크립트 | 용도 |
|---|---|
| [gen_world_random.py](src/webots_robot_spawner/scripts/gen_world_random.py) | **시드마다 다른 건물** — 복도 + 방 + 장애물 산포 |
| [gen_world.py](src/webots_robot_spawner/scripts/gen_world.py) | 넓은 작전 지역을 처음부터 생성 (창고형, 결정적) |
| [gen_world_from_map.py](src/webots_robot_spawner/scripts/gen_world_from_map.py) | SLAM 맵·건물 도면(점유격자) → 월드 |
| [prepare_world.py](src/webots_robot_spawner/scripts/prepare_world.py) | 밖에서 가져온 `.wbt`를 소환 가능 상태로 |

> 📖 옵션·알고리즘·OS별 명령·트러블슈팅은 **[02. 월드 생성](02_WORLD_GEN.md)** 에 있다.
> 아래는 편대를 올리기까지의 **최단 경로**만 적는다.

**① 월드 생성** — 편대 매니페스트가 **같이** 나온다. 생성기는 어디가 비었는지 알기 때문에
좌표까지 써준다 (손으로 고르면 선반 안에 로봇을 놓기 쉽다).

```
docker run --rm -v "${PWD}:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3
```
→ `worlds/arena_s3.wbt` + `config/fleet/arena_s3.yaml` + `config/doorways/arena_s3.yaml`

> ⚠️ 위는 줄바꿈을 `\`로 쓴 예다. **PowerShell은 백틱**, cmd는 `^`를 쓴다 —
> 셸별 정확한 문법은 [02장 2절](02_WORLD_GEN.md#2-os별-실행-방법-중요)에 표로 있다.

만들어지는 것은 **건물 하나가 놓인 부지**다. 실내에만 갇히지 않게 건물을 안쪽으로 물리고
둘레를 바깥 땅으로 남긴다. **방마다** 로봇이 지나갈 출입구가 있고, **바깥에서도** 건물로
들어올 수 있다. 문짝은 달지 않고, 벽이 끊긴 구간이 곧 출입구이며 그 틈의 중앙 좌표를
`config/doorways/`에 남긴다(복도 중심선도 함께 — 순찰 경로 짤 때 쓴다).
개수는 `--corridors` `--links` `--rooms` `--entrances`로 지정하고, 마당 폭은 `--yard`다
→ [02장 3-2-1](02_WORLD_GEN.md#3-2-1-부지와-외부-출입구), [3-3](02_WORLD_GEN.md#3-3-출입구-yaml).

**② compose를 그 편대에 맞추기** — 로봇 서비스와 `fleet:=` 값이 한꺼번에 맞춰진다.

```
docker run --rm -v "${PWD}:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_fleet_compose.py --fleet arena_s3.yaml
```

> 🚨 ②를 건너뛰면 **조용히 어긋난다.** 소환기는 새 편대를, 로봇 컨테이너는 옛 이름을 쓰게
> 된다. 그래서 생성기가 두 곳을 한 번에 고친다.

**③ Webots에서 월드 열기** — `File > Open World...` → `worlds/arena_s3.wbt`

**④ 컨테이너 기동**
```bash
docker compose -f docker-configs/windows/docker-compose.yml up -d
```

편대가 뜨기까지 30초쯤 걸린다. `fleet_start_delay`가 20초라 느린 게 아니라
[기동 순서](03_SPAWNER.md#7-기동-순서-왜-드론만-다른가)를 지키는 중이다.

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
- **뇌(제어·경로)** 는 로봇별 컨테이너가 담당한다. 컨테이너 경계가 있어야 `cpuset`으로
  코어를 고정하거나 `cpus`로 상한을 걸 수 있다
- **매니페스트에 없는 런타임 소환**만 fleet 컨테이너가 뇌까지 띄운다.
  그 로그는 `/tmp/spawned_robots/{robot_id}.log` (fleet 컨테이너 안)

> 🚨 **compose를 손으로 고치지 않는다.** `# >>> FLEET GENERATED` 마커 사이는
> `gen_fleet_compose.py`가 매니페스트에서 생성한다(3벌 모두). 손으로 유지하면 이중 관리가
> 되고, 어긋나도 **아무 에러 없이 조용히 틀린다.** `--check`를 붙이면 고치지 않고 최신인지만
> 확인한다(CI용). 셸별 문법과 이미지 이름은 [02장 2절](02_WORLD_GEN.md#2-os별-실행-방법-중요).

---

## 13. 문제가 생겼을 때

대부분 아래 5단계에서 끝난다. **순서대로** 본다.

| # | 확인 | 아니면 |
|---|---|---|
| 1 | **Webots가 Play(▶) 상태인가** | 멈춰 있으면 `step()`이 안 돌아 아무것도 안 나온다 |
| 2 | **`ros2 topic hz /clock`** | 0 Hz면 시뮬이 멈췄거나 시계 발행자(`ugv1`)가 없다 → [04장 7절](04_UGV_SETUP.md#7-알아-둘-함정) |
| 3 | **QoS** | `TRANSIENT_LOCAL` 토픽을 기본 QoS로 구독하면 **에러 없이** 아무것도 안 온다 → [01장 6절](01_INTERFACES.md#6-qos-주의-목록) |
| 4 | **`ros2 topic hz`를 믿지 말 것** | 노드 100개가 넘으면 CLI가 거짓말한다. rclpy로 직접 구독 → [10장 10절](10_MAP_MERGE.md#10-해결된-이슈-트러블슈팅-기록) |
| 5 | **로그** | 매니페스트 로봇은 `docker logs -f {ns}_brain_{os}`, 런타임 소환 로봇은 fleet 컨테이너 안 `/tmp/spawned_robots/{ns}.log` |

### 증상별 색인

| 증상 | 어디를 보나 |
|---|---|
| `In order to import the PROTO 'X' ...` | 월드에 `IMPORTABLE EXTERNPROTO` 선언이 없다. `prepare_world.py --check` |
| 월드는 열리는데 로봇이 안 나온다 | 매니페스트 이름과 compose의 `fleet:=` 불일치. `gen_fleet_compose.py --check` |
| 편대가 뜨는 데 30초 걸린다 | 정상이다 → [03장 7절](03_SPAWNER.md#7-기동-순서-왜-드론만-다른가) |
| 월드를 재로드했더니 로봇이 죽었다 | `driver`가 종료되고 `ros2 launch`가 되살리지 않는다. `docker compose restart` |
| `compose down` 했더니 몸이 월드에 남았다 | `stale_body_policy` 기본값이 다음 기동 때 정리 → [03장 8절](03_SPAWNER.md#8-잔여-몸-정책-stale_body_policy) |
| 소환한 드론이 이륙을 못 한다 | 동기화가 TRUE로 복원되지 않았다. 드론만 매 물리 스텝 제어 루프가 돌아야 한다 |
| 월드를 저장했더니 다른 PC에서 안 열린다 | `EXTERNPROTO`가 절대경로로 바뀌었다 → [05장 1절](05_SPOT_SETUP.md#1-몸--래퍼-proto) |
| 맵이 안 나온다 / 스캔이 전부 버려진다 | `use_sim_time: True` 누락 → [05장 4절](05_SPOT_SETUP.md#4-해결된-이슈-트러블슈팅-기록) |
| Spot이 경로를 못 따라간다 / 넘어진다 | [07. Spot 자율주행](07_SPOT_NAV.md) |
| 드론이 고도를 안 바꾼다 | 경로계획 모드가 `2d`일 수 있다 → [09장 0절](09_DRONE_NAV.md#0-경로계획-모드-고르기) |
| `/map_merged`가 안 나온다 | [10장 10절](10_MAP_MERGE.md#10-해결된-이슈-트러블슈팅-기록) |

**장별 트러블슈팅 기록** — 각 장 끝에 그 주제의 전체 기록이 있다:
[02장 8절](02_WORLD_GEN.md#8-트러블슈팅) ·
[03장 10절](03_SPAWNER.md#10-트러블슈팅) ·
[04장 7절](04_UGV_SETUP.md#7-알아-둘-함정) ·
[05장 4절](05_SPOT_SETUP.md#4-해결된-이슈-트러블슈팅-기록) ·
[07장 3절](07_SPOT_NAV.md#3-이슈와-해결--코드설정) ·
[08장 5절](08_DRONE_SETUP.md#5-해결된-이슈-트러블슈팅-기록) ·
[09장 7절](09_DRONE_NAV.md#7-겪은-함정-모음) ·
[10장 10절](10_MAP_MERGE.md#10-해결된-이슈-트러블슈팅-기록)

## 14. 남은 일

각 장의 "알려진 한계" 절과 짝을 이룬다. 그쪽에 근거와 시도해 볼 것이 적혀 있다.

### 미착수 — 아직 만들지 않은 것

| 항목 | 지금 상태 | 근거 |
|---|---|---|
| **Gemini API 연동** | `gemini_goal_assigner.py`는 있지만 `setup.py`의 entry_point가 주석 처리돼 있어 실행되지 않는다 | [8. 외부 연동](#8-외부-연동-웹-목표점--gemini) |
| **자율 탐사 (`explore_lite`)** | 미착수. 목표점을 사람이나 웹이 줘야 한다 | — |
| **실제 SLAM 지도 → 월드 왕복 시험** | `gen_world_from_map.py`는 **합성 지도로만** 검증했다. 로봇이 실제로 만든 `/map`을 다시 월드로 굽는 경로는 안 돌려봤다 | [02장 5절](02_WORLD_GEN.md#5-점유격자--월드-gen_world_from_mappy) |
| **드론 연속 3D 경로계획** | 지금은 2.5D 이산 층 선택. 가려면 ① octomap ② 3D 플래너(Nav2엔 없다) ③ 라이다 수직 FOV 보강이 다 필요하다 — **의도적 보류** | [09장 8절](09_DRONE_NAV.md#8-알려진-한계) |
| **Spot 전용 키보드 텔레옵** | UGV·드론은 있는데 Spot은 `cmd_vel`을 직접 발행해야 한다 | [05장 5절](05_SPOT_SETUP.md#5-ugv와-다른-점--운용할-때-주의할-것) |

### 튜닝 잔여 — 동작은 하는데 덜 좋은 것

| 항목 | 남은 이유 |
|---|---|
| **Spot 배회 1.9~2.2배** | 도달률은 3/3인데 경로가 직선거리의 2배. 다음 시도가 지목돼 있다 — 인플레이션을 UGV 값(0.6 / 3.0)으로 되돌리기 → [07장 5절](07_SPOT_NAV.md#5-미해결--다음-작업) |
| **Spot 리커버리 `time_allowance`** | 기본 10초인데 Spot 회전은 0.247 rad/s라 63°밖에 못 돈다. **BT XML 속성이라 파라미터로 못 바꾼다** — 커스텀 BT XML이 필요 → [07장 5절](07_SPOT_NAV.md#5-미해결--다음-작업) |
| **웹 목표점이 로봇별 프레임을 쓴다** | `world` 프레임이 이미 있으니 `web_goal_relay.py`가 tf2로 변환하게 바꾸면 **웹은 `/map_merged` 하나만 그리고 클릭**하면 된다 → [10장 11절](10_MAP_MERGE.md#11-알려진-한계--다음-작업) |
| **실기 이식 시 SLAM 재튜닝** | 지금 odom은 시뮬 정답값이라 드리프트 0. 실기에서는 로봇 간 정렬이 원리적으로 어긋난다 → [10장 11절](10_MAP_MERGE.md#11-알려진-한계--다음-작업) |

### 정리 부채

| 항목 | 내용 |
|---|---|
| **데이터 수집 경로가 편대 구조 이전 상태** | `ROBOT_ID`가 `SummitXLSteel`(현재 편대는 `ugv1`), compose가 맥 전용, 저장 경로 하드코딩 → [11장 5절](11_DATA_COLLECTION.md#5-현재-구조와-어긋난-부분) |
| **KITTI 변환 결과를 실제 로더에 못 물린다** | `.bin`이 3채널(KITTI는 4채널)이라 **에러 없이 조용히 틀린다**, `rotation_y` 전부 0 → [11장 6절](11_DATA_COLLECTION.md#6-학습에-쓰기-전에-확인할-것) |

### 완료한 것

<details>
<summary>펼쳐 보기</summary>

- ~~**Spot 추가**~~ → 다리 제어 + 뎁스카메라 5개 병합 SLAM ([05장](05_SPOT_SETUP.md))
- ~~**Spot 자율주행 튜닝**~~ → **도달률 0~1/3 → 3/3, 목표 오차 0.36~0.38 m, `map`→`odom`
  표류 0, 넘어짐 없음.** cmd_vel을 m/s로 환산, Spot 전용 Nav2 파라미터, 보행 케이던스 노출,
  SLAM TF 제거, 카메라별 스캔 하한, 운용 속도 0.15 m/s ([07장](07_SPOT_NAV.md))
- ~~**Spot 절대 속도 재측정**~~ → 시뮬 시각 기준 최고 안정속 **0.195 m/s**, 제자리 회전
  **0.247 rad/s**. 벽시계로 잰 옛 값은 최대 4배 틀렸다
- ~~**Spot `map`→`odom` 표류**~~ → slam_toolbox가 **정답 odom에 보정을 얹어** 오차를 키우고
  있었다. SLAM은 지도만 만들고 TF는 항등 정적 변환을 쓴다
  ([07장 3절 ⑩](07_SPOT_NAV.md#-mapodom-표류가-목표-오차로-그대로-나왔다))
- ~~**Spot 여러 대 지원**~~ → 소환기가 로봇마다 DEF 이름을 갈라 붙인다
  ([05장 1절](05_SPOT_SETUP.md#1-몸--래퍼-proto))
- ~~**Drone 추가**~~ → 기체 개조·2단 비행 제어·ROS 2 연동 ([08장](08_DRONE_SETUP.md))
- ~~**드론 자율비행**~~ → 라이다 탑재 + 층별 지도 + 전역 층 선택 + 지역 고도 회피.
  목표점 5개 중 4개 도달, 리밋 사이클 0회 ([09장](09_DRONE_NAV.md))
- ~~**로봇 생성 자동화**~~ → 서비스 호출로 UGV/Spot/드론 런타임 소환, 편대는 yaml
  ([03장](03_SPAWNER.md))
- ~~**여러 월드 지원 / 점유격자에서 월드 자동 생성**~~ → 창고형·무작위 건물·점유격자 변환·
  외부 월드 반입 4종. 무작위 생성은 방마다 출입구를, 바깥에서 들어올 외부 출입구를 보장하며
  좌표를 yaml로 남긴다 ([02장](02_WORLD_GEN.md))
- ~~**다중 로봇 지도 병합**~~ → 마스터 관제 컨테이너에서 `/map_merged` 발행, 로봇 자동
  합류/이탈까지 확인 ([10장](10_MAP_MERGE.md))
- ~~**윈도우도 bridge 네트워크로 전환**~~ → [8-2](#8-2-windows-네트워킹-참고사항-웹-개발자용)

</details>

---

## 참고 자료

**서브모듈 문서** — [src/Webots-SummitXL/README.md](src/Webots-SummitXL/README.md) (원본 프로젝트),
[src/webots_ros2_spot/README.md](src/webots_ros2_spot/README.md) (MASKOR 포크)

**바깥 자료** — Webots [User Guide](https://cyberbotics.com/doc/guide/index) ·
[Reference Manual](https://cyberbotics.com/doc/reference/index) ·
[Nav2 문서](https://docs.nav2.org/) · [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

**이 저장소에서 작업한다면** — [CLAUDE.md](CLAUDE.md)에 절대 규칙(크로스 플랫폼 전제,
compose 자동생성, 월드 저장 시 `git diff` 확인, `use_sim_time`)이 정리돼 있다.
