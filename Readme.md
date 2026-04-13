# Webots를 활용한 다중 로봇 시뮬레이션

## 0. 사전 요구 사항 (Prerequisites)
- 호스트 장치 운영체제 : 윈도우, Mac
- 도커 운영체제 : Ubuntu 22.04
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
3. `multi_webots/src/Webots-SummitXL/workspace/simulator/worlds` 디렉토리 내의 `worlds` 폴더에 있는 월드 파일(`.wbt`)을 선택하여 엽니다.
4. 상단의 **Play** 버튼(또는 `Step` 버튼)을 눌러 시뮬레이션을 시작하고 로봇들의 동작을 확인합니다.

## 5. X11 설치
도커(Docker)처럼 화면이 없는 환경에서 GUI 프로그램(RViz2, Webots 등)을 띄우려면, 호스트 PC(사용자님의 실제 컴퓨터)에 화면을 그려주는 도화지 역할인 **'X 서버(X Server)'**가 준비

운영체제별로 이 도화지를 깔고 설정하는 방법이 조금씩 다름

---

### 1. Windows (윈도우)
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

### 2. macOS (맥)
현재 맥에서 X11 - rviz2 연동이 상당히 불안한 관계로 vnc로 설치

1. 브라우저에서 **http://localhost:6080** 접속 후 **vnc.html** 클릭
2. 화면 한 가운데 Connect 클릭

---

### 3. Ubuntu (우분투 / 리눅스)
우분투는 테스트 안함

---

## 5. 작동 명령어

### 1. Windows (윈도우)
```bash
# 도커 시작 (rviz용, ugb1, ugv2)
docker-compose up --build -d
docker-compose down

# visual code로 도커 컨테이너 접속하여 목표점 주면 자율주행 시작
ros2 topic pub -1 /ugv1/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv1/map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

ros2 topic pub -1 /ugv2/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv2/map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 2. macOS (맥)
```bash
docker-compose -f docker-compose-mac.yaml up --build -d
docker-compose -f docker-compose-mac.yaml down

# visual code로 도커 컨테이너 접속하여 목표점 주면 자율주행 시작
ros2 topic pub -1 /ugv1/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv1/map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

ros2 topic pub -1 /ugv2/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'ugv2/map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

네임스페이스 및 tf 문제 해결이 필요함

## 6. 

## 참고 문서 (References)
- Webots 공식 사용자 가이드 (User Guide)
- Webots 공식 레퍼런스 매뉴얼 (Reference Manual)
