# Webots를 활용한 다중 로봇 시뮬레이션

## 1. 프로젝트 개요
이 프로젝트는 **Webots** 시뮬레이터를 활용하여 다중 로봇(Multi-Robot) 환경을 구축하고 제어하는 시뮬레이션 워크스페이스입니다. 여러 대의 로봇이 동일한 환경에서 상호작용하거나 협력하여 태스크를 수행하는 알고리즘을 테스트하고 검증할 수 있습니다.

## 2. 사전 요구 사항 (Prerequisites)
시뮬레이션을 원활하게 실행하기 위해 시스템에 Webots가 설치되어 있어야 합니다.

- **Webots 클라우드 공유 및 정보**: [https://webots.cloud/](https://webots.cloud/)
- **Webots 다운로드**: [Cyberbotics 공식 홈페이지](https://cyberbotics.com/)에서 운영체제에 맞는 최신 버전을 다운로드하여 설치해 주세요.

## 3. 설치 및 구성 (Installation)

본 워크스페이스는 여러 ROS 2 패키지와 **Git 서브모듈(Submodule)**을 포함하고 있습니다. 터미널에서 다음 명령어를 통해 저장소를 클론하고 서브모듈을 초기화합니다.

```bash
# 저장소 클론 (서브모듈 포함)
git clone https://github.com/seo2730/multi_webots.git
cd multi_webots

# (이미 클론한 상태라면) 서브모듈 초기화 및 업데이트
git submodule update --init --recursive
```

본 워크스페이스 설정을 위해 터미널에서 다음을 실행합니다.
```bash
cd ~/[다운받은 경로]/multi_webots
# 필요한 경우 컨트롤러 빌드 명령어를 실행합니다. (예: make)
```

## 4. 시뮬레이션 실행 방법 (Usage)
1. 설치된 **Webots** 프로그램을 실행합니다.
2. 상단 메뉴에서 `File` -> `Open World...`를 클릭합니다.
3. `multi_webots` 디렉토리 내의 `worlds` 폴더에 있는 월드 파일(`.wbt`)을 선택하여 엽니다.
4. 상단의 **Play** 버튼(또는 `Step` 버튼)을 눌러 시뮬레이션을 시작하고 로봇들의 동작을 확인합니다.

## 5. X11 설치
도커(Docker)처럼 화면이 없는 환경에서 GUI 프로그램(RViz2, Webots 등)을 띄우려면, 호스트 PC(사용자님의 실제 컴퓨터)에 화면을 그려주는 도화지 역할인 **'X 서버(X Server)'**가 준비되어 있어야 합니다. 

운영체제별로 이 도화지를 깔고 설정하는 방법이 조금씩 다릅니다. 각 OS별 완벽 세팅 가이드를 정리해 드립니다!

---

### 🪟 1. Windows (윈도우)
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
5. **도커 설정:** `-e DISPLAY=host.docker.internal:0` 옵션을 주고 실행합니다.

---

### 🍎 2. macOS (맥)
맥도 예전에는 기본 지원했지만 지금은 빠졌기 때문에 **XQuartz**라는 공식 지원 앱을 깔아야 합니다.

1. **설치:** 터미널을 열고 홈브루(Homebrew)로 설치하거나 공식 홈페이지에서 다운받습니다.
   1. [XQuartz 공식 홈페이지(xquartz.org)](https://www.xquartz.org/) 에 접속합니다.
   2. XQuartz-x.x.x.pkg (최신 버전) 파일을 다운로드하고 실행해서 설치(Next 버튼 연타)를 완료합니다
2. **권한 허용 (필수):**
   * Launchpad에서 `XQuartz`를 실행합니다. -> 없으면 유틸리티 폴더 확인
   * 좌측 상단 메뉴바에서 `XQuartz` -> `설정(Preferences)` -> `보안(Security)` 탭으로 이동합니다.
   * 🌟 **"네트워크 클라이언트에서 연결 허용 (Allow connections from network clients)"**을 체크합니다.
3. **재시작:** 설정 적용을 위해 맥을 재부팅하거나 해당 계정을 로그아웃/로그인합니다.
4. **접속 허용:** 맥 터미널을 열고 아래 명령어를 쳐서 도커의 접속을 허락해 줍니다.
   ```bash
   xhost + 127.0.0.1
   # 또는 보안 상관없이 다 열려면 xhost +
   ```
5. **도커 설정:** 윈도우와 동일하게 `-e DISPLAY=host.docker.internal:0` 옵션을 줍니다.

---

### 🐧 3. Ubuntu (우분투 / 리눅스)
우분투는 테스트 안함

---

### 💡 요약
* **Windows:** VcXsrv 설치 후 `Disable access control` 체크하여 실행
* **Mac:** XQuartz 설치 후 설정에서 네트워크 허용

사용하시는 OS에 맞춰서 도화지(X 서버)를 깔아두시면, 도커 안에 있는 RViz2가 알아서 화면을 멋지게 그려낼 겁니다!

## 6. 작동 명령어
```bash
ros2 launch webots_python webots_launch.py
ros2 launch slam_gmapping slam_gmapping.launch.py namespace:=ugv1
ros2 launch navigation nav2.launch.py namespace:=ugv1 map_subscribe_transient_local:=true use_sim_time:=true
```

네임스페이스 및 tf 문제 해결이 필요함

## 5. 참고 문서 (References)
- Webots 공식 사용자 가이드 (User Guide)
- Webots 공식 레퍼런스 매뉴얼 (Reference Manual)