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
https://github.com/marchaesen/vcxsrv/releases

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