# 00. 빠른 시작 — 로봇을 움직여 보기까지

> 📖 [책 목차](Readme.md#-목차) · [01. 인터페이스 총람](01_INTERFACES.md) →

**나머지 문서를 하나도 안 읽은 상태에서** 시뮬레이션을 띄우고, 로봇이 스스로 목표점까지
가는 것을 보고, 로봇을 한 대 더 늘리는 데까지 가는 것이 이 장의 목표다.
설치 시간을 빼면 **20~30분**쯤 걸린다.

원리는 설명하지 않는다. "왜 이렇게 되어 있나"가 궁금해지면 [마지막 절](#8-다음에-볼-것)에
어디로 갈지 적어 뒀다.

## 목차
- [0. 준비물](#0-준비물)
- [1. 내려받기](#1-내려받기)
- [2. 화면 준비 (OS별, 한 번만)](#2-화면-준비-os별-한-번만)
- [3. Webots에서 월드 열기](#3-webots에서-월드-열기)
- [4. 컨테이너 띄우기](#4-컨테이너-띄우기)
- [5. 로봇이 떴는지 확인](#5-로봇이-떴는지-확인)
- [6. 첫 자율주행 — 목표점 주기](#6-첫-자율주행--목표점-주기)
- [7. 로봇 한 대 더 늘리기](#7-로봇-한-대-더-늘리기)
- [8. 다음에 볼 것](#8-다음에-볼-것)
- [9. 튜토리얼에서 자주 막히는 곳](#9-튜토리얼에서-자주-막히는-곳)

---

> 📌 **아래에서 `{os}` 는 `ubuntu` / `windows` / `mac` 중 본인 것**으로 바꿔 읽는다.
> 컨테이너 이름도 마찬가지다 (`ugv1_brain_windows` 처럼).
>
> 📌 **줄 끝의 `\` 는 "줄 바꿈"이라는 뜻**이고 bash 문법이다. **PowerShell에서는 백틱(`` ` ``)**,
> cmd에서는 `^` 를 쓴다. 헷갈리면 **`\` 를 지우고 한 줄로 붙여서** 실행해도 똑같이 동작한다
> (셸별 표는 [02장 2절](02_WORLD_GEN.md#2-os별-실행-방법-중요)).

## 0. 준비물

| | 무엇 | 확인 |
|---|---|---|
| 1 | **Docker** (Desktop 또는 Engine) | `docker --version` |
| 2 | **Webots 2025a** — [cyberbotics.com](https://cyberbotics.com/)에서 받는다. 버전이 다르면 월드가 안 열릴 수 있다 | 실행해서 버전 확인 |
| 3 | **git** | `git --version` |

> 💡 **Webots는 호스트(내 PC)에서 돌고, ROS 2는 컨테이너에서 돈다.** 둘이 TCP로 붙는
> 구조라 Webots를 도커 안에 넣지 않는다.

---

## 1. 내려받기

서브모듈이 있으므로 `--recursive`를 빠뜨리면 안 된다.

```bash
git clone --recursive https://github.com/seo2730/multi_webots.git
cd multi_webots
```

이미 받았는데 서브모듈이 비어 있다면:

```bash
git submodule update --init --recursive
```

---

## 2. 화면 준비 (OS별, 한 번만)

RViz 같은 GUI가 **컨테이너 안에서** 돌기 때문에, 그 화면을 내 PC로 받아오는 준비가 필요하다.
OS마다 다르다.

| OS | 할 일 |
|---|---|
| **Ubuntu** | 터미널에서 `xhost +local:root` — **재부팅할 때마다** 다시 해야 한다 |
| **Windows** | [VcXsrv](https://github.com/marchaesen/vcxsrv) 설치 → `XLaunch` 실행 → `Multiple windows`, Display number `0` → `Start no client` → **`Disable access control` 반드시 체크** → 마침. 트레이에 `X` 아이콘이 뜨면 성공 |
| **macOS** | 준비 불필요. 나중에 브라우저로 `http://localhost:6080` 접속해서 본다 |

> 🚨 **Windows에서 `Disable access control`을 체크하지 않으면** 컨테이너 화면이 거부돼서
> RViz가 안 뜬다. 가장 흔한 실수다.

자세한 배경은 [Readme 5절](Readme.md#5-도커-컨테이너-화면gui-띄우기--os별-사전-준비).

---

## 3. Webots에서 월드 열기

1. **Webots 실행**
2. `File` → `Open World...`
3. `src/Webots-SummitXL/workspace/simulator/worlds/` 에서 **`oneroom.wbt`** 선택

> 🚨 **왜 `my_world.wbt` 가 아닌가** — 지금 저장소에 커밋된 compose 3벌은 편대가
> **`oneroom.yaml`** 로 맞춰져 있다(좌표가 ±47 m). `my_world.wbt`(소형 아레나)를 열면
> 로봇이 월드 밖에 소환된다. **월드와 편대는 반드시 짝을 맞춰야 한다** —
> 바꾸는 법은 [Readme 12-2](Readme.md#12-2-편대-매니페스트).

4. 상단 **▶ (Play)** 버튼을 누른다

이 시점에 **로봇은 한 대도 안 보인다. 정상이다.** 이 프로젝트의 월드에는 로봇이 들어 있지
않고, 다음 단계에서 컨테이너가 소환한다.

> 🚨 **Play를 안 누르면 그 뒤 모든 것이 조용히 실패한다.** 시뮬레이션이 멈춰 있으면
> 데이터가 한 줄도 안 나온다. 뭔가 안 될 때 **가장 먼저 확인할 것**이다.

---

## 4. 컨테이너 띄우기

Webots를 **켜 둔 채로** 새 터미널에서:

```bash
docker compose -f docker-configs/{os}/docker-compose.yml up --build -d
```

처음에는 이미지를 만드느라 **10~20분** 걸린다 (그다음부터는 `--build` 없이 몇 초).

띄워지는 것 6개:

| 컨테이너 | 하는 일 |
|---|---|
| `fleet_spawner_{os}` | 편대 yaml대로 Webots에 **로봇 몸을 주입** |
| `ugv1_brain_{os}` · `ugv2_brain_{os}` | UGV 2대의 뇌 (드라이버 + SLAM + Nav2) |
| `spot1_brain_{os}` | 사족보행 Spot의 뇌 |
| `drone1_brain_{os}` | 드론의 뇌 |
| `rviz_master_{os}` | 관제 화면 + 지도 병합 |

**약 30초 기다린다.** 로봇이 바로 안 나오는 게 정상이다 — 소환기가 몸을 다 넣을 때까지
로봇별 컨테이너가 기다리도록 되어 있다.

기다리는 동안 Webots 화면을 보면 로봇 4대가 하나씩 나타난다.

---

## 5. 로봇이 떴는지 확인

컨테이너 안에서 ROS 2 명령을 실행한다.

```bash
docker exec -it ugv1_brain_{os} bash -c \
  "source /ros2_ws/install/setup.bash && ros2 topic list | grep -E '^/(ugv1|spot1|drone1)/(odom|scan|map)$'"
```

이렇게 나오면 성공이다:

```
/drone1/map
/drone1/odom
/spot1/map
/spot1/odom
/spot1/scan
/ugv1/map
/ugv1/odom
/ugv1/scan
```

**시뮬레이션 시계가 도는지**도 본다. 이게 0이면 아래 어떤 것도 동작하지 않는다.

```bash
docker exec -it ugv1_brain_{os} bash -c \
  "source /ros2_ws/install/setup.bash && ros2 topic echo /clock --once"
```

> ⚠️ **`ros2 topic hz` 는 믿지 말 것.** 이 프로젝트는 노드가 100개를 넘어서 그 CLI가
> "발행되지 않음"이라고 **거짓말을 한다.** `echo --once` 로 확인한다.

안 나오면 → [9절](#9-튜토리얼에서-자주-막히는-곳)

---

## 6. 첫 자율주행 — 목표점 주기

`ugv1`은 `oneroom.yaml`에서 **(-47.75, -23.25)** 에 소환된다. 거기서 몇 미터 떨어진
곳을 목표로 준다.

```bash
docker exec -it ugv1_brain_{os} bash -c \
  "source /ros2_ws/install/setup.bash && \
   ros2 topic pub -1 /ugv1/goal_pose geometry_msgs/msg/PoseStamped \
   '{header: {frame_id: \"ugv1/map\"}, pose: {position: {x: -44.0, y: -23.0}, orientation: {w: 1.0}}}'"
```

Webots 화면에서 **ugv1이 스스로 굴러가면 성공**이다. 라이다로 주변을 훑어 지도를 만들면서,
Nav2가 그 지도 위에 경로를 그려 따라간다.

> 🚨 **`frame_id` 를 정확히 `ugv1/map` 으로 채워야 한다.** 로봇마다 좌표계 이름이 다르다.
> 틀리면 **에러 없이 그냥 무시된다.**

### Spot과 드론도 같은 방식이다

```bash
# Spot — (5.50, 21.00) 에서 출발. 느리다 (운용 속도 0.15 m/s)
ros2 topic pub -1 /spot1/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "spot1/map"}, pose: {position: {x: 8.0, y: 21.0}, orientation: {w: 1.0}}}'

# 드론 — 층을 골라서 장애물을 넘어간다
ros2 topic pub -1 /drone1/goal_pose_3d geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "drone1/map"}, pose: {position: {x: -42.0, y: 23.0}, orientation: {w: 1.0}}}'
```

> 🚨 **세 로봇은 겉만 같다.** 토픽 이름과 타입은 같지만 속은 전혀 다르다 — Spot은 보폭으로,
> 드론은 자세 목표로 번역된다. **한 로봇에서 통한 속도값을 다른 로봇에 옮기면 안 된다**
> → [01장](01_INTERFACES.md)의 `cmd_vel` 비교표.

---

## 7. 로봇 한 대 더 늘리기

이 프로젝트의 핵심이다. **Webots를 멈추지도, 월드 파일을 고치지도, compose를 건드리지도
않는다.** 서비스 호출 한 번이면 된다.

```bash
docker exec -it fleet_spawner_{os} bash -c \
  "source /ros2_ws/install/setup.bash && \
   ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot '{type: \"ugv\", random: true}'"
```

응답에 실제로 부여된 이름(`ugv3`)과 좌표가 담겨 나온다. `random: true`라 **지도의 빈 자리를
알아서 고른다.**

새로 소환한 로봇의 로그는 다른 곳에 쌓인다 (매니페스트에 없는 로봇은 `fleet` 컨테이너가
뇌까지 띄우기 때문):

```bash
docker exec -it fleet_spawner_{os} tail -f /tmp/spawned_robots/ugv3.log
```

### 관제 화면 보기

`rviz_master_{os}` 컨테이너가 RViz를 띄우고, 로봇별 지도를 **하나의 전역 지도**로 합쳐
`/map_merged`로 발행한다. 방금 늘린 `ugv3`도 **마스터를 재시작하지 않고** 자동으로 합류한다.

- Ubuntu / Windows: RViz 창이 알아서 뜬다
- macOS: 브라우저에서 `http://localhost:6080` → `vnc.html` → `Connect`

---

## 8. 다음에 볼 것

여기까지 왔으면 이제 목적에 맞는 장으로 간다.

| 하고 싶은 것 | 어디로 |
|---|---|
| 토픽·서비스로 **뭘 주고받는지** 전부 보기 | [01. 인터페이스 총람](01_INTERFACES.md) |
| **다른 지형**에서 돌리기 / 월드 새로 만들기 | [02. 월드 생성](02_WORLD_GEN.md) |
| 편대 구성 바꾸기, **새 로봇 종류** 추가 | [03. 로봇 소환](03_SPAWNER.md) |
| 로봇이 **어떻게 움직이는지** (기준 로봇부터) | [04. UGV 구성](04_UGV_SETUP.md) |
| **웹/외부 연동** 붙이기 | [01장](01_INTERFACES.md) → [Readme 8절](Readme.md#8-외부-연동-웹-목표점--gemini) → [10. 맵 병합](10_MAP_MERGE.md) |
| 학습용 **데이터셋** 만들기 | [11. 데이터 수집](11_DATA_COLLECTION.md) |

**종료할 때:**

```bash
docker compose -f docker-configs/{os}/docker-compose.yml down
```

> 월드에 로봇 몸이 남아 있어도 괜찮다. 다음에 띄울 때 소환기가 정리한다.

---

## 9. 튜토리얼에서 자주 막히는 곳

| 증상 | 원인 / 조치 |
|---|---|
| **로봇이 안 보인다** | ① Webots가 **Play(▶)** 상태인가 ② 30초 기다렸는가 ③ `docker logs fleet_spawner_{os}` |
| **로봇이 이상한 데(허공/벽 속)에 있다** | 월드와 편대가 안 맞는다. `oneroom.wbt`를 열었는지 확인 ([3절](#3-webots에서-월드-열기)) |
| **`/clock`이 안 나온다** | Webots가 멈춰 있거나, `ugv1` 컨테이너가 안 떴다 (시계는 `ugv1`이 발행한다) |
| **목표점을 줬는데 아무 일도 없다** | `frame_id`가 정확히 `{로봇이름}/map`인가. 틀리면 **에러 없이 무시**된다 |
| **RViz가 안 뜬다 (Windows)** | VcXsrv의 **`Disable access control`** 체크를 빠뜨렸다 ([2절](#2-화면-준비-os별-한-번만)) |
| **`ros2 topic hz`가 "not published yet"** | 노드가 100개를 넘으면 그 CLI가 거짓말한다. `echo --once`로 확인 |
| **Webots에서 월드를 저장했더니 안 열린다** | `EXTERNPROTO` 경로가 절대경로로 바뀌었다. `git diff`에서 `D:/`가 보이면 되돌린다 |
| **드론이 이륙을 못 한다** | 컨테이너를 다시 올린다. 드론만 매 물리 스텝 제어 루프가 돌아야 한다 |

그래도 안 되면 → **[Readme 13. 문제가 생겼을 때](Readme.md#13-문제가-생겼을-때)** 에
5단계 점검 순서와 증상별 색인이 있다.

---

[📖 책 목차](Readme.md#-목차) | [01. 인터페이스 총람](01_INTERFACES.md) →
