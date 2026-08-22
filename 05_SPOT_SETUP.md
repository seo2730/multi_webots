# 05. Spot(사족보행) 구축 기록

> 📖 [책 목차](Readme.md#-목차) · ← [04. UGV 구성](04_UGV_SETUP.md) · [06. Spot 드라이버 함수](06_SPOT_DRIVER.md) →

Boston Dynamics Spot을 [seo2730/webots_ros2_spot](https://github.com/seo2730/webots_ros2_spot)
(MASKOR/webots_ros2_spot 포크)로 연동하기까지의 기록. **몸을 어떻게 구성했고, 라이다가
없는 로봇에서 어떻게 스캔을 만들었고, 어떤 함정에 빠졌는지**를 다룬다.

드라이버 **함수별 설명**은 [06장](06_SPOT_DRIVER.md), **자율주행 튜닝**은
[07장](07_SPOT_NAV.md)에 있다. 빠른 사용법은 [Readme 10절](Readme.md#10-spot-사족보행-로봇).

- 서브모듈: `src/webots_ros2_spot`(포크, 다리 제어), `src/webots_spot_msgs`(커스텀 메시지)
- 래퍼 PROTO: [`protos/SpotSensorized.proto`](src/Webots-SummitXL/workspace/simulator/protos/SpotSensorized.proto)

## 목차
- [1. 몸 — 래퍼 PROTO](#1-몸--래퍼-proto)
- [2. 센서 구성 (라이다 없음 → 뎁스카메라 5개 병합)](#2-센서-구성-라이다-없음--뎁스카메라-5개-병합)
- [3. 자세 제어 서비스](#3-자세-제어-서비스)
- [4. 해결된 이슈 (트러블슈팅 기록)](#4-해결된-이슈-트러블슈팅-기록)
- [5. UGV와 다른 점 / 운용할 때 주의할 것](#5-ugv와-다른-점--운용할-때-주의할-것)

---

## 1. 몸 — 래퍼 PROTO

🔄 **이 설정은 이제 월드가 아니라 래퍼 PROTO에 들어 있다.**
[`SpotSensorized.proto`](src/Webots-SummitXL/workspace/simulator/protos/SpotSensorized.proto)가
아래 구성을 통째로 품고 있고, 월드에는 `IMPORTABLE EXTERNPROTO` 선언만 있으면 된다.
소환기가 이 PROTO로 몸을 주입한다 ([03장](03_SPAWNER.md)). 예전처럼 월드에 인라인 40줄을
박아 둘 필요가 없다.

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

**`middleExtension`이 뭔가?** `Spot.proto`가 노출하는 확장 슬롯
(`frontExtension`/`middleExtension`/`rearExtension`) 중 하나로, proto 파일을 수정하지 않고
로봇 몸통(등 중앙부)에 장치를 추가 장착하는 통로. 위의 하향 거리센서 4개는
`float_mode`(제자리 호버링)가 바닥까지의 거리를 재는 데 필요한데, 포크 `Spot.proto`엔 이
센서가 없어서 MASKOR 원본 월드와 동일한 방식·배치로 꽂아줬다. 센서가 없으면 드라이버가
자동 감지해서 `float_mode`만 비활성화되고 나머지(걷기/SLAM)는 정상 동작한다. 여기에 **더**
얹고 싶으면 `SpotSensorized`의 `extraMiddleSlot` 필드를 쓴다 (거리센서 4개는 그대로 유지).

### 🚨 지키지 않으면 조용히 깨지는 것들

| 규칙 | 안 지키면 |
|---|---|
| **디바이스 이름을 바꾸지 말 것** | `spot_driver.py`가 `front_left_dist` / `front_right_dist` / `rear_left_dist` / `rear_right_dist`를 **이름으로** 찾는다 |
| **`EXTERNPROTO`는 로컬 상대경로** | GitHub raw URL로 참조하면 `Spot.proto` 내부의 `EXTERNPROTO "SpotLeg.proto"`(상대경로)가 "공식 Webots 에셋 아니면 상대경로 추론 안 해줌" 정책에 걸려 **다리가 하나도 안 뜬다** |
| **`supervisor TRUE` 필수** | `spot_driver.py`가 `getFromDef()` 같은 Supervisor 전용 API를 쓴다. 빠지면 `init()`이 조용히 실패하고 엉뚱한 곳(`touch_fl` 등)에서 크래시난다. `SpotSensorized`는 기본 TRUE |
| **씬트리에서 "Spot"을 Add Node로 다시 추가하지 말 것** | Webots 기본 내장(스톡) proto가 잡혀 위 설정이 통째로 날아간다. 옛 월드를 손볼 때는 텍스트 에디터로 `.wbt`를 직접 고치고 `Ctrl+Shift+R`로 리로드 |

> 🚨 **커밋 전 `EXTERNPROTO` 줄을 항상 확인할 것.** Webots에서 월드를 저장(`Ctrl+S`)할
> 때마다 이 줄이 `D:/Document/...` 같은 **절대경로로 자동 변경된다.** 그대로 커밋하면 다른
> 컴퓨터에서 월드가 안 열린다. 원인은 Webots가 "월드의 프로젝트 폴더(`simulator/`) 바깥"에
> 있는 proto만 절대경로로 정규화하기 때문이다 (`simulator/protos/` 안의
> `VelodyneVLP-16.proto` 등은 상대경로가 유지된다). Webots 옵션으로 끌 수 없으니
> `git diff`에서 `D:/`가 보이면 되돌리고 커밋한다.

### 여러 대 띄우기

`getFromDef()`를 쓰기 때문에 소환기는 **Spot만 `DEF` 이름을 붙여** 주입한다
(`robot_types.py`의 `needs_def`). `spot2` → `DEF SPOT2`처럼 로봇마다 이름이 갈리고, 뇌에는
같은 이름이 `ROBOT_DEF` 환경변수로 전달된다. 그래서 **2대 이상 소환해도 서로 남의 몸을
잡지 않는다** → [03장 3절](03_SPAWNER.md#3-로봇-종류-정의표).

---

## 2. 센서 구성 (라이다 없음 → 뎁스카메라 5개 병합)

Spot에는 UGV의 Velodyne 같은 2D 라이다가 없고, 뎁스카메라 5개(`left_flank_depth`,
`right_flank_depth`, `left_head_depth`, `right_head_depth`, `rear_depth`)만 있다. 그래서:

1. `pointcloud_to_laserscan` 노드 5개가 각 뎁스카메라의 3D 포인트클라우드를
   `{ns}/base_link` 기준으로 변환한 뒤 **z 높이 필터**(`min_height: -0.35` = 지면 위
   ~0.17 m부터만 장애물 인정)를 거쳐 개별 `LaserScan`으로 변환
2. `webots_spot` 패키지의 커스텀 노드 `multi_scan_merger`
   ([multi_scan_merger.py](src/webots_ros2_spot/webots_spot/multi_scan_merger.py))가 tf2로
   5개를 `{ns}/base_link` 기준 하나의 360도 스캔으로 합쳐 `/spot1/scan`으로 발행
3. SLAM Toolbox/Nav2는 이 `/spot1/scan`을 UGV와 완전히 동일한 방식
   (`navigation` 패키지의 `nav2.launch.py` 그대로 재사용)으로 사용

> 🚨 **`depthimage_to_laserscan`을 쓰면 안 된다.** 카메라가 수평이라고 가정하는 노드라,
> 아래로 기울어진 Spot 카메라가 바닥을 장애물로 읽어 로봇 주변에 가짜 원형 벽이 생긴다
> ([4절](#4-해결된-이슈-트러블슈팅-기록)).

> ⚠️ **스캔 하한(`range_min`)은 카메라마다 다르다.** 하나로 묶으면 진행 방향의 하한이
> 통째로 무효가 된다. 카메라별로 갈라 준 이유와 실측은
> [07장 3절 ⑧](07_SPOT_NAV.md#-스캔이-자기-몸-끝을-못-본다).

---

## 3. 자세 제어 서비스

| 서비스 | 타입 | 기능 |
|---|---|---|
| `/spot1/stand_up` | `webots_spot_msgs/srv/SpotMotion` | 일어서기 |
| `/spot1/sit_down` | `webots_spot_msgs/srv/SpotMotion` | 앉기 |
| `/spot1/lie_down` | `webots_spot_msgs/srv/SpotMotion` | 눕기 |
| `/spot1/shake_hand` | `webots_spot_msgs/srv/SpotMotion` | 악수(재롱) |
| `/spot1/set_height` | `webots_spot_msgs/srv/SpotHeight` | 몸높이 조절 |
| `/spot1/float_mode` | `std_srvs/srv/SetBool` | 제자리 호버링 (`middleExtension` 거리센서 4개 사용) |

```bash
ros2 service call /spot1/stand_up webots_spot_msgs/srv/SpotMotion "{override: true}"
```

함수별 동작은 [06장](06_SPOT_DRIVER.md), 전체 서비스 색인은
[01장 4절](01_INTERFACES.md#4-서비스).

---

## 4. 해결된 이슈 (트러블슈팅 기록)

### ① `spot1/odom` TF가 안 올라옴 ✅

원인은 Webots 시뮬레이션이 **Play 상태가 아니었던 것.** 일시정지 상태면 `step()`이 호출되지
않아 TF/odom이 전혀 발행되지 않는다.

> 💡 **시뮬레이션 Play(▶) 상태 확인이 항상 1순위 점검 항목이다.**

### ② `spot1/map`이 안 나옴 (SLAM 맵 생성 실패) ✅

`multi_scan_merger`와 `depthimage_to_laserscan` 노드에 `use_sim_time: True`가 빠져 있어서,
병합 스캔이 **벽시계 시간으로 스탬프**됐다 → 시뮬레이션 시간 기반 TF와 영원히 매칭 안 됨 →
slam_toolbox가 `Message Filter dropping message ... queue is full`을 찍으며 스캔을 전부
버렸다. launch 파일에 `use_sim_time` 추가로 해결(`/spot1/map` 발행 실측 확인).

> 🚨 **새 센서 처리 노드를 추가할 땐 `use_sim_time: True`를 잊지 말 것.**

### ③ `float_mode` 서비스가 항상 비활성화됨 ✅

MASKOR 원본은 거리 센서 4개(`front_left_dist` 등)를 proto가 아니라 **자기 월드 파일에서**
Spot의 `middleExtension` 슬롯에 꽂아주고 있었다. 같은 배치를 추가해 해결(드라이버가 자동
감지). 지금 그 배치는 월드가 아니라 [`SpotSensorized.proto`](src/Webots-SummitXL/workspace/simulator/protos/SpotSensorized.proto)
안에 있다 ([1절](#1-몸--래퍼-proto)).

### ④ 맵이 로봇 주변 반경 1.5 m 감옥처럼 나옴 ✅

`depthimage_to_laserscan`이 아래로 기울어진 뎁스카메라의 "1~2 m 앞 바닥"을 장애물로 읽은
것. `pointcloud_to_laserscan` + z 높이 필터로 교체해 해결 ([2절](#2-센서-구성-라이다-없음--뎁스카메라-5개-병합)).

### ⑤ 주행할수록 위치가 틀어지고 빈 공간에 유령 장애물이 생김 ✅

MASKOR 원본 `spot_driver.py`의 odom 계산에 **"로봇이 180도 돌아서 스폰"을 전제한 마이너스
부호**가 하드코딩되어 있었다. 우리 월드(정방향 스폰)에서는 odom이 이동 방향과 반대로 나와,
SLAM이 매 스캔 잘못된 사전 추정에서 출발 → 맵 오염 + 이동량 비례 드리프트.

> 💡 **판별법** — Spot의 odom은 supervisor 정답 좌표 기반이라 원리상 드리프트가 0이어야
> 한다. 그러므로 `map→odom` 보정량(`tf2_echo`)이 수십 cm 이상이면 **무조건 좌표 변환
> 버그**다.

### ⑥ 맵이 실제 세계와 180도 뒤집혀 그려짐 ✅

⑤를 "접속 시점 자세 기준 상대좌표"로 고쳤더니, 드라이버가 재접속하던 순간 로봇이 이전 주행
자리에서 ~185도 돌아서 있어서 **그 방향이 맵의 기준축이 되어버렸다**(IMU 정답 yaw와 odom
yaw를 대조해 184.8° 차이로 확정). 최종적으로 **UGV `robot_driver.py`와 동일하게 월드
절대좌표를 odom으로 그대로 발행**하도록 바꿔서, 시작 자세·재시작 순서와 무관하게 맵이 항상
월드와 정렬된다.

### ⑦ Nav2 리커버리(spin/backup)가 전부 Abort됨 ✅

behavior_server가 네임스페이스 없는 기본값(`odom`/`base_link`)을 찾다 실패한 것. 처음엔
`local_frame` 키를 넣어 고쳤다고 봤는데 **Humble의 `nav2_behaviors`에 그런 파라미터는 없어서
조용히 무시되고 있었다.** 실제 이름은 `global_frame`이고, RewrittenYaml의 치환값(`{ns}/map`)이
behavior_server에는 맞지 않아 `nav2.launch.py`가 그 노드에서만 `{ns}/odom`으로 덮어쓴다.
자세한 증상·근거는 [04장 5절](04_UGV_SETUP.md#5-nav2-파라미터에서-실제로-중요한-값들).

> **UGV·드론에도 잠재해 있던 버그라 셋 다 함께 고쳐졌다.**

---

## 5. UGV와 다른 점 / 운용할 때 주의할 것

| 항목 | UGV (SummitXL) | Spot |
|---|---|---|
| 이동 방식 | 바퀴 (메카넘) | 다리 (Bezier 보행) |
| `cmd_vel` 의미 | 진짜 속도(m/s) | 진짜 속도(m/s) — **0.045~0.195 m/s** 구간만 |
| 넘어짐 | 불가능 | **가능** — 큰 cmd_vel, 충돌, 급회전에 넘어질 수 있음 |
| 주 센서 | Velodyne 라이다 (360도, ~50 m) | 뎁스카메라 5개 합성 (10 m, 카메라 사이 사각지대 있음) |
| odom 출처 | GPS+IMU 장치값 (월드 절대좌표) | supervisor 정답값 (월드 절대좌표, 동일 컨벤션으로 통일) |
| Nav2 파라미터 | `nav2.yaml` | **`nav2_spot.yaml` (전용)** |

**운용할 때 주의할 것들:**

1. **넘어지면 복구가 안 된다** — 넘어진 뒤에는 `stand_up`으로도 못 일어나는 경우가 많고,
   odom은 정답값이라 넘어진 자세를 그대로 반영해 SLAM/Nav2가 이상해진다. 넘어지면 Webots
   월드 리로드(`Ctrl+Shift+R`) + `docker compose restart spot1`이 가장 빠른 복구다.
2. **Nav2 파라미터는 Spot 전용이 따로 있다** — `navigation/param/nav2_spot.yaml`. cmd_vel
   단위를 m/s로 고치고 속도·가속·footprint를 Spot에 맞췄다. 근거와 실측표는 그 파일 머리말과
   [07장](07_SPOT_NAV.md)에 있다.
3. **footprint는 길이만 늘렸다** — 1.1×0.5 m(몸통 실측). **폭은 일부러 UGV와 같게 뒀다** —
   1.2×0.6으로 키웠더니 내접 반경이 0.30 m가 되어 전역 플래너가 가구 사이를 통행 불가로 보고
   `failed to create plan`을 냈다
   ([07장 3절 ④](07_SPOT_NAV.md#-footprint-폭을-키웠더니-경로가-아예-안-나왔다)).
4. **뎁스카메라 사각지대** — 카메라 5개가 대부분 방향을 커버하지만 FOV 사이 틈이 있어, 정확히
   사각에 있는 얇은 장애물은 스캔에 안 잡힐 수 있다.
5. **새 센서 노드 추가 시 `use_sim_time: True` 필수** — 빠뜨리면 벽시계 스탬프 때문에 SLAM이
   데이터를 전부 버린다 ([4절 ②](#4-해결된-이슈-트러블슈팅-기록)).
6. **실제 로봇 이식 시** — 지금 odom은 시뮬레이션 정답값이라 드리프트가 0이다. 실기에서는
   센서 기반 추정 odom(드리프트 있음)으로 바뀌므로 SLAM 파라미터(보정 강도 등)를 다시 튜닝해야
   한다. 같은 이야기가 맵 병합에도 있다 → [10장 11절](10_MAP_MERGE.md#11-알려진-한계--다음-작업).

### 아직 없는 것

- **Spot 전용 키보드 텔레옵이 없다.** UGV는 `simulator keyboard`, 드론은
  `simulator drone_teleop`이 있지만 Spot은 `cmd_vel`을 직접 발행해야 한다.
- **실제 BD Spot 스펙(순항 1.3 m/s)은 현재 구조로 도달 불가** — 원인은 균형이 아니라 시간
  해상도(`basicTimeStep` 32 ms)다. 월드 전체에 영향을 주는 결정이라 바꾸지 않았다
  → [07장 5절](07_SPOT_NAV.md#5-미해결--다음-작업).

---

← [04. UGV 구성](04_UGV_SETUP.md) | [📖 책 목차](Readme.md#-목차) | [06. Spot 드라이버 함수](06_SPOT_DRIVER.md) →
