# UGV(SummitXL Steel) 구성 기록

메카넘 바퀴 UGV는 이 프로젝트의 **기준 로봇**이다. Spot과 드론은 "UGV와 무엇이 다른가"로
설명되고, Nav2·SLAM 설정도 UGV 것을 그대로 재사용한다. 그런데 정작 UGV 자체는 문서가
Readme 여기저기에 흩어져 있어서 한 곳에 모았다.

- 몸: `SummitXlSteelSensorized.proto` (원본 `SummitXlSteel` + 센서 래퍼)
- 뇌: [single_ugv.launch.py](src/webots_python/launch/single_ugv.launch.py)
- 드라이버: [robot_driver.py](src/Webots-SummitXL/workspace/simulator/simulator/robot_driver.py) (서브모듈)

## 목차
- [1. 전체 구조](#1-전체-구조)
- [2. cmd_vel — 메카넘 역기구학](#2-cmd_vel--메카넘-역기구학)
- [3. odom과 프레임](#3-odom과-프레임)
- [4. 센서 → 스캔 → SLAM → Nav2 사슬](#4-센서--스캔--slam--nav2-사슬)
- [5. Nav2 파라미터에서 실제로 중요한 값들](#5-nav2-파라미터에서-실제로-중요한-값들)
- [6. 키보드 조종](#6-키보드-조종)
- [7. 알아 둘 함정](#7-알아-둘-함정)
- [8. 알려진 한계](#8-알려진-한계)
- [9. 파일 맵](#9-파일-맵)

---

## 1. 전체 구조

```
[Webots]                                  [ugv1_brain_* 컨테이너]

SummitXlSteelSensorized {  (소환)          robot_state_publisher   (URDF → TF)
  controller "<extern>"  ←── TCP 1234 ──→  webots_ros2_driver
}                                            └─ robot_driver.py (플러그인)
  ├─ 메카넘 바퀴 4 (RotationalMotor          │     init() 1회 / step() 매 32ms
  │  + PositionSensor)                       ↓
  ├─ Velodyne VLP-16 (360°, ~50 m)     pointcloud_to_laserscan  (3D → 2D 스캔)
  ├─ GPS / IMU / Compass / Gyro / Accel      ↓
  └─ rgb_camera                        slam_toolbox (mapping)   → /ugv1/map
                                             ↓
                                       Nav2 (3초 지연 기동)
                                       web_goal_relay · robot_registrar
```

Spot·드론과 마찬가지로 일반 ROS 2 노드가 아니라 **Webots가 매 스텝 `step()`을 직접
호출하는 `webots_ros2_driver` 플러그인**이다.

---

## 2. cmd_vel — 메카넘 역기구학

UGV의 `cmd_vel`은 **진짜 속도(m/s)** 다. 셋 다 이제 m/s를 받지만, 그 값이 바퀴에
닿기까지가 다르다 — Spot은 보폭으로 비선형 환산되고(그래서 좁은 속도 구간만 낼 수 있다),
드론은 자세 목표로 바뀐다. UGV만 **대수 변환 한 줄**로 바퀴 속도가 나온다.

```python
WHEEL_RADIUS = 0.123
LX, LY = 0.2045, 0.2225          # 바퀴 배치 반폭 (m)

fl = 1/WHEEL_RADIUS * (vx - vy - (LY + LX) * wz)
fr = 1/WHEEL_RADIUS * (vx + vy - (LY + LX) * wz)
bl = 1/WHEEL_RADIUS * (vx + vy + (LY + LX) * wz)
br = 1/WHEEL_RADIUS * (vx - vy + (LY + LX) * wz)
```

| 필드 | 의미 |
|---|---|
| `linear.x` | 전후 속도 (m/s) |
| `linear.y` | **좌우 평행이동** (m/s) — 메카넘이라 기체를 돌리지 않고 옆으로 간다 |
| `angular.z` | 선회 각속도 (rad/s) |

```bash
ros2 topic pub /ugv1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```

> `linear.y`는 UGV만의 능력이다. 드론도 `linear.y`를 받지만 **기체가 기울어서** 이동하고,
> Spot은 게걸음(`LateralFraction`)으로 번역된다. 세 로봇의 `cmd_vel`이 같은 타입이면서
> 의미가 다르다는 점은 [INTERFACES.md](INTERFACES.md)에 한 표로 모아 뒀다.

---

## 3. odom과 프레임

드라이버는 **GPS 원값(= Webots 월드 절대좌표)과 IMU yaw를 그대로** `odom → base_link`로
발행한다. 추측항법 적분이 아니다.

```python
t.transform.translation.x = float(gps_vals[0])   # Webots 월드 절대 X
```

| 프레임/토픽 | 내용 |
|---|---|
| `{ns}/odom` → `{ns}/base_link` | 드라이버가 발행 (GPS + IMU) |
| `{ns}/map` → `{ns}/odom` | slam_toolbox가 발행 |
| `/{ns}/odom` | `nav_msgs/Odometry` (pose만 채운다 — twist는 비어 있다) |
| `/{ns}/joint_states` | 바퀴 4개의 연속 회전 각도 |

이 선택의 파장이 크다 — **각 로봇의 `{ns}/map` 원점이 사실상 Webots 월드 원점**이 되어,
맵 병합의 `world → {ns}/map`이 항등변환이 된다
([MAP_MERGE.md 2장](MAP_MERGE.md#2-정렬-설계--world-앵커-프레임)).
"map 프레임 원점 = 스폰 위치"로 오해해서 좌표가 정확히 두 배로 어긋난 버그가 실제로 났었다.

**시뮬레이션에서는 드리프트가 0이다.** 실제 로봇으로 옮기면 이 성질이 사라지므로
SLAM 파라미터와 맵 병합 앵커(`odom_is_world_absolute`)를 다시 잡아야 한다.

### 바퀴 관절 발행

바퀴 모터에서 `getPositionSensor()`로 위치 센서를 직접 얻어 `joint_states`를 발행한다.
이게 없으면 `robot_state_publisher`가 바퀴 링크의 TF를 못 만들고, **RViz의 RobotModel이
링크 하나 때문에 통째로 빨간 에러**가 된다
([MAP_MERGE.md 10장 ④](MAP_MERGE.md#-rviz의-robotmodel이-빨갛게-뜬다)).

값이 2000을 넘어가는 것은 연속 회전 관절이라 각도가 누적되기 때문이며, TF 계산은 각도를
그대로 쓰므로 문제없다.

---

## 4. 센서 → 스캔 → SLAM → Nav2 사슬

Spot이 뎁스카메라 5개를 합성해 만드는 스캔을, UGV는 Velodyne 하나로 얻는다.

```
/{ns}/Velodyne_VLP_16/point_cloud   (PointCloud2, 360°)
        ↓  pointcloud_to_laserscan
/{ns}/scan                          (LaserScan)
        ↓  slam_toolbox (async, mode: mapping)
/{ns}/map  +  {ns}/map → {ns}/odom TF
        ↓  nav2 (3초 지연 기동)
/{ns}/cmd_vel
```

`pointcloud_to_laserscan` 설정에서 의미 있는 값:

| 파라미터 | 값 | 이유 |
|---|---|---|
| `target_frame` | `{ns}/base_link` | 로봇 기준 스캔으로 변환 |
| `min_height` / `max_height` | 0.1 / 2.0 | 바닥(0.1 m 아래)을 장애물로 읽지 않게 자른다 |
| `range_min` / `range_max` | 0.2 / 50.0 | VLP-16 사양 |
| `angle_increment` | 0.0087 (≈0.5°) | 720점 스캔 |
| `transform_tolerance` | 5.0 | 시뮬이 실시간보다 느리게 돌 때 TF 대기 여유 |
| `use_sim_time` | **true** | 빠뜨리면 벽시계 스탬프 때문에 SLAM이 스캔을 전부 버린다 |

**정적 맵 파일은 없다.** `map_server`/`amcl`을 쓰지 않고 slam_toolbox가 `mode: mapping`으로
실시간 생성한 맵을 Nav2가 그대로 쓴다 ([mapper_params_online_async.yaml](src/webots_python/config/mapper_params_online_async.yaml)).

**Nav2를 3초 늦게 띄우는 이유** — `{ns}/map` 프레임이 생기기 전에 Nav2가 뜨면
`Invalid frame ID "ugv1/map"`을 무한 반복한다. 런치에서 `TimerAction`으로 미룬다.

---

## 5. Nav2 파라미터에서 실제로 중요한 값들

[navigation/param/nav2.yaml](src/Webots-SummitXL/workspace/navigation/param/nav2.yaml) (서브모듈).
네임스페이스는 `nav2.launch.py`의 `RewrittenYaml`이 주입한다.

| 항목 | 값 | 비고 |
|---|---|---|
| `footprint` | `[[-0.35,-0.25],[-0.35,0.25],[0.35,0.25],[0.35,-0.25]]` | 0.7 × 0.5 m. **드론도 이 파일을 공유한다** (Spot은 [nav2_spot.yaml](src/Webots-SummitXL/workspace/navigation/param/nav2_spot.yaml)로 분리) |
| `max_vel_x` / `max_vel_theta` | 0.5 / 0.3 | |
| `max_vel_y` | **0.0** | 메카넘이지만 Nav2는 옆걸음을 안 쓴다 |
| `acc_lim_x` / `acc_lim_theta` | 2.5 / 3.2 | |
| costmap `resolution` | 0.1 | slam_toolbox·맵 병합과 같은 해상도 |
| `inflation_radius` | 0.6 | |
| behavior_server `global_frame` / `robot_base_frame` | `{ns}/odom` / `{ns}/base_link` | **리커버리(spin/backup)가 쓰는 프레임.** 아래 ⚠️ 참고 |
| `xy_goal_tolerance` | 0.5 | 이 안에 들어오면 SUCCEEDED |

> `max_vel_y: 0.0`이라 **Nav2 자율주행 중에는 옆걸음이 안 나온다.** 메카넘의 이점을 쓰려면
> 이 값을 올리고 컨트롤러를 옴니 대응(예: DWB의 `vy_samples`)으로 바꿔야 한다.

> ⚠️ **behavior_server 의 프레임 파라미터 이름은 `global_frame` 이다.** `local_frame` 이
> 아니다 — Humble 의 `nav2_behaviors` 에 그런 파라미터는 없어서 yaml 에 써 봐야 조용히
> 무시되고, 정작 `global_frame` 은 yaml 에 없으니 RewrittenYaml 이 바꿀 대상도 없어서
> Nav2 기본값인 **네임스페이스 없는 `odom`** 이 그대로 남는다. 증상은 이렇다.
>
> ```
> [transformPoseInTargetFrame] target frame "odom" does not exist
> [behavior_server] Initial checks failed for spin  -> Aborting handle
> [bt_navigator] Goal failed
> ```
>
> **주행 자체는 멀쩡한데 리커버리만 죽는다**는 것이 헷갈리는 지점이다. 평소에는 목표에
> 잘 가다가, BT 가 리커버리를 한 번이라도 타는 순간(경로 재계산이 늦어지는 등) 목표가
> 통째로 ABORT 된다. 실측으로 확인했다 — 수정 전 4 m 목표가 1.43 m 를 남기고 ABORT,
> 수정 후 같은 조건에서 SUCCEEDED.
>
> `global_frame` 치환값은 `{ns}/map` 이라 behavior_server 에는 맞지 않으므로,
> `nav2.launch.py` 가 그 노드에서만 `{ns}/odom` 으로 덮어쓴다 (local_costmap 과 같은 예외).

목표점을 주는 방법:

```bash
ros2 topic pub -1 /ugv1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'ugv1/map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"
```

`frame_id`는 **반드시 `{ns}/map`** 이어야 한다. 웹에서 클릭으로 보낼 때도 같다
([Readme 8-1](Readme.md#8-1-맵목표점-데이터-규격-웹-개발자용)).

---

## 6. 키보드 조종

두 갈래가 있다. 하나는 Webots 서브모듈의 `simulator` 패키지, 하나는 이 저장소의
`webots_python` 패키지다.

```bash
docker exec -it ugv1_brain_windows bash -c \
  "source /ros2_ws/install/setup.bash && ros2 run simulator keyboard --ros-args -r __ns:=/ugv1"
```

```
Q : 좌회전    W : 전진     E : 우회전
A : 좌 평행이동  S : 후진   D : 우 평행이동
Space : 정지          = / - : 속도 증감
```

`webots_python`의 `summit_telop`도 같은 역할이다(`ros2 run webots_python summit_telop`).
드론용은 축이 하나 더 있는 별도 텔레옵을 쓴다 ([Readme 11-2](Readme.md#11-2-cmd_vel-사용법-ugv와-다름-주의)).

---

## 7. 알아 둘 함정

**① `/clock`을 발행하는 것은 `ugv1` 하나다.**
`robot_driver.py`는 네임스페이스가 `ugv1`(또는 빈 문자열)일 때만 자기를 "시계 마스터"로
정하고 `/clock`을 발행한다. Spot·드론 드라이버에는 이 코드가 없다.

```python
if self.namespace == 'ugv1' or self.namespace == '':
    self.clock_publisher = self.__node.create_publisher(Clock, '/clock', 10)
```

편대에서 `ugv1`을 빼거나 `ugv1` 컨테이너만 죽이면, **`use_sim_time`을 쓰는 모든 노드
(SLAM·Nav2·맵 병합)가 시각을 못 받아 조용히 멈출 수 있다.** 무언가 전부 멈춘 것 같으면
`ros2 topic hz /clock`을 **가장 먼저** 확인한다. 0 Hz면 시뮬이 멈췄거나 시계 발행자가
없는 것이다.

대안으로 [sim_clock_bridge](src/webots_python/webots_python/sim_clock_bridge.py)가 있다 —
어떤 로봇의 `odom` 헤더 시각을 그대로 `/clock`으로 중계하는 노드다. 데이터 수집 런치에서
`use_clock_bridge:=true`로 쓴다.

**② 시뮬레이션 Play(▶) 상태 확인이 항상 1순위.**
일시정지 상태면 `step()`이 호출되지 않아 TF/odom/스캔이 전혀 발행되지 않는다. 증상만
보면 코드가 고장 난 것처럼 보인다.

**③ 새 센서 처리 노드에는 `use_sim_time: True`를 반드시 넣는다.**
빠뜨리면 벽시계 스탬프가 찍혀 tf2가 "아득한 미래의 데이터"로 취급하고, 이후 정상
데이터까지 `TF_OLD_DATA`로 거부한다
([MAP_MERGE.md 10장 ⑤](MAP_MERGE.md#-로봇-노드는-반드시-use_sim_time-true로-띄울-것)).

**④ 드라이버의 `synchronization`은 몸의 필드와 값이 같아야 한다.**
소환된 로봇은 `synchronization FALSE`로 주입되므로 소환기가 `ROBOT_SYNCHRONIZATION=false`를
넣어 준다 ([SPAWNER.md 4장](SPAWNER.md#4-소환-한-번에-일어나는-일)).

**⑤ `odom` 토픽의 twist는 비어 있다.** 드라이버가 pose만 채운다. 속도가 필요하면
`cmd_vel`을 쓰거나 pose를 미분해야 한다.

---

## 8. 알려진 한계

- **옆걸음이 Nav2 경로에 안 쓰인다** (`max_vel_y: 0.0`). 메카넘의 이점을 자율주행에서
  살리려면 컨트롤러 튜닝이 필요하다
- **`odom` 드리프트가 0**이라 SLAM 파라미터가 실기 기준으로 검증되지 않았다.
  실제 로봇으로 옮길 때 가장 먼저 깨질 가정이다
- **footprint를 세 로봇이 공유한다.** Spot(다리 벌림)과 드론(비행)에는 맞지 않는다
- **rgb_camera는 데이터 수집 경로에서만 쓴다.** 주행·SLAM은 라이다만 본다
  ([DATA_COLLECTION.md](DATA_COLLECTION.md))
- **`explore_lite` 자율 탐사는 미착수.** 서브모듈에 코드는 들어와 있다
  (`src/Webots-SummitXL/workspace/explore/`)

---

## 9. 파일 맵

| 파일 | 역할 |
|---|---|
| `src/Webots-SummitXL/workspace/simulator/protos/SummitXlSteelSensorized.proto` | 센서를 품은 래퍼 PROTO (소환용) |
| `.../simulator/simulator/robot_driver.py` | 메카넘 + GPS odom + 바퀴 관절 + `/clock` |
| `.../simulator/simulator/keyboard.py` | 키보드 텔레옵 |
| `.../navigation/param/nav2.yaml` | Nav2 파라미터 (Spot·드론도 공유) |
| `.../navigation/launch/nav2.launch.py` | 네임스페이스 주입 |
| [src/webots_python/urdf/SummitXlSteel.urdf.xacro](src/webots_python/urdf/SummitXlSteel.urdf.xacro) | 플러그인 연결 + 디바이스 매핑 |
| [src/webots_python/launch/single_ugv.launch.py](src/webots_python/launch/single_ugv.launch.py) | 뇌 전체 (드라이버·스캔·SLAM·Nav2·중계·등록) |
| [src/webots_python/config/mapper_params_online_async.yaml](src/webots_python/config/mapper_params_online_async.yaml) | slam_toolbox 설정 |
| [src/webots_python/webots_python/summit_telop.py](src/webots_python/webots_python/summit_telop.py) | 텔레옵 (이 저장소 쪽) |
| [src/webots_python/webots_python/sim_clock_bridge.py](src/webots_python/webots_python/sim_clock_bridge.py) | odom 시각 → `/clock` 중계 |

### 관련 문서

- [Readme.md](Readme.md) — 실행 방법, 좌표계 규약
- [INTERFACES.md](INTERFACES.md) — 세 로봇의 토픽·서비스 총람
- [MAP_MERGE.md](MAP_MERGE.md) — UGV 맵이 전역 맵에 합류하는 경로
- [spot_driver_functions.md](spot_driver_functions.md) / [drone_setup.md](drone_setup.md) — 다른 두 로봇
