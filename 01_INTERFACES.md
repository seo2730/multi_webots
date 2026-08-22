# 01. 인터페이스 총람 (Topics · Services · Frames)

> 📖 [책 목차](Readme.md#-목차) · ← [00. 빠른 시작](00_QUICKSTART.md) · [02. 월드 생성](02_WORLD_GEN.md) →

"이 시스템에 무엇을 보내면 무엇이 나오는가"를 한 장에 모은 색인.
새로 합류한 사람과 **웹/외부 연동 개발자**가 제일 먼저 볼 문서다.

각 항목의 **왜**와 **어떻게**는 담당 문서에 있고, 여기서는 규격만 다룬다.
`{ns}`는 로봇 이름(= ROS 2 네임스페이스, 예: `ugv1`, `spot1`, `drone1`, `ugv3`)이다.

## 목차
- [1. 로봇에게 명령하기](#1-로봇에게-명령하기)
- [2. 로봇에서 나오는 것](#2-로봇에서-나오는-것)
- [3. 전역(관제) 인터페이스](#3-전역관제-인터페이스)
- [4. 서비스](#4-서비스)
- [5. 프레임(TF) 트리](#5-프레임tf-트리)
- [6. QoS 주의 목록](#6-qos-주의-목록)
- [7. 환경 변수](#7-환경-변수)
- [8. 컨테이너와 설정 파일 색인](#8-컨테이너와-설정-파일-색인)

---

## 1. 로봇에게 명령하기

### `cmd_vel` — 타입은 같고 의미는 다르다 🚨

`/{ns}/cmd_vel` (`geometry_msgs/msg/Twist`)는 세 로봇 모두 받지만 **해석이 전혀 다르다.**
같은 값을 그대로 옮기면 안 된다.

| 필드 | UGV (메카넘) | Spot (사족보행) | Drone (쿼드콥터) |
|---|---|---|---|
| `linear.x` | 전후 속도 (m/s) | 전후 속도 (m/s) | 전후 속도 (m/s) |
| `linear.y` | 좌우 **평행이동** (m/s) | 게걸음 방향 비율 | 좌우 속도 (기체가 **기울어서** 이동) |
| `linear.z` | — | — | **상승 속도** (m/s, 목표 고도를 적분) |
| `angular.z` | 선회 각속도 (rad/s) | 제자리 회전 속도 | 선회 각속도 (rad/s) |
| 권장 범위 | Nav2 기준 ±0.5 | **0.045~0.195 m/s, ±0.247 rad/s** (그 위는 클램프, 그 아래는 하한으로 올라감) | ±1~2 |
| 자세히 | [04장 2절](04_UGV_SETUP.md#2-cmd_vel--메카넘-역기구학) | [06장](06_SPOT_DRIVER.md#cmd_vel-단위) · [07장](07_SPOT_NAV.md) | [08장 3절](08_DRONE_SETUP.md#3-인터페이스-규격) |

> 🔄 **2026-08-16 변경.** Spot 의 `linear.x`/`angular.z` 는 예전에 **보폭 배율**이었다
> (`StepLength = 0.15 * linear.x`). 지금은 **m/s · rad/s** 다. 옛 값을 그대로 쓰면
> 약 1.3 배 빨라진다 → [06_SPOT_DRIVER.md](06_SPOT_DRIVER.md#cmd_vel-단위)
>
> ⚠️ Spot 은 **최저 속도 아래를 못 낸다** (보폭 하한 — 현재 운용점에서 약 0.045 m/s).
> 그보다 작은 명령도 그 속도로 나간다. 위 값들은 **시뮬 시각(`odom.header.stamp`) 기준
> 실측 확정치**이고, 케이던스(`swing_period`/`step_velocity`)를 바꾸면 함께 움직인다
> → [07_SPOT_NAV.md](07_SPOT_NAV.md)
>
> 🚨 **Nav2 운용점은 0.15 m/s 다.** 최고속 0.195 는 직진에서 잰 값이라 그대로 쓰면
> 회전 여유가 0 이 되어 간헐적으로 넘어진다.

### 목표점 (자율주행)

| 토픽 | 타입 | 비고 |
|---|---|---|
| `/{ns}/goal_pose` | `geometry_msgs/msg/PoseStamped` | Nav2 표준 입력. **`frame_id`는 `{ns}/map`** |
| `/{ns}/goal_pose_3d` | `geometry_msgs/msg/PoseStamped` | **드론 전용.** 층을 골라 고도를 맞춘 뒤 `goal_pose`로 넘긴다 → [09_DRONE_NAV.md](09_DRONE_NAV.md) |
| `/{ns}/cruise_altitude` | `std_msgs/msg/Float64` | **드론 전용.** 순항 고도 지시 |
| `/web/goal_point` | `geometry_msgs/msg/PointStamped` | 웹 클릭용. `frame_id`가 정확히 `{ns}/map`인 것만 그 로봇의 `goal_pose`로 중계된다 (다른 값은 무시) |

```bash
ros2 topic pub -1 /ugv1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'ugv1/map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"
```

> **좌표 값은 로봇끼리 호환되지만 프레임 이름은 아니다.** 이 프로젝트에서는 모든
> `{ns}/map` 원점이 사실상 Webots 월드 원점이라 숫자는 그대로 통하지만, `frame_id`는
> 로봇마다 달라서 정확히 채워야 한다 ([Readme 7절](Readme.md#7-로봇-위치-및-맵-데이터)).

### 전문가용 입력

| 토픽 | 로봇 | 내용 |
|---|---|---|
| `/{ns}/inverse_gait_input` | Spot | 보행 파라미터 15개를 직접 지정. 발행자가 있으면 `cmd_vel`이 무시된다 |

---

## 2. 로봇에서 나오는 것

| 토픽 | 타입 | ugv | spot | drone |
|---|---|:---:|:---:|:---:|
| `/{ns}/odom` | `nav_msgs/msg/Odometry` | ✅ (twist 비어 있음) | ✅ | ✅ (z·roll·pitch 포함) |
| `/{ns}/joint_states` | `sensor_msgs/msg/JointState` | ✅ 바퀴 4 | ✅ 관절 12 + 4 | — (움직이는 관절 없음) |
| `/{ns}/scan` | `sensor_msgs/msg/LaserScan` | ✅ Velodyne 변환 | ✅ 뎁스카메라 5개 병합 | — (층별 매퍼가 직접 격자를 만든다) |
| `/{ns}/map` | `nav_msgs/msg/OccupancyGrid` | ✅ | ✅ | ✅ **층 합집합** (병합기용) |
| `/{ns}/robot_description` | `std_msgs/msg/String` | ✅ | ✅ | ✅ |
| `/{ns}/Velodyne_VLP_16/point_cloud` | `sensor_msgs/msg/PointCloud2` | ✅ | — | ✅ |
| `/{ns}/down_depth/point_cloud` | `sensor_msgs/msg/PointCloud2` | — | — | ✅ 하향 뎁스 |
| `/{ns}/map_active` | `nav_msgs/msg/OccupancyGrid` | — | — | ✅ 현재 순항 고도 (Nav2가 본다) |
| `/{ns}/map_layer_{k}` | `nav_msgs/msg/OccupancyGrid` | — | — | ✅ 후보 층 |
| `/{ns}/altitude_status` | `std_msgs/msg/String` | — | — | ✅ 층 선택 근거 |
| `/{ns}/avoid_status` | `std_msgs/msg/String` | — | — | ✅ 지역 회피 로그 |
| `/{ns}/cmd_vel_nav` | `geometry_msgs/msg/Twist` | — | — | ✅ Nav2 출력 (회피기가 받는다) |
| `/{ns}/rgb_camera/image_color` | `sensor_msgs/msg/Image` | ✅ | — | — |
| `/{ns}/camera/image_color` | `sensor_msgs/msg/Image` | — | — | ✅ 짐벌 카메라 |
| `/{ns}/*_depth/*` | 뎁스 카메라 5종 | — | ✅ | — |

센서 토픽은 대부분 URDF의 `<device>` 매핑을 보고 **`webots_ros2_driver`가 자동으로**
만든다. 드라이버 플러그인이 직접 발행하는 것은 `odom` / `joint_states` / TF 정도다.

---

## 3. 전역(관제) 인터페이스

| 토픽 | 타입 | 발행 주체 | 내용 |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | **`ugv1` 드라이버** | 시뮬레이션 시각. ⚠️ [04장 7절](04_UGV_SETUP.md#7-알아-둘-함정) |
| `/map_merged` | `nav_msgs/msg/OccupancyGrid` | master (`map_merger`) | 전역 병합 맵, frame = `world` |
| `/robot_markers` | `visualization_msgs/msg/MarkerArray` | master | 로봇별 화살표 + 이름표 |
| `/robot_registry` | `std_msgs/msg/String` (JSON) | 로봇마다 (`robot_registrar`) | 명함 + 1 Hz 하트비트 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 전원 | 네임스페이스 없이 공유 |

`/robot_registry` 메시지:

```json
{"robot_id": "ugv3", "init_x": -2.0, "init_y": 4.5, "init_yaw": 1.57,
 "has_map": true, "map_topic": "/ugv3/map"}
```

> `/map_merged`는 **관제·시각화·웹 전용**이다. 각 로봇 Nav2에 되먹이면 프레임 순환과
> 코스트맵 진동이 생긴다 ([10장 10절 ⑩](10_MAP_MERGE.md#-병합-맵을-nav2에-되먹이지-말-것)).

---

## 4. 서비스

### 로봇 소환

| 서비스 | 타입 | 비고 |
|---|---|---|
| `/spawn_robot` | `webots_spawner_msgs/srv/SpawnRobot` | `type`: `ugv`/`spot`/`drone`, `robot_id`, `random`, `x`/`y`/`yaw`, `min_clearance`, `force` |

```bash
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'ugv', random: true}"
```

필드별 의미와 실패 사유는 [03장 9절](03_SPAWNER.md#9-파라미터-표).

### Spot 자세 제어

| 서비스 | 타입 | 기능 |
|---|---|---|
| `/{ns}/stand_up` | `webots_spot_msgs/srv/SpotMotion` | 일어서기 |
| `/{ns}/sit_down` | `webots_spot_msgs/srv/SpotMotion` | 앉기 |
| `/{ns}/lie_down` | `webots_spot_msgs/srv/SpotMotion` | 눕기 |
| `/{ns}/shake_hand` | `webots_spot_msgs/srv/SpotMotion` | 악수 |
| `/{ns}/set_height` | `webots_spot_msgs/srv/SpotHeight` | 몸높이 ±0.2 m |
| `/{ns}/float_mode` | `std_srvs/srv/SetBool` | 제자리 호버링 (거리센서 4개 필요) |

```bash
ros2 service call /spot1/stand_up webots_spot_msgs/srv/SpotMotion "{override: true}"
```

`{override: true}`가 없으면 이전 모션 재생 중에는 거부된다.
`blocksworld_pose`는 MASKOR 원본의 잔재이므로 호출하지 않는다
([06_SPOT_DRIVER.md](06_SPOT_DRIVER.md)).

Nav2 액션(`/{ns}/navigate_to_pose` 등)은 표준 그대로다. 파라미터는 로봇마다 다르다.

| 로봇 | 파라미터 파일 |
|---|---|
| ugv · drone | [nav2.yaml](src/Webots-SummitXL/workspace/navigation/param/nav2.yaml) |
| **spot** | [nav2_spot.yaml](src/Webots-SummitXL/workspace/navigation/param/nav2_spot.yaml) — 속도·가속·footprint 가 다르다. 근거는 그 파일 머리말 |

> ⚠️ 드론도 여기에 포함되지만, **플래너는 2D 그대로다.** 목표의 `position.z`는
> 무시된다. 다만 드론에는 그 위에 층 선택기가 얹혀 있어서, `/{ns}/goal_pose_3d`
> 로 주면 **어느 고도로 갈지 골라 준 뒤** Nav2 에 넘긴다 (연속 3D 경로는 아니고
> 이산 층 선택이다) → [08장 7절](08_DRONE_SETUP.md).

---

## 5. 프레임(TF) 트리

```
world                       ← 공통 기준 (map_merger가 static TF로 못 박는다)
├── ugv1/map                ← slam_toolbox
│   └── ugv1/odom           ← 드라이버 (GPS 절대좌표)
│       └── ugv1/base_link
│           ├── ugv1/Velodyne_VLP_16
│           └── ugv1/rgb_camera ...
├── spot1/map → spot1/odom → spot1/base_link → base_footprint, 다리 링크들
└── drone1/map              ← **static TF (항등)**. 드론은 slam_toolbox 를 안 쓴다
    └── drone1/odom         ← 드라이버 (GPS 절대좌표)
        └── drone1/base_link
            ├── drone1/Velodyne_VLP_16
            ├── drone1/down_depth
            └── drone1/camera, gps, gyro, ...
```

> 드론의 `map → odom` 이 static 인 이유: odom 이 이미 월드 절대좌표라 보정할 드리프트가
> 없다. `map_merger` 가 `world → {ns}/map` 을 항등으로 두는 것과 같은 근거다
> (`odom_is_world_absolute`). → [08장 7절](08_DRONE_SETUP.md)

> 맵이 없는 로봇(`has_map: false`)은 `{ns}/map` 대신 `{ns}/odom`을 `world`에 매단다.
> 드론이 라이다를 달기 전까지 그랬다 → [10장 10절 ⑨](10_MAP_MERGE.md).

| 프레임 | 부모 | 발행 주체 |
|---|---|---|
| `world` | (최상위) | — |
| `{ns}/map` | `world` | `map_merger` (static) |
| `{ns}/odom` | `{ns}/map`, 맵 없으면 `world` | slam_toolbox / `map_merger` |
| `{ns}/base_link` | `{ns}/odom` | 로봇 드라이버 |
| 그 아래 링크들 | `{ns}/base_link` | `robot_state_publisher` (`frame_prefix={ns}/`) |

---

## 6. QoS 주의 목록

**QoS가 안 맞으면 에러도 경고도 없이 아무것도 안 들어온다.** 직접 구독해 디버깅할 때
반드시 맞춰야 하는 토픽:

| 토픽 | QoS |
|---|---|
| `/map_merged` | Reliable + **Transient Local** + depth 1 |
| `/{ns}/map` | Reliable + **Transient Local** + depth 1 (slam_toolbox 기본) |
| `/robot_registry` | Reliable + **Transient Local** + depth 20 |
| `/{ns}/robot_description` | Reliable + **Transient Local** |

```bash
ros2 topic echo /ugv1/map --qos-durability transient_local --qos-reliability reliable --field info
```

> **`ros2 topic hz`를 믿지 말 것.** 노드가 100개를 넘으면 있는 토픽도 "does not appear to
> be published yet"으로 나온다. rclpy로 직접 구독해 세는 쪽이 정확하다
> ([10장 10절 ②-2](10_MAP_MERGE.md#-2-ros2-topic-hz가-거짓말을-할-때)).

---

## 7. 환경 변수

| 변수 | 쓰는 곳 | 의미 |
|---|---|---|
| `ROBOT_ID` | 모든 로봇 런치 | 네임스페이스. 기본값은 런치마다 다름(`ugv1`/`spot1`) |
| `ROBOT_INIT_X` / `_Y` / `_YAW` | `robot_registrar` | 스폰 좌표(맵 병합 명함용). 현재 기본 설정에서는 값이 쓰이지 않는다 |
| `ROBOT_SYNCHRONIZATION` | 드라이버 | 몸의 `synchronization` 필드와 **같아야** 한다. 소환기가 넣어 준다 |
| `ROBOT_DEF` | Spot 드라이버 | 씬 트리의 DEF 이름 (`spot2` → `SPOT2`) |
| **`NAV_MODE`** | **드론 런치** | 경로계획 모드 — `2d` / `2.5d_local` / `2.5d`(기본). 모르는 값이나 `3d`를 주면 런치가 **에러를 내고 멈춘다** → [09장 0절](09_DRONE_NAV.md#0-경로계획-모드-고르기) |
| `WEBOTS_HOST` / `WEBOTS_PORT` | 드라이버 | 기본 `host.docker.internal:1234` |
| `ROS_DOMAIN_ID` | 전부 | **30** |
| `RMW_IMPLEMENTATION` | 전부 | `rmw_fastrtps_cpp` |
| `ROS_LOCALHOST_ONLY` | 전부 | `0` |

호스트 셸에서 `ros2 topic list`로 들여다보려면 위 세 개를 그대로 맞춰야 한다.

---

## 8. 컨테이너와 설정 파일 색인

| 컨테이너 | 무엇이 도나 |
|---|---|
| `{ns}_brain_{os}` (예: `ugv1_brain_windows`) | 로봇 하나의 뇌 (드라이버 · SLAM · Nav2 · 등록) |
| `rviz_master_{os}` | `map_merger` · `joint_state_filler` · `robot_marker_publisher` · RViz2 |
| `fleet_spawner_{os}` | `spawn_supervisor` (+ 런타임 소환 로봇의 뇌) |

| 설정 파일 | 다루는 것 |
|---|---|
| [webots_map_merge/config/robots.yaml](src/webots_map_merge/config/robots.yaml) | 병합 파라미터 → [10장 7절](10_MAP_MERGE.md#7-인터페이스-규격) |
| [webots_robot_spawner/config/spawner.yaml](src/webots_robot_spawner/config/spawner.yaml) | 소환 파라미터 → [03장 9절](03_SPAWNER.md#9-파라미터-표) |
| [webots_robot_spawner/config/fleet/](src/webots_robot_spawner/config/fleet/) | 편대 매니페스트 |
| [webots_robot_spawner/config/doorways/](src/webots_robot_spawner/config/doorways/) | 생성 월드의 방·출입구 좌표 → [02_WORLD_GEN.md 3-3](02_WORLD_GEN.md#3-3-출입구-yaml) |
| [webots_python/config/mapper_params_online_async.yaml](src/webots_python/config/mapper_params_online_async.yaml) | slam_toolbox |
| `src/Webots-SummitXL/workspace/navigation/param/nav2.yaml` | Nav2 (세 로봇 공유) |
| `docker-configs/*/docker-compose.yml` | 서비스 구성 (매니페스트에서 생성) |

### 관련 문서

- [Readme.md](Readme.md) — 설치·실행·전체 그림
- [03_SPAWNER.md](03_SPAWNER.md) · [10_MAP_MERGE.md](10_MAP_MERGE.md) · [02_WORLD_GEN.md](02_WORLD_GEN.md)
- [04_UGV_SETUP.md](04_UGV_SETUP.md) · [06_SPOT_DRIVER.md](06_SPOT_DRIVER.md) · [08_DRONE_SETUP.md](08_DRONE_SETUP.md)

---

← [00. 빠른 시작](00_QUICKSTART.md) | [📖 책 목차](Readme.md#-목차) | [02. 월드 생성](02_WORLD_GEN.md) →
