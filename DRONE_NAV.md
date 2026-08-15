# 드론 자율비행 — 구조와 직접 테스트하는 법

드론이 목표점까지 스스로 날아가고, 앞이 막히면 **고도를 올려 넘어가는** 구조의 설명서.
"무엇을 만들었나 / 왜 그렇게 만들었나 / 어떻게 직접 돌려보나"를 한 곳에 모았다.

기체 자체(개조·제어·센서)는 [drone_setup.md](drone_setup.md), 빠른 사용법은
[Readme 11장](Readme.md#11-drone-중형급-쿼드콥터)에 있다.

## 목차
- [0. 경로계획 모드 고르기](#0-경로계획-모드-고르기)
- [1. 한눈에 보기](#1-한눈에-보기)
- [2. 직접 테스트하기](#2-직접-테스트하기)
- [3. 왜 이 구조인가](#3-왜-이-구조인가)
- [4. 노드별 역할](#4-노드별-역할)
- [5. 파라미터 손잡이](#5-파라미터-손잡이)
- [6. 실측 결과](#6-실측-결과)
- [7. 겪은 함정 모음](#7-겪은-함정-모음)
- [8. 알려진 한계](#8-알려진-한계)

---

## 0. 경로계획 모드 고르기

고도를 다루는 축이 **둘**이고, 모드는 그 조합이다.

| `NAV_MODE` | 전역 층 선택 | 지역 고도 회피 | 동작 | 언제 쓰나 |
|---|:---:|:---:|---|---|
| `2d` | ✗ | ✗ | 고정 고도, Nav2만 | 지상 로봇과 똑같이 비교하고 싶을 때 |
| `2.5d_local` | ✗ | ✅ | 고도는 고정하되 **앞이 막히면 넘어감** | 순항 고도를 내가 정하고 싶을 때 |
| `2.5d` (기본) | ✅ | ✅ | 목표마다 층 선택 + 지역 회피 | 평소 |
| `3d` | — | — | **미구현. 런치가 거부한다** | — |

```bash
# compose 로 (기본값은 생성된 compose 의 NAV_MODE=2.5d)
#   docker-configs/{os}/docker-compose.yml 의 drone1 서비스에서 NAV_MODE 를 바꾼다

# 손으로 띄워 볼 때
NAV_MODE=2d ROBOT_ID=drone1 ros2 launch webots_python single_drone.launch.py
```

**모드와 무관하게 항상 켜져 있는 것**이 둘 있다. 경로계획이 아니라 안전장치이기 때문이다.

- **발밑 안전 바닥** (`ground_clearance`) — 하향 센서가 잰 표면 위로 반드시 여유를 남긴다
- **`cmd_vel` 단독 소유** — `local_altitude_avoider`가 항상 돌면서 순항 고도를 잡는다.
  `2d`에서는 회피 판단만 꺼진다.

> ⚠️ `2d` / `2.5d_local`에서는 `altitude_selector`가 **안 뜬다.** 그 모드에서
> `/{ns}/goal_pose_3d`로 목표를 주면 아무도 받지 않으므로, `/{ns}/goal_pose`를 써야 한다.

### `3d`를 왜 안 만들었나

연속 3D 경로계획은 **계획 호출마다** 3D 탐색을 해서 군집에서 비용이 대수만큼 곱해진다.
이 프로젝트는 그 비용을 피하는 쪽을 골랐다 — 자세한 근거와 만들려면 무엇이 필요한지는
[3장](#3-왜-이-구조인가)에 있다.

`NAV_MODE=3d`로 띄우면 런치가 **에러를 내고 멈춘다.** 이름만 받아 주고 조용히 `2.5d`처럼
돌면 나중에 "3d로 돌렸는데 왜 고도가 계획 안 되지"로 헤매기 때문이다.

---

## 1. 한눈에 보기

```
Velodyne(수평) ─┐
                ├─▶ drone_layer_mapper ─┬─▶ /drone1/map          층 합집합 ─▶ 맵 병합기
down_depth(하향)┘                        ├─▶ /drone1/map_active  현재 층   ─▶ Nav2
                                         └─▶ /drone1/map_layer_k          ─▶ altitude_selector
                                                                                 │
  /drone1/goal_pose_3d ─▶ [altitude_selector] ─▶ 순항 고도 ─┐                    │
                                                            ▼                    │
                          Nav2 ─▶ /drone1/cmd_vel_nav ─▶ [local_altitude_avoider]
                                                            │
                                                            ▼
                                                   /drone1/cmd_vel ─▶ 드라이버
```

**고도를 다루는 두 층이 있다.** 둘의 역할이 다르다.

| | `altitude_selector` (전역) | `local_altitude_avoider` (지역) |
|---|---|---|
| 언제 | 목표를 받을 때 **1회** | 주행 중 **0.1초마다** |
| 근거 | 층 지도의 회랑 검사 | 라이다 실시간 룩어헤드 3 m |
| 하는 일 | 순항 고도를 고른다 | 순항 고도를 기준으로 **넘고 되돌아온다** |
| 출력 | `/drone1/cruise_altitude` | `/drone1/cmd_vel` (단독 소유) |

수평은 전부 Nav2가 하고, 이 둘은 **`linear.z`만** 건드린다.

---

## 2. 직접 테스트하기

### 준비

Webots에서 월드를 열고 ▶(Play)를 눌러 둔 뒤:

```bash
docker compose -f docker-configs/windows/docker-compose.yml up -d
```

드론 노드가 다 떴는지 확인 (`avoid_status`가 보이면 준비 완료):

```bash
docker exec drone1_brain_windows bash -c \
  "source /ros2_ws/install/setup.bash && ros2 topic list | grep drone1"
```

> ⚠️ **가구를 옮겼거나 드론을 손으로 움직였으면 컨테이너를 다시 띄운다.**
> 지도는 망각 기능이 있어 시간이 지나면 스스로 갱신되지만, 즉시 깨끗하게 시작하려면
> `docker compose ... up -d --no-deps --force-recreate drone1` 이 확실하다.

### (a) 고도 고정으로 목표점 주기 — 가장 단순

Nav2에 직접 준다. 고도는 순항 고도에 머문다 (지역 회피는 계속 동작한다).

```bash
docker exec drone1_brain_windows bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic pub -1 /drone1/goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: \"drone1/map\"}, pose: {position: {x: 4.0, y: 2.0}, orientation: {w: 1.0}}}'"
```

### (b) 층을 골라서 가기 — 전역 고도 선택

목표까지의 회랑을 층마다 검사해 **가장 낮은 뚫린 층**으로 이동한 뒤 출발한다.

```bash
docker exec drone1_brain_windows bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic pub -1 /drone1/goal_pose_3d geometry_msgs/msg/PoseStamped \
'{header: {frame_id: \"drone1/map\"}, pose: {position: {x: -5.0, y: -2.0}, orientation: {w: 1.0}}}'"
```

판단 근거는 로그로 나온다:

```bash
docker exec drone1_brain_windows bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic echo /drone1/altitude_status"
```

```
층 선택: 3.0 m — 기준 통과 (현재 2.00 m)
  [1m:X(장애물22.3%/미탐색29%) | 2m:X(3.2%/42%) | 3m:OK(0.0%/26%)]
```

### (c) 순항 고도 바꾸기

```bash
docker exec drone1_brain_windows bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic pub -1 /drone1/cruise_altitude std_msgs/msg/Float64 '{data: 1.5}'"
```

> `goal_pose_3d`를 쓰면 층 선택기가 이 값을 덮어쓴다. 고도를 고정하고 싶으면
> `goal_pose`(a번)를 쓴다.

### (d) 지역 회피 관찰 — 가장 볼만한 것

**Webots에서 드론 앞에 상자를 하나 놓고** 목표를 반대편에 준 뒤 이 토픽을 본다.

```bash
docker exec drone1_brain_windows bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic echo /drone1/avoid_status"
```

```
전방 2.7 m 에 높이 +0.28 m 장애물 → 3.50 m 로 회피 (필요한 만큼만)
장애물 지나감 → 순항 2.0 m 로 복귀
발밑 1.50 m 에 무언가 있음 → 1.95 m 아래로는 안 내려감
```

세 줄이 각각 다른 기능이다 — 순서대로 **상승 회피 / 복귀 / 착지 방지**.

### (e) 층별 지도 보기

RViz에서 `/drone1/map_layer_0`, `_1`, `_2`를 각각 Map 디스플레이로 추가하면
같은 장소가 고도별로 어떻게 다른지 보인다. QoS는 **Transient Local + Reliable**이어야 한다.

```bash
# 칸 수로 빠르게 비교
docker exec drone1_brain_windows bash -c "source /ros2_ws/install/setup.bash && python3 - <<'EOF'
import time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n=Node('layers'); S={}
q=QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
             reliability=QoSReliabilityPolicy.RELIABLE)
for name in ['map','map_active','map_layer_0','map_layer_1','map_layer_2']:
    n.create_subscription(OccupancyGrid, f'/drone1/{name}',
                          lambda m,k=name: S.__setitem__(k,m), q)
t0=time.time()
while time.time()-t0 < 20: rclpy.spin_once(n, timeout_sec=0.1)
for k,v in S.items():
    d=np.asarray(v.data, dtype=np.int8)
    print(f'{k:<12} 장애물 {int((d>=50).sum()):5d}  빈곳 {int((d==0).sum()):6d}  미탐색 {int((d==-1).sum()):6d}')
EOF"
```

### (g) 모드 비교 — `2d`는 막히고 `2.5d_local`은 넘어가는 것 보기

**아직 실증되지 않은 항목이다.** 모드가 서로 다르게 뜨는 것(노드 구성·고도 고정·회피 0건)은
확인했지만, "같은 상황에서 결과가 갈리는" 사례는 아래 방법으로 직접 만들어야 한다.

앞선 시도가 실패한 이유가 방법 설계에 그대로 반영돼 있다 — 드론 스폰 지점
`(-6.5, 5.5)` 구석에서 임의의 목표 두 개를 줬더니 **두 모드 결과가 똑같았다**
(오차 11.48 vs 11.49). 그 경로에 벽만 있고 **넘을 수 있는 장애물이 없었기** 때문이다.

#### 준비 — 넘을 수 있는 장애물을 만든다

Webots에서 드론 진행 방향에 상자를 놓는다. **높이가 핵심이다.**

| 조건 | 값 | 이유 |
|---|---|---|
| 상자 높이 | **순항 고도 ± 0.2 m** (순항 2.0 m면 1.9~2.1 m) | 기체가 실제로 부딪히는 대역(±0.25 m)에 걸려야 회피가 발동한다 |
| 상자 폭 | 2~3 m | 좁으면 Nav2가 그냥 옆으로 돌아가 버려 고도 회피가 안 쓰인다 |
| 위쪽 | **비워 둘 것** | 천장이 막혀 있으면 어느 모드로도 못 넘는다 |
| 위치 | 드론에서 4~6 m 앞 | 룩어헤드 3 m 밖에서 출발해야 접근 과정이 보인다 |

> 상자가 너무 낮으면(윗면이 −0.3 m 이하) 회피 자체가 안 뜬다 — 닿지 않으니 정상이다.
> 너무 높으면(꼭대기가 안 보이면) 층 단위로 물러난다. 둘 다 [7장 ③④](#7-겪은-함정-모음) 참고.

#### 실행

목표는 **상자 반대편**에 준다. 같은 목표를 두 모드로 돌려 비교한다.

```bash
# 1) 2d 로 — 못 넘어야 한다
sed -i 's/NAV_MODE=.*/NAV_MODE=2d/' docker-configs/windows/docker-compose.yml
docker compose -f docker-configs/windows/docker-compose.yml up -d --no-deps --force-recreate drone1

# 목표를 주고 결과·고도를 본다 (좌표는 상자 반대편으로 바꿀 것)
docker exec drone1_brain_windows bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic pub -1 /drone1/goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: \"drone1/map\"}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}}'"

docker exec drone1_brain_windows bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic echo /drone1/odom --field pose.pose.position"

# 2) 2.5d_local 로 — 넘어가야 한다
sed -i 's/NAV_MODE=.*/NAV_MODE=2.5d_local/' docker-configs/windows/docker-compose.yml
docker compose -f docker-configs/windows/docker-compose.yml up -d --no-deps --force-recreate drone1
# (같은 목표를 다시 준 뒤 avoid_status 를 함께 본다)
```

#### 무엇을 보면 성공인가

| | `2d` | `2.5d_local` |
|---|---|---|
| `avoid_status` | 아무것도 안 뜸 | `전방 … 장애물 → … 로 회피` → `장애물 지나감 → 복귀` |
| 고도 | 순항에 고정 (폭 < 0.05 m) | 상자 높이 + 0.5 m 까지 올라갔다 내려옴 |
| 목표 도달 | 상자 앞에서 멈추거나 크게 우회 | 넘어서 도달 |

> ⚠️ **모드를 바꾸려면 컨테이너를 재생성해야 하고, 그러면 지도가 비워진다.** 두 실행의
> 출발 조건을 맞추려면 같은 자리에서 시작하고, 각 실행 전에 스캔 시간을 40~60초씩
> 똑같이 준다.
>
> ⚠️ `sed`로 compose를 고치는 것은 **임시 시험용**이다. 끝나면
> `gen_fleet_compose.py`를 다시 돌려 생성된 상태로 되돌린다
> ([CLAUDE.md 절대규칙 2](CLAUDE.md)).

```bash
docker run --rm -v "%cd%:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_fleet_compose.py
```

### (f) 잘 안 될 때 보는 순서

1. **드론이 월드 안에 있나** — `ros2 topic echo /drone1/odom --field pose.pose.position`.
   `my_world`는 x −9.6~9.5, y −6.9~8.0이다. 밖으로 나가면 어떤 목표도 실패한다.
2. **`cmd_vel` 발행자가 1개인가** — `ros2 topic info /drone1/cmd_vel`.
   회피기가 단독이어야 한다. 2개면 z를 두고 싸운다.
3. **Nav2가 회피기를 거치나** — `ros2 topic info /drone1/cmd_vel_nav`.
   발행자 5(Nav2), 구독자 1(회피기)이 정상이다.
4. **지도가 드론을 가두고 있나** — [2-(e)](#e-층별-지도-보기)로 `map_active`를 보고,
   그래도 모르겠으면 컨테이너를 재생성해 지도를 비운다.
5. **물리적으로 낀 건지 확인** — `cmd_vel`을 직접 줘서 움직이면 Nav2/지도 문제다.
   ⚠️ **이때 월드 밖으로 밀어내지 않게 짧게만 준다.** 실제로 그러다 벽 위에 올려
   놓은 적이 있고, 그 뒤 모든 목표가 실패했다.

---

## 3. 왜 이 구조인가

### 연속 3D 경로계획을 안 쓴 이유

3D 플래너는 **계획 호출마다** 3D 탐색을 한다. 군집으로 가면 그 비용이 대수만큼 곱해진다.
그래서 계획은 **지금과 똑같은 2D A***로 두고, 고도라는 한 축만 바깥에서 다룬다.

- 늘어나는 비용: 층당 직선 회랑 검사(수백 칸) + 룩어헤드 상자 점 세기
- **드론 1대당 노드는 오히려 줄었다** — `pointcloud_to_laserscan` + `slam_toolbox` 2개가
  `drone_layer_mapper` 1개로 대체됐다

### Nav2를 안 고치고 z를 얹을 수 있는 이유

세 가지가 맞물린다.

1. 드라이버가 `linear.x/y/z`를 **동시에** 받고, Nav2는 `linear.z`를 **항상 0**으로 둔다.
   z축이 통째로 비어 있어 충돌 없이 쓸 수 있다.
2. `map_active`가 **고도를 따라 자동으로 바뀐다.** 올라가면 Nav2의 static layer에서
   그 장애물이 사라지므로 수평 우회와 수직 회피가 싸우지 않는다.
3. 라이다가 ±15°라 전방 몇 m의 **위아래를 이미 보고 있다.**

Nav2에서 바꾼 것은 인자 두 개뿐이다 — `map_topic`(→`map_active`),
`cmd_vel_topic`(→`cmd_vel_nav`).

### 2D 지도를 셋으로 나눈 이유

층을 쌓으면 `/{ns}/map` 하나에 여러 고도가 섞이는데, 그 토픽을 **두 소비자가 동시에** 본다
— 맵 병합기와 **드론 자신의 Nav2**. 합집합을 그대로 주면 드론이 다른 고도의 장애물 때문에
지금 고도에서는 뚫린 공간을 못 지나간다. 그래서 갈랐다.

| 토픽 | 내용 | 소비자 | 근거 |
|---|---|---|---|
| `/{ns}/map` | 층 **합집합** | 병합기·관제 | 병합 규칙이 `np.maximum`(장애물 OR)이라 드론이 3 m에서 본 빈 곳은 UGV가 0.8 m에서 본 책상에 어차피 진다 — **아무것도 지우지 않는다.** 이름 유지로 **병합기는 무수정** |
| `/{ns}/map_active` | 현재 순항 고도 | 드론 Nav2 | 플래너는 자기가 나는 층만 봐야 한다 |
| `/{ns}/map_layer_k` | 후보 층 | 선택기·회피기 | 어느 층이 열렸는지 판단 |

### SLAM을 뺀 이유

드라이버가 GPS 절대좌표를 그대로 odom으로 발행하므로 자세가 이미 정답값이다.
`slam_toolbox`는 사실상 점유 격자 누적기로만 쓰이고 있었다. 맵 병합이 이미
`odom_is_world_absolute: true`로 같은 가정 위에 서 있으므로 새 가정도 아니다.
`{ns}/map → {ns}/odom`은 항등이라 static TF 하나로 대체했다.

> 대가: 루프 클로저·드리프트 보정이 없다. **실기 이식 때는 3D SLAM이 필요하다.**

---

## 4. 노드별 역할

| 노드 | 파일 | 하는 일 |
|---|---|---|
| `drone_layer_mapper` | [drone_layer_mapper.py](src/webots_python/webots_python/drone_layer_mapper.py) | 라이다+하향 뎁스 → 층별 점유 격자 3장 + 합집합 + active |
| `altitude_selector` | [altitude_selector.py](src/webots_python/webots_python/altitude_selector.py) | 목표까지 회랑을 층마다 검사 → 순항 고도 결정 |
| `local_altitude_avoider` | [local_altitude_avoider.py](src/webots_python/webots_python/local_altitude_avoider.py) | 룩어헤드 감시 → `linear.z` 합성, `cmd_vel` 단독 발행 |

배선은 [single_drone.launch.py](src/webots_python/launch/single_drone.launch.py)에 있다.

---

## 5. 파라미터 손잡이

전부 [single_drone.launch.py](src/webots_python/launch/single_drone.launch.py)에서 준다.

### 모드 (`NAV_MODE` 환경 변수)

| 값 | 의미 |
|---|---|
| `2d` / `2.5d_local` / `2.5d` | [0장](#0-경로계획-모드-고르기) 참고 |
| `avoid_enabled` | 런치가 모드에서 계산해 회피기에 넘긴다 (직접 주지 않는다) |

### 회피 민감도 (`local_altitude_avoider`)

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `lookahead` | 3.0 | 얼마나 앞을 보나 |
| `half_width` | 0.7 | 회랑 반폭 (기체 반경 0.35 + 여유) |
| `block_above` / `block_below` | 0.25 / 0.24 | **부딪히는 높이 범위.** 기체 실측(라이다 윗면 +0.156, 랜딩기어 −0.138) + 여유 |
| `clearance` | 0.5 | 장애물 윗면 위 여유 = **상승량을 직접 정한다** |
| `ground_clearance` | 0.45 | 발밑 표면 위 최소 여유 = **착지 방지** |
| `move_threshold` | 0.05 | 이보다 느리면 회피를 시작하지 않는다 |
| `clear_hold` | 15 | 이만큼 연속 "내려가도 됨"이어야 복귀 |

### 층 판정 (`altitude_selector`)

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `max_occupied_ratio` | 0.02 | 회랑에 장애물이 이 비율 넘으면 그 층 탈락 |
| `max_unknown_ratio` | 0.35 | 미탐색 허용 비율 |
| `corridor_half_width` | 0.6 | 회랑 반폭 |

### 지도 (`drone_layer_mapper`)

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `layer_heights` | [1.0, 2.0, 3.0] | 후보 순항 고도. **매퍼·선택기·회피기가 같은 값을 봐야 한다** |
| `hit_gain` / `odds_limit` | 3 / 12 | 점유 점수. 크면 잘 안 잊고 작으면 빨리 잊는다 |
| `cloud_stride` | 4 | 점 솎기 — 군집 대비 비용 조절 |

---

## 6. 실측 결과

### 목표점 5개 (my_world, 순항 2.0 m)

| # | 목표 | 결과 | 오차 | 고도 | 방향전환 | 회피 |
|---|---|---|---|---|---|---|
| 1 | (4.0, 2.0) | SUCCEEDED | 0.10 m | 1.36~2.78 | 0 | 1 |
| 2 | (6.0, −2.0) | SUCCEEDED | 0.12 m | 1.31~2.72 | 0 | 1 |
| 3 | (0.0, −4.0) | 시간초과 | 3.08 m | 1.32~3.00 | 0 | 2 |
| 4 | (−5.0, −2.0) | SUCCEEDED | 0.07 m | 1.36~2.80 | 0 | 2 |
| 5 | (−6.0, 3.0) | SUCCEEDED | 0.22 m | 1.36~3.44 | 0 | 2 |

**방향전환 0회**가 중요하다 — 리밋 사이클(위아래 왕복)이 없다는 뜻이다.

### 전역 층 선택

| 상황 | 판정 | 결과 |
|---|---|---|
| 위가 열림 | `1m:X(22.3%) 2m:X(3.2%) 3m:OK(0.0%)` | 3.0 m로 상승, 2.00→2.97 (오차 0.03 m) |
| 아래가 열림 | `1m:OK(0.5%) 2m:X(5.9%) 3m:X` | 1.0 m로 하강, 1.04 m 안착 |
| 현재 층 열림 | `2m:OK(0.0%)` | 고도 유지 |

### 안전장치

| 시험 | 지시 | 결과 |
|---|---|---|
| 발밑 착지 방지 | 순항 고도 **0.05 m** | **0.52 m에서 멈춤** (`발밑 0.00 m → 0.45 m 아래로는 안 내려감`) |
| 정지 중 안정성 | 목표 없이 60초 | 고도 변동 **0.000 m**, 회피 0건 |

---

## 7. 겪은 함정 모음

전부 실측으로 잡은 것이고, 다시 만지다 재발하기 쉬운 것들이다.

**① 하향 센서는 회전해서 달려 있다.** PROTO의 `rotation 0 1 0 1.5708`이 센서 +x(시선)를
기체 −z(아래)로 보낸다. 빠뜨리면 **발밑 바닥이 "정면 1.9 m 앞의 벽"으로 찍힌다.**
`linear.x 0.5`를 15초 줘도 0.33 m밖에 못 갔다(정상 1.8 m). 올바른 변환은 `(sx,sy,sz) → (sz, sy, −sx)`.

**② 회랑 검사를 하드 실패로 두면 아무 층도 안 뚫린다.** "장애물·미탐색이 한 칸이라도 있으면
탈락"으로 뒀더니 **108개 방향 중 0개**였다. 지도를 그려 보니 오른쪽이 5 m 넘게 비어 있었는데
1.2 m 옆 가구 한 덩이 때문에 전부 탈락한 것. 비율 기준으로 바꾸고, 전부 탈락하면
**가장 덜 막힌 층**을 고르게 했다.

**③ 층 눈금에 스냅시키면 20 cm 턱을 넘으려고 1 m를 오른다.** 라이다로 장애물 윗면을 재서
`현재고도 + 윗면 + clearance`로 간다. 단 ±15° 시야 때문에 거리 d에서 0.268d까지만 보이므로,
꼭대기가 그 한계에 닿으면 못 잰 것으로 보고 층 단위로 물러난다.

**④ 감지 대역이 기체보다 크면 닿지도 않을 것을 피한다.** ±0.4 m로 뭉뚱그렸더니 윗면이
0.38 m **아래**인 가구까지 피했다. 기체 실측 치수로 교체.

**⑤ 감지 상자는 기체와 함께 돈다.** 제자리 요잉만으로 주변을 훑어 사방의 장애물을 잡고,
**정지 중에도** 회피를 시작한다. 전진 명령이 있을 때만 회피하도록 게이트를 걸었다.

**⑥ 복귀 조건은 "지금 비었나"가 아니라 "순항 고도로 내려가도 되나"다.** 전자로 두면 올라가는
순간 스스로 "트였다"고 판정하고 내려왔다가 다시 올라가는 **리밋 사이클**에 빠진다.

**⑦ 속도 명령이 위치 목표를 적분하는 계에 실제 위치로 피드백하면 반드시 오버슈트한다.**
드라이버는 `target_altitude += linear.z * dt`다. 실제 고도가 허용오차에 들어왔을 때
target은 이미 지나가 있어서, 명령을 끊어도 계속 간다 — **2.0 → 1.0 지시에 0.30 m까지
떨어졌다.** 보낸 명령을 같은 식으로 적분해 target을 추정하고 그 기준으로 멈춘다.

**⑧ 전방만 보는 복귀 판정으로는 장애물 위에 착지한다.** 장애물이 드론 **바로 아래**로 들어오면
룩어헤드 상자(`x > 1.05 m`)에서 빠져 "지나갔다"가 되는데 실제로는 아직 그 위다. 라이다는
`minRange`와 ±15° 원뿔 때문에 발밑을 못 본다. **하향 센서를 하강 판단에 연결해야 한다.**

**⑨ 지도가 누적만 하면 드론이 스스로를 가둔다.** `hits >= 2`를 영구 장애물로 두면 가구를
옮겨도 옛 자리가 벽으로 남는다. 목표 5개 중 3개가 ABORTED였고 **직접 `cmd_vel`을 주면
자유롭게 움직였다** — 물리적으로 낀 게 아니라 지도가 막고 있었다. 점수 방식(맞으면 +3,
광선 통과하면 −1)으로 바꿔 저절로 지워지게 했다.

**⑩ 디버깅한다고 `cmd_vel`을 오래 주면 드론이 월드 밖으로 나간다.** 실제로 x=10.5(월드 밖)로
밀어내 벽 위에 올려놨고, 그 뒤 **모든 목표가 실패**했다. 원인을 한참 지도에서 찾았다.
위 [2-(f)](#f-잘-안-될-때-보는-순서)의 5번 주의 참고.

---

## 8. 알려진 한계

- **`3d` 모드는 없다.** 런치가 명시적으로 거부한다 ([0장](#0-경로계획-모드-고르기)).
- **연속 3D 경로가 아니다.** 전역은 이산 층 선택, 지역은 반응형 상승이다. 상승과 수평이동이
  겹치기는 하지만 경로 자체가 (x,y,z) 곡선으로 최적화되지는 않는다.
- **회랑 검사는 직선만 본다.** 직선은 막혔지만 우회로가 있는 층을 놓친다(보수적).
  놓쳐도 Nav2가 현재 층에서 우회를 시도하므로 기능이 깨지지는 않는다.
- **1 m 이내 수평은 사각이다.** 라이다 `minRange`가 1 m다. 발밑은 하향 센서가 메웠지만
  옆·앞의 근접은 여전히 못 본다.
- **천장을 못 본다.** 위를 보는 센서가 없어서, 올라갈 수 있는지는 누적된 층 지도에만 의존한다.
- **시뮬레이션이 실시간의 약 23%로 돈다.** 목표 도달 시간을 잴 때 감안해야 하고,
  Nav2의 BT 타임아웃이 간헐적으로 뜨는 원인이기도 하다.

---

### 관련 문서

- [drone_setup.md](drone_setup.md) — 기체 개조·제어 구조·센서 검증
- [Readme 11장](Readme.md#11-drone-중형급-쿼드콥터) — 빠른 사용법
- [INTERFACES.md](INTERFACES.md) — 토픽·프레임 규격
- [MAP_MERGE.md](MAP_MERGE.md) — 드론 맵이 병합에 들어가는 방법
