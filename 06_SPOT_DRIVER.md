# 06. Spot 드라이버 함수 설명서 (spot_driver.py)

> 📖 [책 목차](Readme.md#-목차) · ← [05. Spot 구축](05_SPOT_SETUP.md) · [07. Spot 자율주행](07_SPOT_NAV.md) →

`webots_spot/spot_driver.py`는 Webots의 Spot 로봇을 제어하는 **webots_ros2_driver 플러그인**.
일반 ROS2 노드가 아니라, Webots가 시뮬레이션 스텝마다 `step()`을 직접 호출해주는 구조
(`init()` 1회 → 매 스텝 `step()` 반복).

## 전체 구조 한눈에 보기

```
[ROS2 외부 입력]                        [매 스텝 실행 루프: step()]
/spot1/cmd_vel ──→ __cmd_vel ─┐          ┌─ float_motion()        (float_mode일 때)
/spot1/inverse_gait_input     ├─ 상태 ──→│─ defined_motions()     (자세 서비스 실행 중일 때)
/spot1/stand_up 등 서비스 ────┘          └─ spot_inverse_control() (평상시 = 걷기)
                                              ↓
                                         handle_transforms_and_odometry()
                                         (TF/odom/joint_states 발행)
```

동작 모드는 3개이며 `step()`이 매 스텝 하나를 선택:
| 모드 | 조건 | 담당 함수 |
|---|---|---|
| 호버링 | `float_mode == True` | `float_motion()` |
| 고정 자세 재생 | `fixed_motion == True` (stand/sit 등 서비스 호출 직후) | `defined_motions()` |
| 보행 (기본) | 그 외 | `spot_inverse_control()` |

---

## 모듈 상수

| 이름 | 값 | 의미 |
|---|---|---|
| `NUMBER_OF_JOINTS` | 12 | 다리 4개 × 관절 3개(어깨 벌림/어깨 회전/팔꿈치) |
| `HEIGHT` | 0.52 | 서 있을 때 몸통(base_link)의 지면 기준 높이(m) |
| `MAX_STEP_LENGTH` | **0.090** | cmd_vel로 만들 수 있는 보폭 상한 (넘으면 넘어짐, 실측 튜닝). 케이던스 운용점에서 **0.195 m/s** |
| `MIN_STEP_LENGTH` | 0.015 | 보폭 하한. 이보다 작으면 발이 안 떨어진다. 같은 운용점에서 **약 0.045 m/s 미만을 못 내는 이유** |
| `SPEED_COEF` / `SPEED_EXP` | **1.371** / 0.81 | m/s -> 보폭 환산 ([cmd_vel 단위](#cmd_vel-단위)). 지수는 형상, 계수는 운용점에서 재교정 |
| `swing_period` / `step_velocity` | 0.12 / 3.0 | **런타임 파라미터.** 케이던스 — 바꾸면 위 속도값이 전부 따라 바뀐다 |
| `YAW_PER_RADPS` | **8.11** | rad/s -> YawRate 환산. 상한 2.0 ÷ 포화 0.247 rad/s |
| `MAX_YAW_RATE` | **2.0** | 회전 속도 상한. 0.5 였을 때 제자리 회전이 0.079 rad/s 였다 (180도에 40초) |
| `motions` | dict | "stand"/"sit"/"lie" 각각의 목표 관절 각도 12개 (자세 서비스가 사용) |

---

## 쿼터니언/오일러 변환 헬퍼 (모듈 함수)

로봇 자세 표현을 서로 변환하는 순수 수학 함수들. Webots는 축-각(axis-angle),
ROS는 쿼터니언, 사람이 보긴 오일러(roll/pitch/yaw)가 편해서 3종 변환이 다 필요함.

| 함수 | 입력 → 출력 | 용도 |
|---|---|---|
| `quaternion_to_euler(q)` | 쿼터니언 → (roll, pitch, yaw) | odom twist 계산, yaw 추출 |
| `quaternion_from_euler(a)` | (roll, pitch, yaw) → 쿼터니언 | odom 메시지 orientation 채우기 |
| `quat_from_angle_axis(aa)` | Webots 축-각 → 쿼터니언 | Webots rotation 필드 읽을 때 |
| `diff_quat(q2, q1)` | 두 쿼터니언 → q2 ⊗ q1⁻¹ (상대 회전) | 현재는 미사용 (과거 상대좌표 odom의 잔재) |

---

## SpotDriver 클래스

### `init(webots_node, properties)` — 초기화 (1회)

webots_ros2_driver가 로봇 접속 시 한 번 호출. 하는 일 순서대로:

1. **네임스페이스 설정** — URDF plugin에서 넘어온 `namespace`(예: `spot1`)로 노드/토픽/TF 프레임 이름 구성
2. **Supervisor 핸들 획득** — `getFromDef("Spot")`으로 월드의 Spot 노드를 잡음 (정답 위치 읽기용. 이래서 월드에 `supervisor TRUE` 필수)
3. **모터 12개 + 위치센서 12개 획득/활성화**
4. **ROS 인터페이스 생성** — 구독: `cmd_vel`, `inverse_gait_input` / 발행: `joint_states`, `odom` / 서비스: `stand_up`·`sit_down`·`lie_down`·`shake_hand`·`set_height`·`blocksworld_pose`·`float_mode`
5. **거리센서 4개 확인** — 있으면 활성화, 없으면 `float_mode`만 비활성화 (월드의 `middleExtension` 참고)
6. **터치센서 4개 확인** — 발끝 접지 감지용, 없어도 동작
7. **보행 엔진 초기화** — `SpotModel`(다리 역기구학)과 `BezierGait`(발끝 궤적 생성기) 준비

### `step()` — 매 시뮬레이션 스텝 (32ms)

드라이버의 심장박동. 매 스텝:
1. `rclpy.spin_once` — 밀린 ROS 콜백(cmd_vel, 서비스 등) 처리
2. 터치센서로 네 발의 접지 상태 갱신
3. 모드에 따라 `float_motion()` / `defined_motions()` / `spot_inverse_control()` 중 하나 실행
4. `handle_transforms_and_odometry()` — TF/odom/joint_states 발행
5. `__model_cb()` — 현재 yaw 갱신

---

### 보행 (걷기) 관련

#### `__cmd_vel(msg)` — cmd_vel 구독 콜백
`geometry_msgs/Twist`를 Bezier 보행 파라미터로 번역.
- **핵심**: `linear.x`는 **m/s**다. 내부에서 `step_from_speed()`가 비선형 환산해 보폭이 된다

<a id="cmd_vel-단위"></a>
##### cmd_vel 단위 — 2026-08-16 변경

예전에는 `linear.x`가 **보폭 배율**(`StepLength = 0.15 × linear.x`)이었고 `angular.z`는
YawRate로 그대로 들어갔다. 그런데 Nav2의 DWB는 `cmd_vel`을 m/s·rad/s로 알고 궤적을
예측한다 — **컨트롤러의 세계 모형이 로봇과 어긋난 채로 돌았다.**

실측(Webots, 20초 구간, 시뮬 23% 속도 보정):

| `linear.x` | 0.100 | 0.200 | 0.333 | 0.500 | 1.000 |
|---|---|---|---|---|---|
| 실제 m/s | 0.096 | 0.169 | **0.256** | 0.249 | 0.249 |

| `angular.z` | 0.200 | 0.300 | 0.500 | 0.800 |
|---|---|---|---|---|
| 실제 rad/s | 0.106 | 0.164 | **0.264** | 0.265 |

회전은 **실제가 예측의 0.53배**로 고정이었다. 상한을 어떻게 잡아도 이 비율은 안 바뀌므로
파라미터로는 못 고친다. 그래서 포화 지점에서 역산해 환산 계수를 넣었다.

| 상수 | 값 | 유도 |
|---|---|---|
| `YAW_PER_RADPS` | 1.9 → **4.37** | YawRate 상한 0.5 ÷ 포화 각속도 (회전은 선형이었다) |
| `SPEED_COEF` / `SPEED_EXP` | 2.92 → **1.371** / 0.81 | 전진은 **비선형** — 아래 참고. 계수는 케이던스를 올린 뒤 재교정했다 |

> 이 절의 숫자는 벽시계 측정이라 절대값이 틀렸다. 확정값은
> [아래](#시뮬-시각으로-재측정해-확정했다-)에 있다.

**전진은 계수 하나로 안 된다.** 보폭이 작을수록 효율이 높다.

| StepLength | 0.015 | 0.030 | 0.050 |
|---|---|---|---|
| 실제 m/s | 0.096 | 0.169 | 0.256 |
| m/s ÷ L | 6.4 | 5.6 | 5.1 |

처음엔 포화점에 맞춘 선형 계수(`STEP_PER_MPS = 0.195`)를 썼는데 **저속에서 33% 초과속**이
났다. 로그-로그 회귀로 `v = 2.92·L^0.81` 을 얻어 역산한다.

```python
StepLength = (|v| / SPEED_COEF) ** (1/0.81)      # step_from_speed()
```

재측정 — 명령과 실제가 맞는다:

| 명령 m/s | 0.10 | 0.15 | 0.25 |
|---|---|---|---|
| 선형 환산 | +33% | +17% | +11% |
| **비선형 환산** | **−2%** | **−13%** | **+4%** |

| 명령 rad/s | 0.10 | 0.18 | 0.26 |
|---|---|---|---|
| 오차 | +5% | +3% | +7% |

##### 케이던스를 올린 뒤 계수를 다시 잡았다

위 표는 **기본 케이던스**에서 잰 것이다. 그 뒤 `swing_period`/`step_velocity`를 노출해
케이던스를 올리자([07장 3절 ⑦](07_SPOT_NAV.md#-속도를-올리려면-보폭이-아니라-케이던스다))
같은 보폭이 더 빠른 속도를 내게 됐다. 그래서 운용점(`swing=0.12`, `stepV=3.0`)에서 한 점
교정했다.

##### 시뮬 시각으로 재측정해 확정했다 ✅

위 표들은 전부 **벽시계로 재고 시뮬 속도를 23% 고정으로 가정**한 값이라 최대 4배까지
틀렸다 ([07장 4절](07_SPOT_NAV.md#-속도를-벽시계로-쟀다)). `odom.header.stamp`
기준으로 다시 재서 확정한 값은 이렇다.

| | 값 | 비고 |
|---|---|---|
| 최고 안정속 (L = `MAX_STEP_LENGTH` 0.090) | **0.195 m/s** | 이 이상은 자세가 흔들린다 |
| 최저 속도 (L = `MIN_STEP_LENGTH` 0.015) | **약 0.045 m/s** | 아래는 발이 안 떨어진다 |
| 제자리 회전 | **0.247 rad/s** | `MAX_YAW_RATE` 2.0 에서 포화. 2.5 로 올리면 roll 이 6.5도 -> 13도로 두 배 |

```
L = 0.090 에서 0.195 m/s  =>  0.195 = coef · 0.090^0.81  =>  coef = 1.371
                                YAW_PER_RADPS = 2.0 / 0.247 = 8.11
```

지수 0.81은 곡선의 **형상**이라 그대로 뒀다. 1점 교정이므로 **저속 쪽은 외삽**이다.

⚠️ 위쪽 표들(벽시계 측정)의 **절대값은 믿지 말 것.** 단위가 m/s라는 것과 항목 간
상대 관계, 그리고 "전진은 비선형·회전은 선형"이라는 결론만 유효하다.

⚠️ 보폭 하한 때문에 **최소 속도 아래는 아예 못 낸다.** 물리적 제약이라 없앨 수 없다
(없애면 발이 안 떨어진다). 대신 Nav2 쪽에 최저 속도를 알려 **못 내는 속도를 계획하지
않게** 했다 — nav2_spot.yaml 참고.

⚠️ 이 계수를 바꾸면 [nav2_spot.yaml](src/Webots-SummitXL/workspace/navigation/param/nav2_spot.yaml)의
`max_vel_x`/`max_vel_theta`도 같이 바꿔야 한다. 한 쌍이다.
- `linear.y` → `LateralFraction` (게걸음 방향), `angular.z` → `YawRate`(제자리 회전)
- 너무 작은 보폭은 최소값으로 올리고(발이 안 떨어지는 것 방지), `MAX_STEP_LENGTH`/`MAX_YAW_RATE`로 **상한 클램프** (넘어짐 방지, 우리가 추가)
- `inverse_gait_input`에 발행자가 있으면 무시됨 (수동 보행 튜닝이 우선권을 가짐)

#### `__gait_cb(msg)` — inverse_gait_input 구독 콜백
cmd_vel의 "자동 번역"을 거치지 않고 보행 파라미터 15개(보폭, 스윙 주기, 발 들어올림 높이,
몸통 자세 등)를 **직접 지정**하는 전문가용 입력. 보행 튜닝/실험할 때 사용.

#### `spot_inverse_control()` — 보행 1스텝 계산 (기본 모드의 본체)
매 스텝:
1. `BezierGait.GenerateTrajectory()` — 현재 보행 파라미터와 발 접지 상태로 **네 발끝의 목표 위치**(베지어 곡선 궤적)를 계산
2. `SpotModel.IK()` — 발끝 위치를 만들기 위한 **관절 각도 12개**를 역기구학으로 계산
3. `__talker()`로 모터에 전달

#### `yaw_control()` — 방향 유지 컨트롤러
`YawControlOn`일 때(= `inverse_gait_input` 전문가 입력에서 켰을 때만), 목표 방위각(`YawControl`)과
현재 yaw의 차이에 비례해 회전 속도를 계산하는 P형 제어기. ±180° 경계 넘어갈 때의
최단 방향 회전 처리 포함. cmd_vel 주행에서는 쓰이지 않음.

#### `__talker(motors_target_pos)` — 모터 출력
계산된 관절 각도 12개에 관절별 기본 오프셋(어깨 0, 회전 +0.52, 팔꿈치 −1.182)을 더해
실제 모터에 `setPosition`. IK 결과와 모터 영점의 차이를 보정하는 마지막 단계.

---

### 자세(고정 모션) 관련

#### `movement_decomposition(target, duration)` — 모션 보간 준비
목표 관절 각도 12개까지 `duration`초 동안 부드럽게 이동하도록, 스텝당 이동량
(`step_difference`)을 계산해둠. 실제 이동은 `defined_motions()`가 수행.
급격한 관절 이동으로 로봇이 튕겨나가는 것을 막는 장치.

#### `defined_motions()` — 고정 모션 재생 (fixed_motion 모드의 본체)
매 스텝 `step_difference`만큼 관절을 이동시켜 목표 자세로 수렴.
악수(`paw`) 모드일 땐 도착 후 4초간 앞다리를 sin 곡선으로 흔들고 원위치.

#### 자세 서비스 콜백 4형제
`__stand_motion_cb` / `__sit_motion_cb` / `__lie_motion_cb` / `__shakehand_motion_cb`
- 각각 `motions` 딕셔너리의 목표 자세(악수는 인라인 각도)를 `movement_decomposition`에 넘기고 `fixed_motion = True`로 전환
- 이전 모션 재생 중이면 거부 — `{override: true}`로 강제 실행 가능

#### `__spot_height_cb(request, response)` — 몸높이 조절 서비스
±0.2m 범위에서 `zd`(몸통 높이 오프셋)를 설정. 걷기 IK에 반영되어 앉은 채 걷기/까치발 같은 효과.

#### `blocksworld_pose(request, response)` — MASKOR 아레나 전용 잔재
로봇을 특정 좌표(-7.28, -3.78)로 **순간이동**시키고 눕히는 서비스. MASKOR의 블록 쌓기
데모용이라 우리 월드에서는 의미 없음 (호출하지 말 것. 추후 제거 후보).

---

### 호버링(float_mode) 관련

#### `float_mode_cb(request, response)` — float_mode 켜기/끄기 서비스
거리센서 4개가 없으면 켜기를 거부(우리가 추가한 방어). 켜면 걷기 대신 `float_motion()`이 실행됨.

#### `float_motion()` — 호버링 (물리 무시 이동)
다리를 편 채 로봇을 **supervisor로 직접 순간이동**시키며 나는 것처럼 이동:
1. 하향 거리센서 4개로 지면까지 거리 측정
2. **PD 제어 + 저역통과필터**로 목표 높이(0.54m) 유지, 4개 센서의 앞뒤/좌우 차이로 지형 경사에 맞춰 pitch/roll 기울임
3. cmd_vel의 vx/vy/wz를 적분해 위치/방향 갱신 → Webots 필드에 직접 기록
4. 매 스텝 물리 리셋(`resetPhysics`) — 중력으로 떨어지지 않게 함
지형 추종 데모용이며, 물리를 속이는 방식이라 SLAM/Nav2 주행과 같이 쓰는 건 비추천.

---

### 상태 발행 관련

#### `handle_transforms_and_odometry()` — TF/odom/joint_states 발행 (매 스텝)
1. 관절 위치센서 12개 읽기
2. **odom→base_link TF**: supervisor가 주는 **월드 절대좌표·절대자세를 그대로** 발행
   (UGV `robot_driver.py`와 동일 컨벤션. 원래 MASKOR는 "접속 시점 기준 상대좌표"였는데
   맵 뒤집힘/드리프트의 원인이라 우리가 절대좌표로 변경 — Readme 10-6 참고)
3. **base_link→base_footprint TF**: 몸통 높이만큼 아래(지면)에 투영한 프레임
4. **`odom` 토픽**: 위 TF와 같은 pose + 이전 스텝과의 차분으로 구한 twist(속도)
5. **`joint_states` 토픽**: 관절 12개 + 무동력 관절(피스톤) 4개의 각도 → robot_state_publisher가 다리 TF를 만드는 재료

#### `__model_cb()` — 현재 yaw 갱신
supervisor에서 로봇의 현재 월드 yaw를 읽어 `yaw_inst`에 저장. `yaw_control()`의 피드백 입력.
float_mode 중에는 스킵.

---

## 외부 의존 모듈 (이 파일에는 없음)

| 모듈 | 역할 |
|---|---|
| `SpotKinematics.SpotModel` | 몸통 자세+발끝 위치 → 관절 각도 (역기구학). 내부적으로 `LegKinematics`/`LieAlgebra` 사용 |
| `Bezier.BezierGait` | 보행 파라미터 → 발끝 궤적 (스윙은 베지어 곡선, 스탠스는 사인 곡선) |

---

## 관련 문서

- [Readme 10절](Readme.md#10-spot-사족보행-로봇) — 사용법, 센서 구성, 해결된 이슈
- [01_INTERFACES.md](01_INTERFACES.md) — 자세 제어 서비스 목록과 `cmd_vel` 의미 차이
- [03_SPAWNER.md](03_SPAWNER.md) — Spot만 `DEF` 이름이 필요한 이유 (`getFromDef`)
- [10_MAP_MERGE.md](10_MAP_MERGE.md) — Spot 팔 관절을 0으로 채우는 이유
- [04_UGV_SETUP.md](04_UGV_SETUP.md) / [08_DRONE_SETUP.md](08_DRONE_SETUP.md) — 다른 두 로봇

---

> 📘 튜닝 과정에서 겪은 이슈와 해결은 [07_SPOT_NAV.md](07_SPOT_NAV.md)에 따로 정리했다.

## 자율주행 직접 테스트하기

Spot의 Nav2 파라미터는 [nav2_spot.yaml](src/Webots-SummitXL/workspace/navigation/param/nav2_spot.yaml)에
따로 있다(UGV·드론과 다르다). 값을 바꾼 뒤 이렇게 확인한다.

### 🚨 먼저 알아야 할 함정

**테스트 노드에 `use_sim_time`을 반드시 켠다.** 안 켜면 목표 스탬프가 벽시계 시각이라
Nav2가 수백 초 과거로 TF를 조회하고 **로봇이 한 발도 안 뗀다.**

```
Extrapolation Error: Requested time 2982.516 but the earliest data is at time 3364.256
```

이 메시지가 보이면 파라미터 문제가 아니라 이것이다. 실제로 이것 때문에 세 번을 헛돌았다.

```python
n = Node('test', parameter_overrides=[Parameter('use_sim_time', value=True)])
```

### 속도 환산이 맞는지 (명령 = 실제인가)

```bash
docker exec spot1_brain_windows bash -c "source /ros2_ws/install/setup.bash && python3 - <<'EOF'
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rclpy.init(); n=Node('v'); S={}
n.create_subscription(Odometry,'/spot1/odom',lambda m:S.__setitem__('o',m),10)
pub=n.create_publisher(Twist,'/spot1/cmd_vel',10)
t0=time.time()
while time.time()-t0<20 and 'o' not in S: rclpy.spin_once(n,timeout_sec=0.1)
pos=lambda: (S['o'].pose.pose.position.x, S['o'].pose.pose.position.y)
for v in [0.10, 0.15, 0.25]:
    pub.publish(Twist()); t=time.time()
    while time.time()-t<4: rclpy.spin_once(n,timeout_sec=0.05)
    p0=pos(); t=time.time(); tw=Twist(); tw.linear.x=v
    while time.time()-t<20:
        pub.publish(tw); rclpy.spin_once(n,timeout_sec=0.02); time.sleep(0.02)
    pub.publish(Twist()); p1=pos()
    sim=(time.time()-t)*0.23          # 시뮬이 실시간의 ~23% 로 돈다
    act=math.hypot(p1[0]-p0[0],p1[1]-p0[1])/sim
    print(f'{v:.2f} -> {act:.3f} m/s ({(act-v)/v*100:+.0f}%)')
EOF"
```

오차 ±15% 안이면 정상이다. 크게 벗어나면 `spot_driver.py`의 `SPEED_COEF`/`SPEED_EXP`를
다시 잡아야 한다 ([cmd_vel 단위](#cmd_vel-단위)).

### 목표점 주행

목표는 **맵에서 빈 곳을 골라** 준다. 임의 좌표를 쓰면 월드 밖이나 벽 속을 찍어 결과가
무의미해진다(실제로 그렇게 세 번 날렸다).

```bash
docker exec spot1_brain_windows bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic pub -1 /spot1/goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: \"spot1/map\"}, pose: {position: {x: -6.0, y: -4.0}, orientation: {w: 1.0}}}'"
```

> ⚠️ `ros2 topic pub`은 스탬프가 0이라 Nav2가 "최신"으로 처리해 통과한다. 하지만
> 액션(`navigate_to_pose`)으로 스크립트를 짤 때는 위의 `use_sim_time`이 필수다.

### 무엇을 보면 되는가

| 지표 | 정상 | 비정상일 때 볼 곳 |
|---|---|---|
| 목표 오차 | < 0.5 m (`xy_goal_tolerance`) | 회전 환산(`YAW_PER_RADPS`) |
| **배회 배수** (주행 ÷ 직선거리) | 1~2배 | 컨트롤러 설정 (RPP `lookahead_dist`) |
| 주행 0.00 m | — | `use_sim_time`, 시뮬 정지, Spot이 앉아 있는지 |
| `GridBased: failed to create plan` | — | `footprint` 폭, `inflation_radius` |

### 시뮬이 멈추는 함정

**드론(`drone1`) 컨테이너를 내리면 시뮬레이션 전체가 멈춘다.** 드론은
`synchronization TRUE`라 Webots가 매 스텝 드론 컨트롤러를 기다린다. Spot만 테스트한다고
다른 로봇을 내릴 때 드론은 반드시 살려 둔다 (`ugv1`도 `/clock` 발행자라 유지).
증상은 `/spot1/odom`이 0건으로 뚝 끊기는 것이다.

---

← [05. Spot 구축](05_SPOT_SETUP.md) | [📖 책 목차](Readme.md#-목차) | [07. Spot 자율주행](07_SPOT_NAV.md) →
