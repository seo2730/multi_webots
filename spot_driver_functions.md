# spot_driver.py 함수 설명서

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
| `MAX_STEP_LENGTH` | 0.05 | cmd_vel로 만들 수 있는 보폭 상한 (넘으면 넘어짐, 실측 튜닝) |
| `MAX_YAW_RATE` | 0.5 | 회전 속도 상한 |
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
- **핵심**: `linear.x`는 속도(m/s)가 아니라 `StepLength = 0.15 × linear.x`로 **보폭**이 됨
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
