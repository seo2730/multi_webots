# 드론(Mavic2ProMedium) 구축 기록

DJI Mavic 2 Pro를 중형급(6.35kg)으로 개조해 월드에 넣고, UGV·Spot과 동일한
**`<extern>` 컨트롤러 + ROS 2 드라이버** 구조로 붙이기까지의 전 과정 기록.

빠른 사용법은 [Readme 11장](Readme.md#11-drone-중형급-쿼드콥터)에 있고,
이 문서는 **왜 그렇게 만들었는지**와 **어떻게 검증했는지**를 다룬다.

---

## 전체 구조 한눈에 보기

```
[Webots]                                   [Docker / ROS 2]

Mavic2ProMedium {   (런타임 소환)            fleet_spawner_windows 안의
  controller "<extern>"  ←─── TCP 1234 ───→  webots_ros2_driver
}                                                  │
  │                                                └─ drone_driver.py (플러그인)
  ├─ Propeller ×4 (RotationalMotor)                     │
  ├─ GPS / InertialUnit / Gyro / Compass               │  init() 1회
  ├─ 짐벌 3축 (HingeJoint + PositionSensor)            │  step() 매 32ms
  └─ cameraSlot: Camera 400×240                        │
                                                       ↓
                                            /drone1/cmd_vel  (구독)
                                            /drone1/odom     (발행)
                                            /tf              (발행)
                                            /drone1/camera/* (드라이버 자동)
```

Spot과 마찬가지로 일반 ROS 2 노드가 아니라 **Webots가 매 스텝 `step()`을 직접 호출**하는
플러그인이다. 그래서 통신 지연이 있어도 제어 주기가 깨지지 않는다 — 드론처럼 루프가
끊기면 추락하는 기체를 `<extern>`으로 돌릴 수 있는 이유다.

---

## 1. 기체 구성 (PROTO 개조)

### 개조 방식

순정 `Mavic2Pro.proto`(0.907kg) 기준으로:

| 항목 | 배율 | 결과 |
|---|---|---|
| 기하 (좌표·앵커·충돌체·메시) | **×2** | 대각선 약 0.7 m |
| 질량 | **×7** | 0.907 → **6.35 kg** |
| `thrustConstants` / `torqueConstants` | **×7** | — |

핵심은 **질량과 추력을 같은 비율로 키운 것**이다. 추력 대 중량비가 보존되므로
순정 제어 법칙이 그대로 유효하고, 수직 동역학이 정확히 같아진다.

### 내장 센서

PROTO에 이미 들어 있는 것 (추가 작업 불필요):

| 디바이스 | Webots 이름 | 용도 |
|---|---|---|
| GPS | `gps` | 위치 + 속도벡터 |
| InertialUnit | `inertial unit` | roll/pitch/yaw |
| Gyro | `gyro` | 각속도 |
| Compass | `compass` | 방위 |
| Camera | `camera` | 짐벌 장착 400×240 |
| 짐벌 모터 | `camera roll/pitch/yaw` | + 각각 `... sensor` |
| 프로펠러 | `front left propeller` 등 4개 | |

**거리 측정 센서는 없다.** 추가하려면 `bodySlot`(동체 고정) 또는
`cameraSlot`(짐벌 장착 — 안정화를 공짜로 받음)을 쓴다.

### Physics.damping (공기 저항)

```
physics Physics {
  mass 2.8
  damping Damping { linear 0.5  angular 0.5 }
}
```

순정 Mavic 데모 월드는 이 값을 `WorldInfo.defaultDamping`으로 **전역** 설정하는데,
`my_world.wbt`에는 없다. 전역으로 켜면 UGV·Spot 물리까지 바뀌므로 **드론 본체에만** 걸었다.

물리적으로도 이게 맞다 — 수평 제어 루프가 없던 시절 이걸 빼면 진공에서 나는 셈이 되어
미세한 기울기에도 무한히 가속했다(20초에 7.5m, 계속 증가). 실기체는 공기 저항을 받는다.

---

## 2. 제어 구조 (drone_driver.py)

### 왜 UGV와 다른가

UGV는 [robot_driver.py](src/Webots-SummitXL/workspace/simulator/simulator/robot_driver.py)에서
Twist를 메카넘 역기구학에 넣어 바퀴 속도를 뽑는다 — 대수 변환 한 줄이다.

드론은 **모터 4개로 6자유도를 제어하는 underactuated 시스템**이라 그게 불가능하다.
속도 명령과 모터 사이에 자세 안정화 루프가 반드시 끼어야 한다.

```
cmd_vel ──▶ [외부 루프: 속도 PI] ──▶ 자세 목표 ──▶ [내부 루프: 자세/고도] ──▶ 모터 4개
            (m/s를 실제로 추종)        (외란값)      (Webots 샘플에서 이식)
```

### 외부 루프 — 속도 추종 (PI)

월드 프레임 속도를 yaw로 회전해 기체 프레임으로 바꾼 뒤, 오차를 자세 외란으로 변환한다.

```python
vx_body =  cos_yaw * v[0] + sin_yaw * v[1]
vy_body = -sin_yaw * v[0] + cos_yaw * v[1]

vel_i_x += (target_vx - vx_body) * dt        # 적분 (와인드업 제한)
pitch_disturbance = -(K_VEL_P * err_vx + K_VEL_I * vel_i_x)   # 전진 = 기수 down = 음수
roll_disturbance  =  (K_VEL_P * err_vy + K_VEL_I * vel_i_y)
```

**적분항이 반드시 필요하다.** P만 쓰면 1.0 m/s 명령에 0.59 m/s만 나왔다(41% droop, 실측).
P 게인을 키워 잡으려면 약 26까지 올려야 하는데 기울기 상한에 바로 포화한다.

부수 효과로, 정지 명령(0 m/s)을 능동 추종하므로 **순정 제어 법칙의 고질적 수평 드리프트가
사라진다** (0.09 → 0.009 m/s).

### 외부 루프 — 선회 (방위 적분)

각속도 오차에 P만 걸면 같은 droop이 생긴다(0.5 rad/s 명령에 0.185 rad/s, 실측).
그래서 **목표 방위를 명령 각속도로 적분**하는 방식으로 바꿨다.

```python
yaw_hold = wrap_angle(yaw_hold + target_wz * dt)          # 목표 방위가 명령 속도로 이동
yaw_error = clamp(wrap_angle(yaw_hold - yaw), ±MAX_YAW_ERROR)
yaw_hold = wrap_angle(yaw + yaw_error)                     # 안티 와인드업 (되감기)
yaw_disturbance = K_YAW_P * yaw_error + K_YAW_RATE * (target_wz - yaw_velocity)
```

방위 오차 항이 각속도 오차의 적분 역할을 해서 정상상태 오차가 사라지고,
**`target_wz`가 0이면 목표 방위가 그대로 유지되므로 "방위 유지"가 자동으로 된다.**
모드 분기가 필요 없다.

### 내부 루프 — 자세/고도

Webots 공식 `mavic2pro` 샘플에서 이식. 게인 두 개만 다르다.

```python
roll_input  = K_ROLL_P  * clamp(roll)  + K_RATE_D * roll_velocity  + roll_disturbance
pitch_input = K_PITCH_P * clamp(pitch) + K_RATE_D * pitch_velocity + pitch_disturbance

clamped  = clamp(target_altitude - altitude + K_VERTICAL_OFFSET, -1, 1)
vertical = K_VERTICAL_P * clamped**3 - K_VERTICAL_D * v[2]

base = K_VERTICAL_THRUST + vertical
fl = base - roll_input + pitch_input - yaw_input      # 대각 프로펠러 역회전
fr = base + roll_input + pitch_input + yaw_input      # → setVelocity 시 fr, rl에 음수
rl = base - roll_input - pitch_input + yaw_input
rr = base + roll_input - pitch_input - yaw_input
```

### 게인 표

| 상수 | 값 | 출처 / 근거 |
|---|---|---|
| `K_VERTICAL_THRUST` | 68.5 | 순정 |
| `K_VERTICAL_OFFSET` | 0.6 | 순정 |
| `K_VERTICAL_P` | 3.0 | 순정 |
| **`K_VERTICAL_D`** | **8.0** | **추가.** 순정은 P항만이라 이중적분기에서 준안정 → 39% 오버슈트 |
| `K_ROLL_P` | 50.0 | 순정 |
| `K_PITCH_P` | 30.0 | 순정 |
| **`K_RATE_D`** | **2.0** | **상향.** 회전 관성이 제어 토크보다 빠르게 증가(×28 vs ×14) |
| **`K_VEL_P` / `K_VEL_I`** | **2.0 / 1.0** | **추가.** P만으로는 41% droop |
| **`K_YAW_P` / `K_YAW_RATE`** | **6.0 / 1.5** | **추가.** 방위 적분 방식 |
| `MAX_TILT_DISTURBANCE` | 4.0 | 과도한 기울기 방지 (약 7.6°) |
| `VEL_I_LIMIT` / `MAX_YAW_ERROR` | 4.0 / 0.5 | 안티 와인드업 |

---

## 3. 인터페이스 규격

### cmd_vel (UGV와 다름, 주의)

| 필드 | 의미 | 비고 |
|---|---|---|
| `linear.x` | 전후 속도 (m/s) | 기체 기준 |
| `linear.y` | 좌우 속도 (m/s) | **기체가 기울어서 이동** (메카넘과 다름) |
| `linear.z` | **상승 속도** (m/s) | 목표 고도를 적분. 위치 명령이 아님 |
| `angular.z` | 선회 각속도 (rad/s) | |

**Nav2 호환**: Nav2는 `linear.x` / `angular.z`만 쓰므로 고도 고정 상태로 그대로 붙는다.

### odom

UGV와 달리 **z와 roll/pitch까지 싣는다** (드론은 3D로 움직이므로).
`twist.twist.linear`은 **기체 프레임** 속도다.

프레임: `drone1/odom` → `drone1/base_link`

---

## 4. 검증 방법

### 헤드리스 하네스

`my_world.wbt`는 헤드리스로 못 돌린다 — UGV·Spot이 `<extern>`이라 Webots가
컨트롤러 연결을 기다리며 멈춘다. 그래서 스크래치패드에 **격리 프로젝트**를 만들었다.

```
dronetest/
  worlds/test_*.wbt          아레나 + 드론 + supervisor 로거만
  protos/                    실제 PROTO 사본
  controllers/logger/        wb_supervisor_node_get_position() 을 초당 1회 출력
```

```bash
webots.exe --batch --mode=fast --no-rendering --minimize --stdout --stderr <world>
```

GUI를 눈으로 보는 대신 **정착 시간·오버슈트·드리프트·속도 추종을 숫자로** 잴 수 있다.

### C 미러링

이 PC에는 실제 Python이 없어서(Store 스텁만) Python 드라이버를 직접 못 돌린다.
그래서 **같은 수식을 C로 미러링해 알고리즘만 먼저 검증**했다.
위험한 건 새로 짠 외부 루프이지 ROS 배선이 아니었기 때문(배선은 UGV 템플릿 그대로).

이 방식으로 속도 droop(41%)과 선회 droop(63%)을 실환경 이전에 발견해 고쳤다.

### 실환경 검증 결과

| 항목 | 헤드리스 예측 | 실환경 (ROS 2) |
|---|---|---|
| 목표 고도 2.0 m | 1.998 m, 오버슈트 0 | `alt=2.00/2.00` |
| 호버 드리프트 | 0.009 m/s | `vbody=(0.00, -0.00)` |
| 전진 1.0 m/s | 1.01 m/s | **1.00 m/s** |
| 선회 0.5 rad/s | 0.50 rad/s | — |
| 자세 | level 1.000 | `rp=(0.000, 0.000)` |

예측과 실환경이 일치해서 미러링 검증 방식 자체도 유효함이 확인됐다.

---

## 5. 해결된 이슈 (트러블슈팅 기록)

### ① 드론이 투명하게 보임

Webots R2025a는 메시·텍스처 에셋을 설치본에 포함하지 않는다
(`projects/robots/dji/mavic/` 폴더 자체가 없음). `webots://` 참조가 **조용히 실패**해
셰이프가 하나도 안 그려졌다. 물리는 정상이라 "안 보이는데 날아다니는" 상태가 됐다.

→ 메시 14개 + 텍스처를 `protos/Mavic2Pro/`에 로컬 포함하고 상대 경로로 참조.

### ② 고도가 0.2~4.2 m로 진동

순정 데모 월드의 `WorldInfo.defaultDamping`에 의존해 안정화되고 있었는데
`my_world.wbt`에는 그 설정이 없었다. 격리 테스트로 원인을 분리:

| 테스트 | basicTimeStep | 댐핑 | 결과 |
|---|---|---|---|
| A | 32 (기본) | ✗ | 진동 |
| C | 8 | ✗ | 진동 (동일) |
| D | 32 (기본) | ✓ | **2.0 m 안착** |
| B | 8 | ✓ | 안착 |

**`basicTimeStep`은 무관**했다 → 8ms로 낮출 필요가 없어 시뮬레이션 속도 손해가 없다.
→ 드론 PROTO의 `Physics.damping`에만 적용.

### ③ 고도 39% 오버슈트

순정은 고도를 **P항만으로** 제어한다. 이중적분기에 P만 걸면 준안정이라,
②의 댐핑이 사실상 D항을 대신하고 있었다.

→ `k_vertical_d` 추가(`gps.getSpeedVector()[2]` 사용). 2.79 m 오버슈트가 사라지고
2.000 m에 단조 수렴. 댐핑은 공기 저항으로서 유지 — 역할이 다르다.

| 구성 | 오버슈트 | 20초 수평 이동 |
|---|---|---|
| 댐핑만 | 2.79 m (+39%) | 1.11 m |
| D항만, 댐핑 제거 | 없음 | 7.46 m (가속) |
| **댐핑 + D항** | **없음** | **1.07 m** |

### ④ 속도 / 선회 정상상태 오차

P 제어의 droop. 1.0 m/s → 0.59 m/s, 0.5 rad/s → 0.185 rad/s.
→ 속도는 적분항, 선회는 방위 적분 방식으로 해결 (2장 참고).

### ⑤ Webots GUI 저장 시 컨트롤러가 `"<none>"`으로 바뀜

Spot.proto 절대경로 변형과 같은 부류의 현상. 저장 후
`controller "<extern>"`인지 확인할 것. 또한 **월드를 열어둔 채 파일을 수정했으면
반드시 리로드(Ctrl+Shift+R)** — 저장(Ctrl+S)하면 옛 메모리 상태가 디스크를 덮어쓴다.

### ⑥ 토픽은 생기는데 데이터가 안 나옴

**Webots는 월드의 `<extern>` 로봇이 전부 연결돼야 스텝을 밟는다.**
현재 4대(ugv1, ugv2, spot1, drone1)이므로 drone1만 띄우면 시뮬레이션이 멈춰 있다.

결정적 단서는 **드라이버가 자체 발행하는 `/drone1/gps`도 같이 멈춰 있던 것** —
내 코드 문제가 아님을 가려낼 수 있었다.

---

## 6. 알려진 한계 / 다음 작업

### 수직 이동 속도가 0.375 m/s로 제한됨

로그에서 하강 시 `vz`가 정확히 -0.37에 고정된다. 원인은 `K_VERTICAL_P=3.0`의 3차항이
±3.0에서 포화하고 `K_VERTICAL_D=8.0`과 균형을 이루는 지점(3.0 / 8.0 = 0.375).

**설계상 그런 것이지 버그가 아니다.** 목표 근처에서는 문제없지만 고도를 크게 바꾸면
매우 느리다. 고고도 정찰을 하려면 조정이 필요하다.

### 그 외

- **거리 센서 없음** — 자율 비행·장애물 회피에 필요. 뎁스카메라(`cameraSlot`, 짐벌 안정화를
  공짜로 받음) + 하향 거리센서 조합이 유력. **이게 붙어야 SLAM/Nav2를 연결할 수 있다.**
- **속도 제어이지 위치 제어가 아님** — `cmd_vel`이 0이면 속도 0을 유지하지만, 외란에 밀린 뒤
  원위치로 돌아가지는 않는다. 웨이포인트 비행에는 위치 루프가 필요하며, Nav2가 그 역할을 한다.
- **짐벌이 각속도 댐핑만 함** — 순항 중 기체가 5~15° 기울면 카메라도 같이 기운다.
  정찰 용도로는 자세 자체를 상쇄하도록(`-roll`/`-pitch`) 바꾸는 편이 낫다.
- **시뮬레이션이 실시간의 약 27%로 동작** (컨테이너 4개 + GUI 렌더링). 테스트 시 감안할 것.

### 폐기된 것

초기에는 Webots 내장 C 컨트롤러(`controllers/mavic2pro_medium/`)로 키보드 조종을 했으나,
**OS별 컴파일이 필요해 이식성이 없고 ROS 2 미션 스택에 붙일 수 없어** 폐기했다.
5장의 이슈 ①~③은 그 컨트롤러로 규명한 것이며 결론은 그대로 유효하다.
복원이 필요하면 `git log -- workspace/simulator/controllers/`.

---

## 7. 파일 맵

| 파일 | 역할 |
|---|---|
| `src/Webots-SummitXL/workspace/simulator/protos/Mavic2ProMedium.proto` | 개조 PROTO |
| `.../protos/Mavic2Pro/` | 메시 14개 + 텍스처 (로컬 포함) |
| `.../simulator/simulator/drone_driver.py` | webots_ros2 플러그인 (2단 제어 + odom/TF) |
| `.../simulator/simulator/drone_teleop.py` | 키보드 조종 (고도 축 있음) |
| `.../simulator/worlds/my_world.wbt` | `DEF DRONE1` 인스턴스 |
| `src/webots_python/urdf/Mavic2ProMedium.urdf.xacro` | 플러그인 연결 + 디바이스 매핑 |
| `src/webots_python/launch/single_drone.launch.py` | 런치 (`ROBOT_ID` 방식) |
| `docker-configs/*/docker-compose.yml` | `drone1` 서비스 |
