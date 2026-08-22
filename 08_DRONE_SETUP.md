# 08. 드론(Mavic2ProMedium) 구축 기록

> 📖 [책 목차](Readme.md#-목차) · ← [07. Spot 자율주행](07_SPOT_NAV.md) · [09. 드론 자율비행](09_DRONE_NAV.md) →

DJI Mavic 2 Pro를 중형급(6.35kg)으로 개조해 월드에 넣고, UGV·Spot과 동일한
**`<extern>` 컨트롤러 + ROS 2 드라이버** 구조로 붙이기까지의 전 과정 기록.

빠른 사용법은 [Readme 11절](Readme.md#11-drone-중형급-쿼드콥터)에 있고,
이 문서는 **왜 그렇게 만들었는지**와 **어떻게 검증했는지**를 다룬다.

---

## 전체 구조 한눈에 보기

```
[Webots]                                   [Docker / ROS 2]

Mavic2ProMediumSensorized {  (매니페스트로 소환)  drone1_brain_windows
  controller "<extern>"  ←─── TCP 1234 ───→  webots_ros2_driver
}                                                  │
  │                                                └─ drone_driver.py (플러그인)
  ├─ Propeller ×4 (RotationalMotor)                     │
  ├─ GPS / InertialUnit / Gyro / Compass               │  init() 1회
  ├─ 짐벌 3축 (HingeJoint + PositionSensor)            │  step() 매 32ms
  ├─ cameraSlot: Camera 400×240                        │
  ├─ bodySlot: Velodyne VLP-16   ← 래퍼가 얹은 것      │
  └─ bodySlot: RangeFinder(하향) ← 래퍼가 얹은 것      │
                                                       ↓
                                            /drone1/cmd_vel  (구독)
                                            /drone1/odom     (발행)
                                            /tf              (발행)
                                            /drone1/camera/* (드라이버 자동)
                                            /drone1/Velodyne_VLP_16/point_cloud
                                            /drone1/down_depth/point_cloud
                                                       │
                                            drone_layer_mapper  (7절)
                                            ├─▶ /drone1/map         층 합집합 ─▶ 맵 병합기
                                            ├─▶ /drone1/map_active  현재 층   ─▶ Nav2
                                            └─▶ /drone1/map_layer_k          ─▶ altitude_selector
                                                                                    │
                            /drone1/goal_pose_3d ─▶ 층 선택 ─▶ 고도 이동 ─▶ Nav2 ─▶ cmd_vel
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

### 라이다 (나중에 추가)

기체 PROTO에는 **거리 측정 센서가 없었다.** 그래서 드론만 SLAM을 못 돌렸다
(`robot_types.DRONE.has_map`이 `False`였던 이유).

UGV·Spot과 같은 방식 — 센서를 품은 래퍼 PROTO — 로 VLP-16을 얹었다.

| | |
|---|---|
| 래퍼 | `protos/Mavic2ProMediumSensorized.proto` |
| 장착 위치 | `bodySlot`, 동체 위 **0.12 m** |
| 디바이스 이름 | `Velodyne_VLP_16` (URDF가 참조 — 바꾸면 안 됨) |
| 질량 | **없음** (`lidarPhysics FALSE`) |

**왜 뎁스카메라가 아니라 라이다인가**

- UGV와 파이프라인이 100% 같아진다: `Velodyne → pointcloud_to_laserscan → slam_toolbox`.
  뎁스는 Spot처럼 시야를 채우려면 5개 + `multi_scan_merger`가 필요해 로봇당 노드가 6개 는다.
- 라이다 1개로 360°. 뎁스는 하나당 FOV가 90° 남짓이라 제자리 요잉이 잦은 드론에 불리하다.

> 이후 7절(고도 회피)에서 이 파이프라인은 `drone_layer_mapper` 한 노드로 대체됐다.
> 라이다를 고른 판단은 그대로 유효하다 — 360°와 노드 수 이점이 더 커졌다.

**하향 뎁스센서 (나중에 추가)**

라이다의 가장 아래 광선은 −15°라 고도 `h`에서 바닥에 닿는 곳이 수평 `3.73h` 지점이다.
2 m로 날면 **반경 7.5 m짜리 장님 원반**이 드론 바로 아래에 생긴다(`minRange` 1 m까지
겹쳐 더 나쁘다). 층을 바꿔 넘어가려면 "지금 아래로 내려가도 되나"를 알아야 하는데,
그 근거가 정확히 이 원반 안에 있다. 그래서 `bodySlot`에 하향 `RangeFinder`를 달았다.

| | |
|---|---|
| 디바이스 이름 | `down_depth` |
| 장착 | base_link 아래 0.12 m, `rotation 0 1 0 1.5708` (+x → −z) |
| 해상도 / 시야 | 64×48 / 120° — 군집 대비해 일부러 작게 |

검증(고도 2 m, 발밑에 높이 1 m 상자): 깊이 범위 0.880~1.880 m,
상자 위 1134화소 / 바닥 1938화소 / **애매한 값 0개**. 센서 지상고 1.88 m와 정확히 맞는다.

> ⚠️ 반환값은 **광축 방향 평면 깊이**지 방사거리가 아니다. 아래가 평평하면 전 화소가
> 같은 값으로 읽히므로, 이걸 모르면 "고장"으로 오해한다.

**왜 질량을 주지 않았나**

VLP-16 실물은 0.83 kg이고 본체 `Physics.mass`는 2.8 kg이다. 켜면 +30%인데,
추력은 `ω²`에 비례하므로 호버 각속도는 `√m`으로 움직인다 — 아래 3절에서 맞춰 놓은
`K_VERTICAL_THRUST`(68.5)부터 다시 잡아야 하고, `centerOfMass`도 올라가 자세 게인까지
흔들린다. 매핑용 센서를 다는 것이 목적이라 기본은 무게 없는 센서로 뒀다.
탑재 중량 영향까지 보려면 `lidarPhysics TRUE`로 켜고 아래 4절의 헤드리스 하네스로
호버 추력을 다시 재면 된다.

**장착 높이 0.12 m의 근거 (헤드리스 검증)**

낮게 달면 아래쪽 광선이 기체를 스친다. 이 라이다는 `minRange`가 1 m라 기체가
"가짜 장애물"로 찍히지는 않지만 — Webots는 `minRange`보다 가까운 반사를 정확히 1.0으로
잘라 내보내므로 — **가려진 방향은 1 m 앞에 벽이 있는 것처럼 보이고 그 너머가 영영 안 보인다.**

수직 시야는 30°(±15°)다. 실측으로 확인했다(`wb_lidar_get_vertical_fov` = 0.5236).
가장 아래 광선은 반경 `r`에서 `0.268r`만큼 내려가므로:

| 기체 부위 | 반경 | 표면 z | 그 반경에서 광선 z | 여유 |
|---|---|---|---|---|
| 동체 뒤쪽 끝 | 0.27 | +0.031 | 0.048 | 17 mm |
| 랜딩기어 윗면 | 0.323 | −0.018 | 0.034 | 51 mm |
| 프로펠러 회전면 | 0.46 | −0.064 | −0.003 | 61 mm |

20×20 아레나 한가운데 2 m 고도에 수평으로 고정하고 전체 광선(3600×16 = 57600개)을 읽은 결과:

```
lidar: 3600 x 16 layers, fov=6.2832 vfov=0.5236 min=1.00 max=100.00
finite=34464  min=8.150  max=14.239  clamped_at_minrange=0
  layer  6..13  front range = 10.002 ~ 10.184   (10 m 벽)
  layer 14      front range = 9.457
  layer 15      front range = 8.159             (최하단 광선이 바닥에 닿음)
RESULT: PASS (self-occlusion 없음)
```

`min=8.150`은 최하단 광선이 바닥에 닿는 이론값 `2.12/sin15° = 8.19`와 일치하고,
`minRange`로 잘린 값이 **0개**다 — 자기 몸을 전혀 보지 않는다.
`layer 0~5`(위쪽)가 `inf`인 것도 맞다. 벽 높이가 3 m라 위로 올라간 광선이 벽을 넘어간다.

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

당시 `my_world.wbt`는 헤드리스로 못 돌렸다 — UGV·Spot이 월드에 `<extern>`으로 박혀
있어 Webots가 컨트롤러 연결을 기다리며 멈췄다. 그래서 스크래치패드에 **격리 프로젝트**를
만들었다.

> 지금은 월드에 로봇이 없고(소환기가 넣는다) 소환도 `synchronization FALSE`로 하므로
> 월드 자체는 헤드리스로 뜬다. 그래도 이 격리 하네스는 기체 단독 거동을 재는 데
> 여전히 제일 빠르다 — 노드가 적어 물리 스텝이 훨씬 빨리 돈다.

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
→ 속도는 적분항, 선회는 방위 적분 방식으로 해결 (2절 참고).

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

- **1 m 이내가 사각** — 라이다의 `minRange`가 1 m다. 좁은 복도나 벽 가까이에서는 벽이
  사라지므로 자율 비행의 안전 여유를 그 이상으로 잡아야 한다. 장애물 회피를 제대로
  하려면 근접/하향 센서를 더 달아야 하고, 래퍼 PROTO의 `extraBodySlot` 필드가 그 자리다.
- **맵이 비행 고도의 수평 단면** — 라이다 수직 시야가 ±15°라 한 번에 보이는 것은 그 고도
  주변뿐이다. 지금은 `drone_layer_mapper`가 **층(1/2/3 m)마다 격자를 따로 누적**해서
  이 성질을 다루지만(7절), 층 **사이** 높이의 장애물은 여전히 어느 층에도 제대로 안 찍힌다.
  다른 로봇이 보는 맵(`/{ns}/map`)은 그 층들의 합집합이라 지상 로봇 기준 높이가 아니다.
- **연속적인 3D 경로계획은 여전히 없다.** 지금은 **2.5D 레이어드**(경로 1)다 — 고도를
  1/2/3 m 이산 층으로 두고 층을 고른다. 상승과 수평이동이 섞이지 않고 순차로 일어나며,
  층 사이 높이의 장애물은 표현되지 않는다. 구조와 실측은 7절에 있다.
  아래는 왜 연속 3D(경로 2)로 가지 않았는지의 배경이다.

  원인은 표현 자체에 있다. Nav2 플래너는 `nav_msgs/OccupancyGrid`, 즉 z 축이 없는 2D
  격자 위에서 돈다. 그 격자를 만드는 `/{ns}/map` 도 비행 고도의 수평 단면이다.
  그래서 "장애물 위로 넘어간다 / 아래로 지나간다" 는 경로는 원리적으로 나올 수 없다.

  3D 로 가려면 세 층이 다 필요하다.

  | 층 | 지금 | 3D 로 가려면 |
  |---|---|---|
  | 지도 | 2D OccupancyGrid 층 3장 (`drone_layer_mapper`) | 3D 점유 지도 — `octomap_server` (의존성은 Dockerfile 에 이미 있다) |
  | 플래너 | Nav2 NavFn (2D) | Nav2 에는 없다. MoveIt+OMPL 이나 UAV 전용 플래너 |
  | 컨트롤러 | `linear.x`/`angular.z` 만 | `linear.z` 까지 — **드라이버는 이미 지원한다** |

  라이다의 수직 시야가 ±15° 뿐이라 3D 지도를 만들려면 센서도 보강해야 한다.
- **속도 제어이지 위치 제어가 아님** — `cmd_vel`이 0이면 속도 0을 유지하지만, 외란에 밀린 뒤
  원위치로 돌아가지는 않는다. 웨이포인트 비행에는 위치 루프가 필요하며, Nav2가 그 역할을 한다.
- **짐벌이 각속도 댐핑만 함** — 순항 중 기체가 5~15° 기울면 카메라도 같이 기운다.
  정찰 용도로는 자세 자체를 상쇄하도록(`-roll`/`-pitch`) 바꾸는 편이 낫다.
- **시뮬레이션이 실시간의 약 27%로 동작** (컨테이너 4개 + GUI 렌더링). 테스트 시 감안할 것.

### 폐기된 것

초기에는 Webots 내장 C 컨트롤러(`controllers/mavic2pro_medium/`)로 키보드 조종을 했으나,
**OS별 컴파일이 필요해 이식성이 없고 ROS 2 미션 스택에 붙일 수 없어** 폐기했다.
5절의 이슈 ①~③은 그 컨트롤러로 규명한 것이며 결론은 그대로 유효하다.
복원이 필요하면 `git log -- workspace/simulator/controllers/`.

---

## 7. 고도 회피 — 2.5D 레이어드 (경로 1)

> 📘 **직접 돌려보는 방법과 전체 구조는 [09_DRONE_NAV.md](09_DRONE_NAV.md)에 따로 정리했다.**
> 이 장은 그중 "왜 이렇게 만들었나"의 요약이다.
>
> 이후 여기에 **지역(local) 고도 회피**가 더해졌다 — 주행 중 앞이 막히면 실시간으로
> 넘어가고 지나면 되돌아온다. 그쪽은 [09장](09_DRONE_NAV.md) 1·4절에 있다.

Nav2 는 2D 플래너라 고도를 계획하지 않는다. **그 한 축만 바깥에서** 담당해
"장애물을 넘어간다" 를 얻는다. Nav2 자체는 한 줄도 고치지 않는다
(`map_topic` 인자 하나만 열었다).

### 왜 연속 3D(경로 2)가 아닌가

3D 플래너는 **계획 호출마다** 3D 탐색을 한다. 군집으로 가면 그 비용이 대수만큼
곱해진다. 반면 이 구조는

- 계획은 지금과 **똑같은 2D A***
- 늘어나는 비용은 층당 직선 회랑 검사(수백 칸)뿐
- 드론 1대당 노드가 **2개 → 1개로 줄었다**
  (`pointcloud_to_laserscan` + `slam_toolbox` → `drone_layer_mapper`)

### 2D 지도를 어떻게 나눴나 — 함정과 해법

층을 여러 개 쌓으면 `/{ns}/map` 하나에 여러 고도가 섞인다. 그런데 그 토픽을
**두 소비자가 동시에** 본다 — 맵 병합기와 **드론 자신의 Nav2 static layer**.

합집합을 그대로 주면 드론이 **다른 고도의 장애물 때문에 지금 고도에서는 뻥 뚫린
공간을 못 지나간다.** 고도 회피를 하려고 만든 기능이 오히려 지금보다 나빠진다.
그래서 토픽 역할을 셋으로 갈랐다.

| 토픽 | 내용 | 소비자 | 근거 |
|---|---|---|---|
| `/{ns}/map` | 층 **합집합** | 맵 병합기·관제 | 병합 규칙이 `np.maximum`(장애물 OR)이라 드론이 3 m 에서 본 "빈 곳"은 UGV 가 0.8 m 에서 본 책상에 어차피 진다 — **아무것도 지우지 않는다.** 이름을 유지하므로 `map_topic_pattern` 에 그대로 걸려 **병합기는 무수정** |
| `/{ns}/map_active` | 현재 순항 고도 한 층 | 드론 Nav2 | 플래너는 자기가 나는 층만 봐야 한다 |
| `/{ns}/map_layer_k` | 후보 층 | `altitude_selector` | 어느 층이 열렸는지 판단 |

### 왜 slam_toolbox 를 뺐나

드라이버가 GPS 절대좌표를 그대로 odom 으로 발행하므로 자세가 이미 정답값이다.
slam_toolbox 는 사실상 점유 격자 누적기로만 쓰이고 있었다. 맵 병합이 이미
`odom_is_world_absolute: true` 로 같은 가정 위에 서 있으므로 새 가정도 아니다.
`{ns}/map → {ns}/odom` 은 항등이라 static TF 하나로 대체했다.

  대가: 루프 클로저·드리프트 보정이 없다. 실기 이식 때는 3D SLAM 이 필요하다.

### 실측

| 상황 | 층 판정 | 결과 |
|---|---|---|
| 현재 층 막힘, 위가 열림 | `1m:X(장애물22.3%) 2m:X(3.2%) 3m:OK(0.0%)` | 3.0 m 로 **상승**, 고도 2.00→2.97 (오차 0.03 m) |
| 현재 층 막힘, 아래가 열림 | `1m:OK(0.5%) 2m:X(5.9%) 3m:X` | 1.0 m 로 **하강**, 목표까지 3.00→0.60 m |
| 현재 층 열림 | `2m:OK(0.0%)` | 고도 유지, 3.1 m 주행 |

층별 지도가 실제로 갈리는 것도 확인했다 — 같은 순간에
층0(1 m) 장애물 1371칸 / 층1(2 m) 933칸 / 층2(3 m) 325칸,
층0과 층1이 **3224칸** 다르고, 합집합(1948칸)은 모든 층의 상위집합이었다.

### 겪은 함정 세 가지 (전부 실측으로 잡았다)

**① 하향 센서는 회전해서 달려 있다.** PROTO 의 `rotation 0 1 0 1.5708` 이 센서의
+x(시선)를 기체의 -z(아래)로 보낸다. 이걸 빠뜨리고 점을 그대로 기체 좌표로 쓰면
**발밑 바닥이 "정면 1.9 m 앞의 벽"으로 찍힌다.** 층 지도마다 유령 장애물이 생겨
`linear.x 0.5` 를 15초 줘도 0.33 m 밖에 못 갔다 (정상은 1.8 m — 시뮬이 실시간의
23% 로 도는 것을 감안한 값이다). 올바른 변환은 `(sx,sy,sz) -> (sz, sy, -sx)`.

**② 회랑 검사를 하드 실패로 두면 아무 층도 안 뚫린다.** 처음엔 "장애물이나 미탐색이
한 칸이라도 있으면 탈락" 이었는데 **108개 방향 중 뚫린 곳이 0개**였다. 지도를 그려
보니 드론 오른쪽은 5 m 넘게 비어 있었는데 1.2 m 옆 가구 한 덩이 때문에 전부
탈락한 것이다. 이 검사의 목적은 "직선이 완벽히 비었나" 가 아니라 **"어느 층이 더
열려 있나"** 이므로 비율 기준(장애물 2%, 미탐색 35%)으로 바꾸고, 전부 탈락하면
**가장 덜 막힌 층**을 고르게 했다. 실제 회피는 Nav2 코스트맵이 한다.

**③ 속도 명령이 위치 목표를 적분하는 계에 실제 위치로 피드백을 걸면 반드시
오버슈트한다.** 드라이버는 `target_altitude += linear.z * dt` 로 목표 고도를 만든다.
실제 고도가 허용오차에 들어왔을 때 `target_altitude` 는 이미 한참 지나가 있어서,
명령을 끊어도 기체는 그 target 까지 계속 간다 — **2.0 m → 1.0 m 를 지시했는데
0.30 m(MIN_ALTITUDE)까지 떨어졌다.** 그래서 선택기가 자기가 보낸 명령을 같은 식으로
적분해 `target_altitude` 를 추정하고, **그 추정값** 기준으로 멈춘다. 수정 후 오차 0.03 m.

### 남은 한계

- **이산 층이지 연속 3D 가 아니다.** 상승과 수평이동이 섞이지 않고 순차로 일어나며,
  층 사이 높이의 장애물은 표현되지 않는다.
- **회랑 검사는 직선만 본다.** 직선은 막혔지만 우회로가 있는 층을 놓친다(보수적).
  놓쳐도 Nav2 가 현재 층에서 우회를 시도하므로 기능이 깨지지는 않는다.
- 천장 쪽은 여전히 사각이다. 라이다 ±15° 와 하향 센서로는 위를 못 본다.

---

## 8. 파일 맵

| 파일 | 역할 |
|---|---|
| `src/Webots-SummitXL/workspace/simulator/protos/Mavic2ProMedium.proto` | 개조 PROTO (기체) |
| `.../protos/Mavic2ProMediumSensorized.proto` | 래퍼 PROTO (기체 + 라이다) — **소환은 이쪽** |
| `.../protos/VelodyneVLP-16.proto` | 라이다 (UGV와 공용) |
| `.../protos/Mavic2Pro/` | 메시 14개 + 텍스처 (로컬 포함) |
| `.../simulator/simulator/drone_driver.py` | webots_ros2 플러그인 (2단 제어 + odom/TF) |
| `.../simulator/simulator/drone_teleop.py` | 키보드 조종 (고도 축 있음) |
| `src/webots_robot_spawner/config/fleet/*.yaml` | 드론의 스폰 좌표 (월드에 인스턴스를 박아 두지 않는다) |
| `src/webots_python/urdf/Mavic2ProMedium.urdf.xacro` | 플러그인 연결 + 디바이스 매핑 |
| `src/webots_python/launch/single_drone.launch.py` | 런치 (`ROBOT_ID` 방식) |
| `src/webots_python/webots_python/drone_layer_mapper.py` | 층별 지도 (slam_toolbox 대체) — 7절 |
| `src/webots_python/webots_python/altitude_selector.py` | 층 선택 + 순항 고도 결정 — 7절 |
| `src/webots_python/webots_python/local_altitude_avoider.py` | **지역 고도 회피** (cmd_vel 단독 발행) — [09_DRONE_NAV.md](09_DRONE_NAV.md) |
| `.../navigation/launch/nav2.launch.py` | `map_topic` 인자 (드론만 `map_active`) |
| `docker-configs/*/docker-compose.yml` | `drone1` 서비스 |

### 관련 문서

- [09_DRONE_NAV.md](09_DRONE_NAV.md) — **자율비행 구조 + 직접 테스트하는 법**
- [Readme 11절](Readme.md#11-drone-중형급-쿼드콥터) — 빠른 사용법
- [01_INTERFACES.md](01_INTERFACES.md) — 세 로봇의 `cmd_vel` 의미 차이 표
- [03_SPAWNER.md](03_SPAWNER.md) — 드론만 `synchronization TRUE`로 되돌리는 이유와 기동 순서 교착
- [10_MAP_MERGE.md](10_MAP_MERGE.md) — 드론 맵이 병합에 들어가는 방법
- [04_UGV_SETUP.md](04_UGV_SETUP.md) — 비교 대상인 기준 로봇

---

← [07. Spot 자율주행](07_SPOT_NAV.md) | [📖 책 목차](Readme.md#-목차) | [09. 드론 자율비행](09_DRONE_NAV.md) →
