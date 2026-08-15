# Spot 자율주행 튜닝 기록

Spot의 Nav2 자율주행을 손보면서 겪은 것 전부 — **무엇을 바꿨고, 어디서 막혔고,
왜 그랬는지**. 값만 보려면 [nav2_spot.yaml](src/Webots-SummitXL/workspace/navigation/param/nav2_spot.yaml)
머리말에 표가 있고, 여기는 그 근거와 시행착오를 남긴다.

기체·드라이버 함수 설명은 [spot_driver_functions.md](spot_driver_functions.md),
드론 쪽 같은 작업은 [DRONE_NAV.md](DRONE_NAV.md)에 있다.

## 목차
- [1. 요약](#1-요약)
- [2. 적용한 변경](#2-적용한-변경)
- [3. 이슈와 해결 — 코드·설정](#3-이슈와-해결--코드설정)
- [4. 이슈와 해결 — 측정 방법론](#4-이슈와-해결--측정-방법론)
- [5. 미해결 / 다음 작업](#5-미해결--다음-작업)
- [6. 재측정하는 법](#6-재측정하는-법)

---

## 1. 요약

시작할 때 Spot은 **UGV와 같은 Nav2 파라미터**를 쓰고 있었고, 목표 도달률이 0~1/3에
배회(주행거리÷직선거리)가 2~5배였다. 세 층에서 문제가 있었다.

| 층 | 문제 | 상태 |
|---|---|---|
| Nav2 공통 | 리커버리가 통째로 죽어 있었다 (네임스페이스 누락) | ✅ 해결 |
| Spot 드라이버 | `cmd_vel`이 m/s가 아니라 **보폭 배율**이었다 | ✅ 해결 |
| Nav2 파라미터 | 속도·가속·footprint가 지상차량 값 | ✅ 해결 |
| 보행 속도 | 보폭만 만지고 **케이던스는 손도 안 댐** | ⚠️ 부분 (아래 5장) |
| 센서 | 스캔이 자기 몸 끝을 못 본다 | ⚠️ 완화만 |

목표 도달률은 **2/3**까지, 배회는 **1.0~2.0배**까지 개선됐다.
다만 **절대 속도값은 측정 오류로 신뢰할 수 없다** ([4장 ①](#-속도를-벽시계로-쟀다)).

---

## 2. 적용한 변경

### 파일

| 파일 | 무엇 |
|---|---|
| `spot_driver.py` | `cmd_vel` 단위 환산, 보행 파라미터 런타임 노출, 보행 정지 버그 수정 |
| `navigation/param/nav2_spot.yaml` | **신규.** Spot 전용 Nav2 파라미터 (UGV·드론은 `nav2.yaml` 그대로) |
| `navigation/launch/nav2.launch.py` | `params_file`·`map_topic`·`cmd_vel_topic` 인자, behavior_server 프레임 수정 |
| `single_spot_launch.py` | `params_file`로 `nav2_spot.yaml` 지정 |
| `navigation/setup.py` | `nav2_spot.yaml` 설치 |

### 주요 값

| 항목 | UGV | Spot | 왜 |
|---|---|---|---|
| 컨트롤러 | DWB | **RPP** | DWB는 속도 샘플링이라 로봇 모델 오차에 흔들린다 |
| `footprint` | 0.7 × 0.5 | **1.1 × 0.5** | 몸통 실측. **폭은 건드리지 말 것** ([3장 ④](#-footprint-폭을-키웠더니-경로가-아예-안-나왔다)) |
| `inflation_radius` / `cost_scaling` | 0.6 / 3.0 | **0.85 / 2.0** | 스캔 사각 보완 |
| 리커버리 회전 | 1.0 rad/s | **0.26** | 낼 수 없는 속도를 목표로 삼으면 리커버리가 실패 |
| 보행 `swing_period` | — | **0.12** | 케이던스. 런타임 변경 가능 |
| 보행 `step_velocity` | — | **3.0** | 위와 같음 |

---

## 3. 이슈와 해결 — 코드·설정

### ① 리커버리가 통째로 죽어 있었다

**증상** — 4 m 목표에 1.43 m 남기고 `ABORTED`. 로그:

```
[transformPoseInTargetFrame] target frame "odom" does not exist
[behavior_server] Initial checks failed for spin -> Aborting handle
[bt_navigator] Goal failed
```

**원인** — `nav2.yaml`에 `local_frame: odom`이라 적혀 있었는데, Humble의
`nav2_behaviors`에는 **그런 파라미터가 없다.** 실제 이름은 `global_frame`이고,
yaml에 없으니 Nav2 기본값(네임스페이스 없는 `odom`)이 그대로 쓰였다.

```
local_frame       = Parameter not set        <- 존재하지 않는 파라미터
global_frame      = odom                     <- 네임스페이스 누락
robot_base_frame  = ugv1/base_link           <- 이건 정상
```

**헷갈리는 지점** — **주행 자체는 멀쩡한데 리커버리만 죽는다.** 평소엔 잘 가다가
BT가 리커버리를 한 번이라도 타는 순간 목표가 통째로 실패한다.

**해결** — `nav2.launch.py`에서 behavior_server 노드에만 `{ns}/odom`으로 덮어쓴다.
(`global_frame` 치환값은 `{ns}/map`이라 이 노드엔 맞지 않는다. local_costmap과 같은 예외)

### ② 리커버리 회전 한계가 무시되고 있었다

**증상** — `Exceeded time allowance before reaching the Spin goal`이 반복.

**원인** — `spin:` 아래에 중첩해서 적었는데 Nav2는 **노드 레벨**에서 읽는다.
설정 0.26이 무시되고 기본값 1.0이 쓰였다. Spot 최대가 0.263 rad/s라 영원히 도달 못 한다.

```yaml
behavior_server:
  ros__parameters:
    max_rotational_vel: 0.26     # 여기 (노드 레벨) — 맞다
    spin:
      max_rotational_vel: 0.26   # 여기 — 조용히 무시된다
```

**해결** — 노드 레벨로 이동. ①과 **같은 부류의 함정**이다.

### ③ `cmd_vel`이 m/s가 아니었다

**증상** — 경로를 못 따라가고 배회가 직선거리의 2~5배.

**원인** — `StepLength = 0.15 × linear.x`, 즉 **보폭 배율**이었다. 그런데 Nav2의 DWB는
`linear.x`를 m/s로 알고 궤적을 예측한다. **컨트롤러의 세계 모형이 로봇과 어긋난 채로 돌았다.**

측정해 보니 회전이 특히 심했다 — **실제가 예측의 0.53배로 고정**. 상한을 어떻게 잡아도
이 비율은 안 바뀌므로 **파라미터로는 못 고친다.**

**해결** — 포화 지점에서 역산해 환산 계수를 넣었다.

```python
StepLength = (|v| / SPEED_COEF) ** (1/SPEED_EXP)   # v[m/s]
YawRate    = YAW_PER_RADPS * angular.z             # [rad/s]
```

수정 후 회전 오차 −47% → **+3~7%**. 목표 오차도 1.96 m → **0.45~0.50 m**(허용치 0.5 안).

### ④ footprint 폭을 키웠더니 경로가 아예 안 나왔다

**증상** — `GridBased: failed to create plan`. 전역 플래너가 경로를 못 만든다.

**원인** — Spot이 1.1 m로 길어서 footprint를 1.2 × **0.6**으로 키웠는데,
**NavFn은 내접 반경 안쪽을 통행 불가로 본다.**

| footprint | 내접 반경 | 장애물마다 막히는 폭 |
|---|---|---|
| 0.7 × 0.5 (UGV) | 0.25 | 0.25 m |
| 1.2 × **0.6** | **0.30** | 0.30 m |

가구 사이가 몸통 폭 정도면 양쪽에서 0.30씩 막혀 통과 가능 폭이 사라진다.

**해결** — **길이만 늘리고 폭은 UGV와 같게**(1.1 × 0.5). Spot의 다리는 주로 앞뒤로
스윙하지 좌우로 몸통보다 넓어지지 않는다.

### ⑤ 저속에서 33% 초과속

**원인** — 보폭↔속도가 선형이 아니다. 보폭이 작을수록 효율이 높다.

| StepLength | 0.015 | 0.030 | 0.050 |
|---|---|---|---|
| m/s ÷ L | 6.4 | 5.6 | 5.1 |

포화점에 맞춘 선형 계수 하나로는 저속을 못 맞춘다.

**해결** — 로그-로그 회귀로 지수를 맞춘 거듭제곱 환산. 저속 오차 33% → **2%**.

### ⑥ 저속에서 보행이 아예 안 걸렸다

**증상** — 명령 0.20 / 0.35 m/s에서 **이동 0.000 m, roll 정확히 0.0°**. 아예 안 움직인다.

**원인** — `Bezier.py`:

```
Tstance = 2L / StepVelocity
if Tstride < Tswing + dt:  RESET ALL     <- 보행이 진행 못 함
```

케이던스를 올리려고 `StepVelocity`를 3.0으로 **상수** 지정했더니, 보폭이 작을 때
`Tstance`가 dt(32 ms)보다 작아져 매 스텝 리셋됐다.

**해결** — 보폭에 맞춰 `StepVelocity`를 깎아 `Tstance ≥ 2·dt`를 보장한다.
큰 보폭에서는 원래 값을 그대로 쓰므로 최고 속도 손해가 없다.

### ⑦ 속도를 올리려면 보폭이 아니라 케이던스다

**증상** — 보폭만 키우니 0.40 m/s 상당에서 균형이 무너져(roll 4° → 17°) 더 못 올림.

**원인** — 속도를 정하는 것이 셋인데 하나만 만졌다.

```
Tstance = 2L / StepVelocity      Tstride = Tstance + SwingPeriod
속도    = 2L / Tstride
```

`SwingPeriod`(0.3)와 `StepVelocity`(0.8)는 **하드코딩된 채 손도 안 댔다.**

**해결** — 둘을 ROS 파라미터로 노출해 런타임 튜닝 가능하게 했다.
케이던스를 함께 올리자 같은 자세 안정성(roll 4°대)에서 **속도가 2.3배**가 됐다.

```bash
ros2 param set /spot1/spot_driver swing_period 0.15
ros2 param set /spot1/spot_driver step_velocity 2.0
```

### ⑧ 스캔이 자기 몸 끝을 못 본다

**증상** — 장애물에 접촉.

**원인** — 병합 스캔의 `range_min`이 **0.5 m**인데 footprint 반길이는 **0.55 m**다.
0.5 m 안쪽은 코스트맵에 아예 안 들어오고, 컨트롤러는 비어 있는 줄 알고 명령을 낸다.

**⚠️ 인플레이션만으로는 못 막는다.** 치명 판정은 footprint 내접 반경(0.25)이 정하고,
`inflation_radius`는 그 바깥의 비용 기울기일 뿐이다.

**완화** — 반경을 넓히고 감쇠를 늦춰 0.5 m 지점 비용을 올렸다.

```
cost = 253 * exp(-scaling * (거리 - 내접반경))
0.5 m 에서:   3.0/0.6 -> 119      2.0/0.85 -> 154
```

**근본 해결은 안 했다.** `range_min`을 낮추거나 근접 센서를 달아야 한다 ([5장](#5-미해결--다음-작업)).

---

## 4. 이슈와 해결 — 측정 방법론

**이 장이 실제로 제일 많은 시간을 잡아먹었다.** 코드가 아니라 재는 방법이 틀려서
결과를 여러 번 날렸다.

### 🚨 속도를 벽시계로 쟀다

**가장 큰 실수다.** 시뮬 속도를 **23% 고정**으로 가정하고 계산했다.

```python
act = 거리 / (벽시계시간 * 0.23)      # 틀렸다
```

그런데 시뮬 속도는 **컨테이너 수와 부하에 따라 23~90%로 변한다**(실측). 컨테이너 하나를
내리는 것만으로 4배가 바뀐다. **그래서 보고한 절대 속도값들이 최대 4배까지 틀렸다.**

**올바른 방법 — odom 헤더의 시뮬 시각을 쓴다.**

```python
def stamp():
    h = odom.header.stamp
    return h.sec + h.nanosec * 1e-9

dt = stamp_끝 - stamp_시작        # 벽시계가 아니다
v  = 이동거리 / dt
```

> 자세(roll/pitch) 기반 **안정성 판정과 항목 간 상대 비교는 유효하다** — 같은 조건에서
> 연속 측정했고, 각도는 시간 척도와 무관하기 때문이다.

### 테스트 노드에 `use_sim_time`을 안 켰다

**증상** — 목표는 수락되는데 **로봇이 한 발도 안 뗀다.**

```
Extrapolation Error: Requested time 2982.516 but the earliest data is at time 3364.256
```

목표 스탬프가 벽시계라 Nav2가 수백 초 과거로 TF를 조회한다. **이것 때문에 세 번을 헛돌았다.**

```python
n = Node('test', parameter_overrides=[Parameter('use_sim_time', value=True)])
```

> `ros2 topic pub`은 스탬프가 0이라 "최신"으로 처리돼 통과한다. 액션으로 스크립트를
> 짤 때만 문제가 된다.

### 목표를 아무 데나 찍었다

`현재위치 + (5, 3)` 같은 상대 좌표를 그대로 썼더니 **월드 밖**(x 최대 9.5인데 11.34)이나
벽 속을 찍었다. 그런 목표는 당연히 실패하는데 원인을 파라미터에서 찾느라 시간을 썼다.

**해결** — 목표를 맵에서 고른다.

- 주변 1.1 m에 **장애물 한 칸도 없을 것**(하드)
- 목표 자리 자체는 관측된 빈 곳, 주변 **55% 이상 관측**
  (100% 관측을 요구하면 후보가 0개가 된다 — `slam_toolbox` 맵은 지나온 자리 위주다)
- 로봇→목표 **직선 회랑(반폭 0.8 m)에 장애물 없음**
- **월드 경계 안**

### 디버깅한다고 `cmd_vel`을 오래 줬다가 월드 밖으로 밀어냈다

"물리적으로 낀 건지" 확인한다고 `linear.x`를 20초 직접 줬더니 Spot이 x=10.5(월드 밖)로
나가 벽 위에 올라앉았다. 그 뒤 **모든 목표가 실패**했고, 원인을 한참 지도에서 찾았다.

**짧게만 준다.** 그리고 측정 전에 항상 월드 안인지 확인한다.

### drone1을 내렸더니 시뮬 전체가 멈췄다

Spot만 테스트한다고 다른 로봇을 내렸는데 `/spot1/odom`이 0건으로 끊겼다.
**드론은 `synchronization TRUE`라 Webots가 매 스텝 드론 컨트롤러를 기다린다.**

부하를 줄이려면 **ugv2를 내리고 drone1·ugv1은 살려 둔다**(`ugv1`은 `/clock` 발행자).

### 로봇이 계속 장애물에 끼어 측정을 날렸다

이 세션에서 측정을 대여섯 번 날린 원인의 대부분이 "정면 0.54 m에 장애물" 또는
"월드 밖"이었다. 벽을 밀고 있으면 속도가 0으로 나와 **파라미터가 잘못된 것처럼 보인다.**

**측정 전에 반드시 확인한다.**

```python
front = min(정면 ±15° 스캔)     # 2.5 m 이상이어야 전진 측정이 유효
```

---

## 5. 미해결 / 다음 작업

### 절대 속도 재측정 (최우선)

현재 `desired_linear_vel: 0.55`가 **실제로 몇 m/s인지 확신할 수 없다.**
[4장 ①](#-속도를-벽시계로-쟀다)의 방법으로 다시 재고 `SPEED_COEF`와
`desired_linear_vel`을 확정해야 한다. 조건:

- Spot을 **직진 5~6 m 가능한 트인 곳**에 놓을 것
- 측정 중 **컨테이너 구성을 바꾸지 말 것**(시뮬 속도가 변한다)

### 실제 BD Spot 스펙(순항 1.3 m/s)은 현재 구조로 도달 불가

케이던스를 올릴수록 이론 대비 효율이 떨어진다(106% → 53%). 자세는 멀쩡한데 **발이
미끄러진다.** 원인은 균형이 아니라 **시간 해상도**다 — 월드에 `basicTimeStep`이 없어
Webots 기본값 32 ms로 도는데, `swing=0.10`이면 스윙 한 번이 **3 스텝**뿐이다.

| 선택지 | 기대 | 대가 |
|---|---|---|
| 현행 유지 | 케이던스 튜닝분(약 2.3배) | 없음 |
| `basicTimeStep` 16 | 더 높은 케이던스 | 시뮬 **2배 느려짐** |
| `basicTimeStep` 8 | 1 m/s대 도전 | 시뮬 **4배 느려짐** |

**월드 전체(드론·UGV 포함)에 영향을 주는 결정**이라 임의로 바꾸지 않았다.

### 스캔 사각 (0.5 m)

[3장 ⑧](#-스캔이-자기-몸-끝을-못-본다) 참고. 인플레이션으로 완화만 했다. 근본 해결은
`range_min`을 낮추거나(뎁스 최소 거리가 허용하면) 근접 센서를 다는 것이다.

### 배회 1~2배

DWB→RPP로 2~5배에서 줄었지만 여전히 직선거리의 1~2배다. RPP의 `lookahead_dist`
튜닝 여지가 남아 있다.

---

## 6. 재측정하는 법

절차와 스크립트는 [spot_driver_functions.md](spot_driver_functions.md#자율주행-직접-테스트하기)에
있다. 순서만 요약하면:

1. **Spot을 트인 곳에 놓는다** — 정면 여유 2.5 m 이상 확인
2. **컨테이너 구성을 고정한다** — 측정 중 켜고 끄지 않는다
3. `use_sim_time=True`로 테스트 노드를 만든다
4. **시뮬 시각(odom stamp)으로** 구간을 잰다
5. 자세(roll/pitch)와 몸높이(z)를 함께 기록한다 — 안정성 판정용

```bash
# 보행 파라미터는 런타임에 바꿀 수 있다 (재빌드 불필요)
ros2 param set /spot1/spot_driver swing_period 0.15
ros2 param set /spot1/spot_driver step_velocity 2.0
ros2 param set /spot1/spot_driver max_step_length 0.07
```

---

### 관련 문서

- [spot_driver_functions.md](spot_driver_functions.md) — 드라이버 함수와 `cmd_vel` 단위
- [nav2_spot.yaml](src/Webots-SummitXL/workspace/navigation/param/nav2_spot.yaml) — 값과 근거표
- [DRONE_NAV.md](DRONE_NAV.md) — 드론 쪽 같은 작업
- [INTERFACES.md](INTERFACES.md) — 토픽·프레임 규격
