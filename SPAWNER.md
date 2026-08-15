# 로봇 소환 (Runtime Spawn) 구축 기록

실행 중인 Webots에 로봇을 **월드 편집 없이** 추가하고, 편대 전체를 yaml 한 장으로
정의하기까지의 전 과정 기록.

빠른 사용법은 [Readme 12장](Readme.md#12-로봇-소환-runtime-spawn)에 있고,
이 문서는 **왜 그렇게 만들었는지**와 **어떤 함정이 있었는지**를 다룬다.

- 담당 패키지: [src/webots_robot_spawner/](src/webots_robot_spawner/) +
  [src/webots_spawner_msgs/](src/webots_spawner_msgs/)
- 서비스: **`/spawn_robot`** (`webots_spawner_msgs/srv/SpawnRobot`)

## 목차
- [1. 문제 정의](#1-문제-정의)
- [2. 세 층 구조 — 몸 / 뇌 / 컨테이너](#2-세-층-구조--몸--뇌--컨테이너)
- [3. 로봇 종류 정의표](#3-로봇-종류-정의표)
- [4. 소환 한 번에 일어나는 일](#4-소환-한-번에-일어나는-일)
- [5. 빈 자리 고르기](#5-빈-자리-고르기)
- [6. 편대 매니페스트와 compose 생성](#6-편대-매니페스트와-compose-생성)
- [7. 기동 순서 (왜 드론만 다른가)](#7-기동-순서-왜-드론만-다른가)
- [8. 잔여 몸 정책 (stale_body_policy)](#8-잔여-몸-정책-stale_body_policy)
- [9. 파라미터 표](#9-파라미터-표)
- [10. 트러블슈팅](#10-트러블슈팅)
- [11. 새 로봇 종류를 추가하려면](#11-새-로봇-종류를-추가하려면)
- [12. 파일 맵](#12-파일-맵)

---

## 1. 문제 정의

예전에는 로봇 4대가 `my_world.wbt`에 박혀 있었다. 한 대를 늘리려면

1. 월드 파일에 인스턴스를 손으로 추가하고 (센서 블록까지 인라인으로)
2. `docker-compose.yml`에 서비스를 하나 더 쓰고
3. Webots를 재시작해서 월드를 다시 읽혀야 했다

셋 다 사람이 하는 일이라 어긋나기 쉬웠고, 무엇보다 **시뮬레이션을 멈춰야** 했다.
"로봇이 계속 추가되는 상황"을 다루려면 이 셋이 전부 없어져야 한다.

지금은 `my_world.wbt`에 로봇이 하나도 없다. 환경(아레나·벽·가구)과 소환 전담 노드
`spawn_supervisor` 하나만 있고, 로봇은 전부 소환으로 들어온다.

```bash
ros2 service call /spawn_robot webots_spawner_msgs/srv/SpawnRobot "{type: 'ugv', random: true}"
```

---

## 2. 세 층 구조 — 몸 / 뇌 / 컨테이너

**몸과 뇌는 1:1이어야 하지만, 뇌와 컨테이너는 그럴 이유가 없다.**
이 구분이 구조의 핵심이다.

```
[Webots — 호스트]                  [fleet 컨테이너]        [로봇별 컨테이너]

spawn_supervisor (유령 로봇) ←TCP→  spawn_supervisor
  supervisor TRUE                    ├─ /spawn_robot 서비스
  synchronization FALSE              ├─ 매니페스트대로 몸 주입
                                     └─ /map_merged 로 빈 자리 찾기
ugv1   (몸)  ←────────────────────────────────────TCP────→ ugv1_brain_*
spot1  (몸)  ←────────────────────────────────────TCP────→ spot1_brain_*    driver
drone1 (몸)  ←────────────────────────────────────TCP────→ drone1_brain_*   + SLAM
                                                                            + Nav2
ugv3 (런타임 소환된 몸) ←TCP→ 이 뇌만 fleet 컨테이너가 띄운다               + registrar
```

- **몸**은 fleet 컨테이너의 소환기가 매니페스트대로 Webots에 주입한다
- **뇌(제어·경로)** 는 로봇별 컨테이너가 담당한다. 컨테이너 경계가 있어야
  `cpuset`으로 코어를 고정하거나 `cpus`로 상한을 걸 수 있다
- **매니페스트에 없는 런타임 소환**만 fleet 컨테이너가 뇌까지 띄운다
  ([brain_launcher.py](src/webots_robot_spawner/webots_robot_spawner/brain_launcher.py)의
  `LocalProcessLauncher` — `ros2 launch`를 자식 프로세스로 돌린다).
  이름과 자리가 런타임에 정해져 기다려 줄 컨테이너가 없기 때문이다.
  그 로그는 fleet 컨테이너 안 `/tmp/spawned_robots/{robot_id}.log`

> ⚠️ 컨테이너를 나눠도 **CPU 총량이 늘지는 않는다.** compose에 `cpus`/`memory` 제한이
> 없으면 컨테이너는 호스트 코어를 그냥 공유하고, OS는 컨테이너가 아니라 프로세스를
> 스케줄한다. 컨테이너 경계가 주는 것은 **코어 고정·자원 상한·장애 격리·개별 재시작**이다.

**Docker 소켓을 쓰는 방식(컨테이너가 컨테이너를 띄우는 방식)은 일부러 만들지 않았다.**
소켓 경로가 플랫폼마다 달라(Linux 유닉스 소켓 / Windows 네임드 파이프 / Mac Desktop VM)
크로스 플랫폼 전제가 깨진다. 필요해지면 `brain_launcher.py`의 인터페이스에 클래스만
더하면 된다.

---

## 3. 로봇 종류 정의표

로봇 하나를 지원한다는 건 결국 두 가지를 아는 것이다 — **씬 트리에 꽂을 노드 문자열(몸)**과
**거기 붙일 ROS 2 런치(뇌)**. [robot_types.py](src/webots_robot_spawner/webots_robot_spawner/robot_types.py)가
그 둘을 종류마다 한 줄로 묶어 둔다.

| | `ugv` | `spot` | `drone` |
|---|---|---|---|
| PROTO | `SummitXlSteelSensorized` | `SpotSensorized` | `Mavic2ProMediumSensorized` |
| 스폰 높이 `spawn_z` | 0.12 m | 0.624 m | 0.13 m |
| 몸통 반경 | 0.47 m (0.72×0.61) | 0.6 m (1.1×0.5) | 0.35 m (대각 ~0.7) |
| 기본 여유 `default_clearance` | 0.75 m | 0.9 m | 0.6 m |
| 뇌 런치 | `webots_python/single_ugv.launch.py` | `webots_spot/single_spot_launch.py` | `webots_python/single_drone.launch.py` |
| SLAM 맵 `has_map` | ✅ | ✅ | ✅ (비행 고도의 단면) |
| `DEF` 필요 `needs_def` | | ✅ | |
| 동기화 필요 `needs_sync` | | | ✅ |

**PROTO가 왜 "Sensorized" 래퍼인가** — UGV와 Spot은 센서가 월드 파일에 인라인으로
박혀 있었다(UGV `bodySlot` 85줄, Spot `middleExtension` 40줄). 그 상태로는 다섯 줄짜리
스폰 문자열을 만들 수 없어서, 센서를 통째로 품은 래퍼 PROTO를 만들었다.

드론은 원래 `Mavic2ProMedium.proto` 안에 센서·짐벌·프로펠러가 전부 들어 있어서 래퍼가
필요 없었고, 그래서 소환 기능의 첫 검증 대상이 드론이었다. 지금은 **라이다(VLP-16)를
얹으면서 드론도 래퍼를 쓴다** — 순정 PROTO에는 거리 측정 센서가 없어 SLAM을 못 돌렸다.
셋 다 같은 규칙이 됐다: 센서 구성은 PROTO 안에 한 번만 적고, 소환 문자열은 다섯 줄이다.

**`needs_def`(Spot)** — `spot_driver.py`가 Supervisor로 자기 몸 노드를 DEF 이름으로 찾는다.
DEF가 없으면 찾을 수단이 없고, 모두 같은 DEF면 2대째가 남의 몸을 잡는다.
소환기가 `spot2` → `DEF SPOT2`로 이름을 만들고 뇌에 `ROBOT_DEF`로 같은 값을 넘긴다.

**`needs_sync`(드론)** — 4장 참고. 주입은 항상 `synchronization FALSE`로 하고,
드론만 뇌 접속 확인 후 TRUE로 되돌린다.

> 씬 트리에서 "이건 우리 로봇이다"라고 알아보는 목록에는 옛 원본 PROTO
> (`SummitXlSteel`, `Spot`, `Mavic2ProMedium`)도 들어 있다. 예전 월드를 열었을 때
> 자동 채번이 `ugv1`을 다시 발급하거나 기존 로봇 위에 겹쳐 소환하는 것을 막는다.

---

## 4. 소환 한 번에 일어나는 일

```
① 이름 채번        씬 트리를 훑어 쓰이지 않는 번호를 고른다 (ugv3, ugv4 ...)
② 자리 고르기      random 이면 5장의 샘플러, 아니면 요청 좌표 + 겹침 검사
③ 몸 주입          importMFNodeFromString — 항상 synchronization FALSE
④ 뇌 띄우기        로봇별 컨테이너가 기다리거나, fleet 컨테이너가 프로세스로 띄운다
⑤ 롤백 감시        brain_grace_period(8초) 안에 뇌가 죽으면 몸을 씬 트리에서 되돌린다
⑥ 동기화 복원      needs_sync 인 로봇만, 뇌 접속을 확인한 뒤 synchronization TRUE
```

### 왜 주입 시점에는 항상 `synchronization FALSE`인가

`<extern>` 컨트롤러가 `synchronization TRUE`면 Webots는 매 스텝 그 컨트롤러의 응답을
기다린다. 로봇을 주입한 순간부터 뇌가 접속할 때까지 **시뮬레이션 전체가 멈춘다.**
그런데 소환기 자신도 같은 시뮬레이션에서 `step()`을 돌고 있으므로 같이 멈추고,
뇌가 끝내 안 뜨면 롤백 감시 타이머조차 돌지 못해 영원히 굳는다(헤드리스 Webots로 실측).

다만 FALSE로 **계속 두면** 뇌가 느릴 때 제어 주기가 물리 주기와 어긋난다. 지상 로봇은
버티지만 드론은 못 버틴다 — 실제로 로봇 6대의 뇌를 한 컨테이너에서 돌리자 소환된
드론이 이륙하지 못하고 바닥(z≈0.03)에 누워 미끄러졌다. 짐벌 경고에 찍힌 롤 각속도
14 rad/s는 "호버링"이 아니라 "뒹구는" 값이다.

그래서 `needs_sync`인 로봇만 **뇌가 붙은 것을 확인한 뒤** TRUE로 되돌린다.
주입 순간의 멈춤은 피하고, 비행에 필요한 보장은 되찾는다.

### 롤백 (despawn은 없다)

despawn 서비스는 만들지 않았다. 대신 **스폰 실패 시 롤백만** 한다 — 뇌가 유예 시간
(`brain_grace_period`, 기본 8초) 안에 죽으면 몸을 씬 트리에서 되돌려, 조종할 수 없는
유령 로봇이 쌓이지 않게 한다. 런치 파일 오타 하나로 시뮬이 시체로 채워지는 것을 막는
장치다.

> 소환기 노드는 **`use_sim_time: false`로 돈다.** 타임스탬프가 붙는 데이터를 발행하지
> 않고, 대신 "뇌가 몇 초 안에 떴는가"를 실제 시간으로 재야 하기 때문이다. 시뮬을
> 일시정지하면 sim time이 멈춰 롤백 감시가 영원히 안 돌게 된다.

---

## 5. 빈 자리 고르기

`random: true`면 [free_space_sampler.py](src/webots_robot_spawner/webots_robot_spawner/free_space_sampler.py)가
자리를 고른다. 기준 맵은 **전역 병합 맵 `/map_merged`**다.

병합 맵을 쓸 수 있는 이유는 `world → {ns}/map`이 이 프로젝트에서 항등변환이기 때문이다
([MAP_MERGE.md 2장](MAP_MERGE.md#2-정렬-설계--world-앵커-프레임)). **병합 맵의 (x, y)가
곧 Webots 월드 좌표**라 좌표 변환 없이 그대로 `translation`에 넣을 수 있다.

| 규칙 | 이유 |
|---|---|
| 맵 QoS는 `TRANSIENT_LOCAL + RELIABLE + depth 1` | 어긋나면 **에러 없이 조용히** 맵이 안 들어온다. 자리 못 찾을 때 제일 먼저 볼 곳 |
| `allow_unknown: false` (기본) | 미탐색(-1)에는 안 놓는다. "SLAM이 비었다고 확인한 곳"에만 |
| 요청 반경 = `clearance` 또는 종류별 기본값 | 원형 마스크로 주변 셀이 전부 비었는지 본다 |
| `robot_separation` (기본 1.0 m)을 더해 검사 | 로봇끼리 붙어 스폰되어 서로 밀어내는 것을 막는다 |
| `sample_attempts` (기본 200)회 시도 | 맵이 좁고 로봇이 많을수록 올린다 |
| `spawn_area`는 **항상** 지켜진다 | 맵이 있으면 그 영역 안에서 장애물까지 피하고, 냉시동(SLAM 맵이 존재할 수 없음)에서는 로봇 간 간격만 보고 고른다 |

`force: true`를 주면 빈 공간 검사에 실패해도 그 자리에 놓는다. 검사 자체가 틀렸다고
판단할 때(예: 아직 아무도 안 가본 구역)만 쓴다.

> 월드 생성기가 편대 매니페스트에 좌표까지 써 주는 것도 같은 이유다. 생성기는 어디가
> 비었는지 격자 단위로 알고 있어서, 손으로 고르는 것보다 정확하다
> ([WORLD_GEN.md](WORLD_GEN.md)).

---

## 6. 편대 매니페스트와 compose 생성

편대는 yaml로 정의한다 ([config/fleet/](src/webots_robot_spawner/config/fleet/)):

```yaml
fleet:
  - {type: ugv,   id: ugv1, x: -6.159, y: 1.263, yaw: -2.910}
  - {type: ugv,   count: 3, random: true}       # 3대를 알아서
  - {type: drone, count: 2, random: true, clearance: 1.0}
spawn_area: [-9.0, -6.0, 9.0, 7.0]              # random 배치 영역
```

| 파일 | 내용 |
|---|---|
| `default.yaml` | ugv1 / ugv2 / spot1 / drone1 — 예전 월드와 같은 좌표 |
| `random_squad.yaml` | UGV 3 + Spot 1 + 드론 2, 무작위 배치 |
| `ugv_only.yaml` | UGV 2대만 (맵 작업용 경량 편대) |
| `warehouse100.yaml` / `arena150.yaml` / `arena_s3.yaml` | 생성기가 월드와 함께 만든 것 |

**compose는 매니페스트에서 생성한다.** 손으로 유지하면 매니페스트와 이중 관리가 되고,
어긋나도 아무 에러 없이 조용히 틀린다 — 소환기는 새 편대를, 로봇 컨테이너는 옛 이름을
쓰게 된다.

```bash
# Ubuntu / macOS / Git Bash / WSL
docker run --rm -v "$PWD:/w" -w /w windows-master \
  python3 src/webots_robot_spawner/scripts/gen_fleet_compose.py --fleet default.yaml
```

셸별 문법(`%cd%` / `${PWD}` / `$PWD`)과 이미지 이름은
[WORLD_GEN.md 2장](WORLD_GEN.md#2-os별-실행-방법-중요)에 표로 정리해 뒀다.

`# >>> FLEET GENERATED` 마커 사이만 갈아 끼우므로 `master`/`fleet` 서비스와 주석은 그대로
남는다. `--check`를 붙이면 고치지 않고 최신인지만 확인한다(CI용).

- 맵 병합·RViz 표시는 손댈 것이 없다. 어떻게 태어난 로봇이든 `robot_registrar`로
  등록해서 마스터 입장에선 구분되지 않는다 ([MAP_MERGE.md](MAP_MERGE.md) 참고)
- 매니페스트는 컨테이너에 **마운트**되어 있어서, 편대만 바꿨다면 재빌드 없이 재시작만
  하면 된다

---

## 7. 기동 순서 (왜 드론만 다른가)

`compose up` 할 때 순서가 중요하다. 세 가지가 얽혀 있다.

| 서비스 | 기다리는 것 | 이유 |
|---|---|---|
| ugv / spot | `fleet`의 **healthcheck** (`service_healthy`) | 소환기가 몸을 다 확정한 뒤 드라이버가 붙어야 한다 |
| **drone** | `fleet`의 **기동만** (`service_started`) | 아래 교착 때문에 먼저 떠야 한다 |

소환기는 몸을 다 확정하면 `/tmp/fleet_ready`를 만들고, fleet의 healthcheck가 그걸 본다.
이게 없으면 **드라이버가 옛 몸에 붙은 직후 소환기가 그 몸을 잔여물로 지워서 드라이버가
끊기고 종료한다**(실측: 접속 t=26s, 제거 t=38s).

드론만 예외인 이유는 교착이다:

```
드론 몸은 비행을 위해 synchronization TRUE
   → compose down 하면 그 몸만 월드에 남는다
   → Webots 가 기다려 줄 컨트롤러가 없어 시뮬을 멈춘다
   → 소환기의 step() 이 막혀 편대 처리를 못 한다
   → /tmp/fleet_ready 가 안 생겨 healthcheck 실패
   → 드론 컨테이너가 안 뜬다 → 처음으로 되돌아감
```

소환기가 스스로 풀 수 없다. 시작할 때 잔여 몸의 `synchronization`을 내려도
**supervisor의 필드 쓰기는 스텝이 돌아야 반영되는데 그 스텝에서 막혀 있다**
(두 번 연속 기동해도 같은 몸이 계속 TRUE로 보이는 것으로 확인). 이 고리를 끊을 수
있는 것은 그 드라이버가 붙는 것뿐이라, 드론 컨테이너만 먼저 띄운다.

대신 `fleet_start_delay`가 20초다. 먼저 뜬 드론이 `robot_registrar`까지 올라올 시간을
줘야 살아있는 드론을 잔여물로 오판해 몸을 지우지 않는다.
**편대가 뜨기까지 30초쯤 걸리는 건 느린 게 아니라 이 순서를 지키는 중이다.**

---

## 8. 잔여 몸 정책 (`stale_body_policy`)

**Webots를 켠 채 `compose down` 하면 로봇의 몸이 월드에 남는다.** 다시 올릴 때
`stale_body_policy`(기본 `recreate`)가 처리한다:

| 몸의 상태 | 처리 | 이유 |
|---|---|---|
| 뇌가 살아 있음 (`/robot_registry`에 보임) | **그대로 둔다** | 지우면 정상 동작 중인 드라이버가 끊기고 `ros2 launch`가 되살리지 않는다 |
| 뇌가 없음 (잔여물) | **지우고 새로 소환** | 지난 세션 잔여물을 물려받지 않는다 |

생사 판단에 `/robot_registry`를 쓰는 이유는 QoS가 `TRANSIENT_LOCAL`이라 **늦게 구독해도
살아있는 registrar의 명함은 받고, 죽은 것의 명함은 안 오기** 때문이다. 그대로 두고
싶으면 `stale_body_policy: adopt`.

> **뇌만 다시 붙이는 방식(`attach`)은 폐기했다.** 장치가 disabled 상태로 남아 센서가
> 죽는다 — 실측: 뇌만 붙인 드론은 `wb_gps_get_values() called for a disabled device`가
> 4726건, 같은 시점에 새로 소환한 드론은 0건이었다.

---

## 9. 파라미터 표

[config/spawner.yaml](src/webots_robot_spawner/config/spawner.yaml) — fleet 컨테이너 전용.
노드 이름 대신 와일드카드(`/**`)를 쓴다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `use_sim_time` | **`false`** | 의도된 값. 롤백 감시를 실제 시간으로 재야 한다 (4장) |
| `map_topic` | `/map_merged` | 빈 자리를 고를 때 볼 맵 |
| `allow_unknown` | `false` | 미탐색(-1) 영역에도 놓을지 |
| `sample_attempts` | `200` | 무작위 자리 찾기 시도 횟수 |
| `robot_separation` | `1.0` | 로봇끼리 최소 이격(m). 요청 clearance에 더해 검사 |
| `brain_grace_period` | `8.0` | 이 시간(초) 안에 뇌가 죽으면 몸을 롤백 |
| `log_dir` | `/tmp/spawned_robots` | 소환된 로봇별 로그 위치 |
| `auto_launch_brain` | `true` | `false`면 몸만 넣고 뇌는 안 띄운다 (디버깅용) |
| `fleet_start_delay` | `20.0` | 매니페스트 처리 전 대기(초). 7장 참고 |
| `manifest_brains` | `true` | 매니페스트 로봇의 뇌까지 fleet이 띄울지. 로봇별 컨테이너 구조에서는 `false` |
| `stale_body_policy` | `recreate` | `recreate` \| `adopt` (8장) |
| `registry_topic` | `/robot_registry` | 생사 판단에 쓰는 등록 토픽 |
| `ready_file` | `/tmp/fleet_ready` | healthcheck가 보는 파일 (7장) |
| `fleet_manifest` | (런치 인자) | **여기 적지 않는다.** 런치의 `fleet:=`가 이긴다 |

### 서비스 요청 필드 (`SpawnRobot.srv`)

| 필드 | 뜻 |
|---|---|
| `type` | `ugv` / `spot` / `drone` |
| `robot_id` | 비우면 씬 트리를 보고 자동 채번 |
| `random` | true면 x/y/yaw 무시하고 맵의 빈 자리를 고름 |
| `x`, `y`, `yaw` | 월드 절대좌표 |
| `min_clearance` | 주변에 요구할 여유 반경(m). 0이면 종류별 기본값 |
| `force` | 빈 공간 검사 실패에도 그 자리에 놓음 |

응답에 실제 부여된 이름과 좌표, 실패 시 사유가 담긴다.

---

## 10. 트러블슈팅

**`In order to import the PROTO 'X', first it must be declared in the IMPORTABLE EXTERNPROTO list.`**
월드에 `IMPORTABLE EXTERNPROTO` 선언이 없다. 일반 `EXTERNPROTO`로는 런타임 주입이 안 된다.
`prepare_world.py --check`로 확인하고, `--in-place`로 고친다
([WORLD_GEN.md 6장](WORLD_GEN.md#6-외부-월드-가져오기-prepare_worldpy)).

**월드는 열리는데 로봇이 안 나온다.**
편대 매니페스트 이름과 compose의 `fleet:=` 값이 다른 경우가 대부분이다.
`gen_fleet_compose.py --check`로 확인한다. 소스를 고쳤다면 이미지 재빌드도 필요하다.

**월드를 재로드하면 뇌들이 죽는다.** `driver` 프로세스가 종료되는데 `ros2 launch`가
되살리지 않는다. `docker compose restart`로 다시 띄운다. `fleet` 컨테이너는
`restart: unless-stopped`라 스스로 돌아온다.

**소환된 로봇이 이상하다 — 로그는 어디에?**
매니페스트에 있는 로봇은 자기 컨테이너에 있다(`docker logs -f ugv1_brain_windows`).
런타임 소환된 로봇은 fleet 컨테이너가 뇌를 띄우므로 로봇별 파일로 쌓인다:

```bash
docker exec fleet_spawner_windows tail -f /tmp/spawned_robots/ugv3.log
```

한 컨테이너에서 여러 뇌가 돌아 표준출력이 섞이기 때문에 파일을 나눠 둔 것이다.

**`ros2 topic hz`를 믿지 말 것.** 로봇이 늘어 노드가 100개를 넘으면 있는 토픽도
"does not appear to be published yet"으로 나온다(CLI가 매번 새 참여자로 discovery를
처음부터 함). rclpy로 직접 구독해 확인한다
([MAP_MERGE.md 10장 ②-2](MAP_MERGE.md#-2-ros2-topic-hz가-거짓말을-할-때)).

**소환한 드론이 이륙을 못 하고 바닥에서 미끄러진다.** 동기화가 TRUE로 복원되지 않았다.
뇌 접속을 소환기가 확인하지 못한 것이므로, 그 로봇의 드라이버 로그부터 본다 (4장).

**빈 자리를 못 찾는다 (`random: true` 실패).** 순서대로 의심한다 —
① `/map_merged`가 나오는가(맵 QoS·마스터 컨테이너), ② `spawn_area`가 실제 자유 공간을
포함하는가, ③ `sample_attempts`를 올리거나 `clearance`를 낮춘다, ④ 정말 자리가 없다면
`force: true`.

---

## 11. 새 로봇 종류를 추가하려면

목표는 **[robot_types.py](src/webots_robot_spawner/webots_robot_spawner/robot_types.py)에
항목 하나를 더하는 것으로 끝나는 것**이다. 체크리스트:

1. **PROTO 준비** — 센서가 월드에 인라인으로 박히지 않고 PROTO 안에 들어 있어야 한다.
   아니면 `*Sensorized.proto` 같은 래퍼를 하나 만든다
2. **월드에 `IMPORTABLE EXTERNPROTO` 선언** — 생성기 4종이 공유하는 `prepare()`가
   자동으로 넣으므로, 목록에 추가만 하면 된다
3. **`RobotType` 항목 추가** — `spawn_z`(바닥에 닿는 높이), `footprint_radius`,
   `default_clearance`, 뇌 런치, `has_map`, 필요하면 `needs_def` / `needs_sync`
4. **뇌 런치 파일** — `os.environ.get('ROBOT_ID')`로 네임스페이스를 받고,
   `robot_registrar`를 포함시킨다 ([MAP_MERGE.md 10장 ⑦](MAP_MERGE.md#-로봇-4종-모두-등록-노드를-띄운다))
5. **compose 생성기** — 새 종류가 매니페스트에 들어가면 `gen_fleet_compose.py`가
   서비스를 만들어야 하므로 그쪽 매핑도 확인

---

## 12. 파일 맵

```
src/webots_robot_spawner/
├── webots_robot_spawner/
│   ├── spawn_supervisor.py     # 소환 본체 (서비스 · 편대 · 롤백 · 잔여 몸)
│   ├── robot_types.py          # 로봇 종류 정의표 (몸 + 뇌)
│   ├── fleet_loader.py         # 편대 매니페스트 해석
│   ├── free_space_sampler.py   # /map_merged 에서 빈 자리 고르기
│   └── brain_launcher.py       # 뇌를 프로세스로 띄우고 거둔다
├── config/
│   ├── spawner.yaml            # 파라미터 (9장)
│   ├── fleet/*.yaml            # 편대 매니페스트
│   └── doorways/*.yaml         # 생성 월드의 방·출입구 좌표 (WORLD_GEN.md)
├── launch/spawner.launch.py    # fleet:= 인자로 매니페스트 선택
└── scripts/                    # 월드·compose 생성기 → WORLD_GEN.md
src/webots_spawner_msgs/srv/SpawnRobot.srv
```

### 관련 문서

- [Readme.md](Readme.md) — 전체 구성과 빠른 사용법
- [WORLD_GEN.md](WORLD_GEN.md) — 월드를 만들면 편대 매니페스트가 함께 나온다
- [MAP_MERGE.md](MAP_MERGE.md) — 소환된 로봇이 관제 화면에 합류하는 경로
- [drone_setup.md](drone_setup.md) — `needs_sync`가 왜 드론에만 필요한지의 근거
- [INTERFACES.md](INTERFACES.md) — 토픽·서비스·프레임 총람
