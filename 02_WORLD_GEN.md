# 02. 월드 생성 (World Generation)

> 📖 [책 목차](Readme.md#-목차) · ← [01. 인터페이스 총람](01_INTERFACES.md) · [03. 로봇 소환](03_SPAWNER.md) →

로봇이 작전할 **환경**을 만드는 방법을 모았다. 로봇을 그 안에 올리는 이야기는
[Readme 12. 로봇 소환](Readme.md#12-로봇-소환-runtime-spawn)에 있다.

- [1. 어떤 방법을 고를까](#1-어떤-방법을-고를까)
- [2. OS별 실행 방법 (중요)](#2-os별-실행-방법-중요)
- [3. 무작위 방·복도 생성 (`gen_world_random.py`)](#3-무작위-방복도-생성-gen_world_randompy)
  - [3-1. 옵션 — 방·복도·출입구 개수 지정](#3-1-옵션--방복도출입구-개수-지정)
  - [3-2. 어떻게 만들어지나](#3-2-어떻게-만들어지나)
  - [3-2-1. 부지와 외부 출입구](#3-2-1-부지와-외부-출입구)
  - [3-3. 출입구 yaml](#3-3-출입구-yaml)
  - [3-4. 검증된 것 / 안 된 것](#3-4-검증된-것--안-된-것)
- [4. 창고형 생성 (`gen_world.py`)](#4-창고형-생성-gen_worldpy)
- [5. 점유격자 → 월드 (`gen_world_from_map.py`)](#5-점유격자--월드-gen_world_from_mappy)
- [6. 외부 월드 가져오기 (`prepare_world.py`)](#6-외부-월드-가져오기-prepare_worldpy)
- [7. 월드를 바꾼 뒤 해야 할 일](#7-월드를-바꾼-뒤-해야-할-일)
- [8. 트러블슈팅](#8-트러블슈팅)

---

## 1. 어떤 방법을 고를까

| 스크립트 | 나오는 것 | 언제 쓰나 |
|---|---|---|
| [gen_world_random.py](src/webots_robot_spawner/scripts/gen_world_random.py) | 시드마다 다른 **건물** — 복도 + 방 + 장애물 | 훈련·벤치마크 환경을 무한히 뽑을 때, "처음 보는 곳" 작전 시험 |
| [gen_world.py](src/webots_robot_spawner/scripts/gen_world.py) | 결정적인 창고 (선반 열) | 매번 같은 배치가 필요할 때 (회귀 시험, 성능 비교) |
| [gen_world_from_map.py](src/webots_robot_spawner/scripts/gen_world_from_map.py) | 점유격자(PGM+YAML)를 그대로 옮긴 월드 | SLAM으로 딴 실제 지도나 건물 도면을 시뮬로 되돌릴 때 |
| [prepare_world.py](src/webots_robot_spawner/scripts/prepare_world.py) | 밖에서 가져온 `.wbt`를 소환 가능 상태로 | Webots 샘플·webots.cloud 자산을 쓸 때 |

넷 다 **같은 `prepare()` 로직**을 거친다. 그래서 어느 쪽으로 만들든 월드에는

- `DEF SPAWN_SUPERVISOR Robot` (소환 전담 유령 로봇)
- 로봇 PROTO 3종의 **`IMPORTABLE EXTERNPROTO`** 선언

이 자동으로 들어간다. 이게 없으면 런타임 소환이 실패한다([8. 트러블슈팅](#8-트러블슈팅)).

### 접촉 설정은 월드가 들고 있다

생성기 3종은 `WorldInfo.contactProperties` 를 `my_world.wbt` 와 같게 넣는다.
**로봇 PROTO 가 아니라 월드에 있어야 하는 값**이라, 새 월드를 만들 때마다 같이
가지 않으면 로봇 거동이 조용히 달라진다.

| material | 무엇 | 왜 필요한가 |
|---|---|---|
| `InteriorWheelMat` / `ExteriorWheelMat` | SummitXL 메카넘 휠 | 롤러가 45도로 누워 있어 그 방향으로만 미끄러져야 게걸음이 된다. 없으면 등방성 마찰이 걸려 **옆으로 못 가고 앞뒤로만 간다** |
| `slope` | 경사면 | 마찰 0.5 |

`coulombFriction [0, 2, 0]` + `frictionRotation ±0.785` 가 그 이방성을 만들고,
`softCFM 0.0001` 은 접촉을 아주 약간 무르게 해 솔버를 안정시킨다.

> 한동안 생성 월드 3종에 이게 통째로 빠져 있었다. `my_world` 에서 튜닝한 주행이
> 생성 월드에서 다르게 나오면 이걸 먼저 의심한다.

---

## 2. OS별 실행 방법 (중요)

스크립트는 파이썬 + numpy를 쓴다. **호스트에 파이썬을 깔 필요는 없다** — 프로젝트
도커 이미지 안에서 돌리면 된다. 대신 **"현재 폴더"를 컨테이너에 넘기는 문법이 셸마다
다르다.** 여기서 두 번 막힌 적이 있으니 자기 셸의 줄을 그대로 복사해 쓰는 게 좋다.

> 이 장은 **셸 문법만** 다룬다. 아래 예시의 `--size --seed --name` 은 최소한이고,
> 방·복도·출입구 **개수를 직접 정하는 옵션**이 따로 있다 →
> [3-1](#3-1-옵션--방복도출입구-개수-지정)

먼저 저장소 최상위(`webots_multi_robot/`)로 이동한 뒤:

### Ubuntu / Linux

```bash
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3
```

### macOS

명령은 Linux와 똑같다. 셸이 zsh든 bash든 `$PWD`가 그대로 통한다.

```bash
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3
```

> 애플 실리콘(M1~)에서 이미지가 `linux/amd64`로 빌드돼 있으면 에뮬레이션으로 돌아
> 느리다. 생성 자체는 몇 초짜리라 문제되지 않는다.

### Windows — cmd.exe (명령 프롬프트)

**`$PWD`도 `${PWD}`도 통하지 않는다. `%cd%`를 쓴다.** 줄을 나누려면 `^`.

```bat
docker run --rm -v "%cd%:/w" -w /w windows-master python3 ^
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3
```

### Windows — PowerShell

**`"$PWD:/w"`는 실패한다.** PowerShell이 `$PWD:`를 드라이브 한정 변수로 읽어서
확장을 못 한다. `${PWD}`로 감싼다. 줄을 나누려면 백틱(`` ` ``).

```powershell
docker run --rm -v "${PWD}:/w" -w /w windows-master python3 `
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3
```

### Windows — Git Bash / WSL

Linux와 같다(`$PWD`, 줄 나눔은 `\`). 단 Git Bash는 경로를 멋대로 윈도우 경로로
바꾸는 버릇이 있어, 실패하면 `MSYS_NO_PATHCONV=1`을 앞에 붙인다.

```bash
MSYS_NO_PATHCONV=1 docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3
```

### 셸이 헷갈리면 — 절대경로

어느 셸에서든 통한다. 확실하게 하고 싶으면 이쪽을 쓴다.

```
-v "D:/Document/Duck_Project/webots_multi_robot:/w"
```

### 요약표

| 셸 | 현재 경로 | 줄 나눔 |
|---|---|---|
| Ubuntu / macOS / WSL | `"$PWD:/w"` | `\` |
| Git Bash | `"$PWD:/w"` (+ `MSYS_NO_PATHCONV=1`) | `\` |
| Windows cmd.exe | `"%cd%:/w"` | `^` |
| Windows PowerShell | `"${PWD}:/w"` | `` ` `` |

> 이미지 이름 `windows-master`는 윈도우 전용이라는 뜻이 아니라 `docker-configs/windows/`
> compose가 붙인 태그일 뿐이다. 우분투/맥에서는 각자 compose가 만든 이미지 이름
> (`ubuntu-master`, `mac-master`)을 쓰거나, `docker images`로 확인해서 넣는다.

---

## 3. 무작위 방·복도 생성 (`gen_world_random.py`)

시드마다 **완전히 다른 지형**이 나온다. 같은 시드면 언제 어디서 돌려도 같은 월드가
나온다(재현 가능).

```bash
# 우분투/맥 기준. 다른 셸은 2절 참고
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3
```

**지형을 시드에만 맡기지 않아도 된다.** 방·복도·출입구 개수를 직접 정할 수 있고,
지정하지 않으면 크기에 맞게 알아서 정해진다:

```bash
# 주복도 3개 · 연결복도 2개 · 방 40개 · 외부 출입구 2개 · 마당 20 m
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3 \
  --corridors 3 --links 2 --rooms 40 --entrances 2 --yard 20
```

| 정하고 싶은 것 | 옵션 | 기본 |
|---|---|---|
| 가로 주복도 개수 | `--corridors N` | 자동 (`--room-depth` 기준) |
| 세로 연결복도 개수 | `--links N` | 자동 (`0` 이면 없음) |
| 방 개수 | `--rooms N` | 자동 |
| 외부 출입구 개수 | `--entrances N` | 자동 (최소 1개) |

전체 목록은 [3-1](#3-1-옵션--방복도출입구-개수-지정)에 있다.

**칸막이 없이 텅 빈 방 하나만 필요하면** `--single-room` 을 쓴다. 로봇이나 센서
자체를 시험할 때 — 지형이 변수로 끼면 드라이버 문제인지 지도 문제인지 가리기가
어렵다.

```bash
# 76 x 76 m 원룸 하나 + 마당 12 m + 출입구 2개, 안은 텅 빔
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world_random.py --name oneroom \
  --single-room --density 0
```

건물 외피·마당·울타리·외부 출입구는 그대로 나오고 **안쪽만 하나로 트인다.**
`--density` 를 주면 그 방 안에 상자가 뿌려진다.

나오는 파일 세 개:

| 파일 | 내용 |
|---|---|
| `src/Webots-SummitXL/workspace/simulator/worlds/arena_s3.wbt` | 월드 |
| `src/webots_robot_spawner/config/fleet/arena_s3.yaml` | 편대 매니페스트 (로봇 4대 배치 좌표) |
| `src/webots_robot_spawner/config/doorways/arena_s3.yaml` | **방·출입구 목록** |

실행 결과 예시:

```
월드 생성: .../worlds/arena_s3.wbt
  100 x 100 m, 격자 200x200 @ 0.5 m, seed 3
  주복도 4개  연결복도 2개  방 45개  장애물 206개
  방 면적 최소 25 / 중앙 58 / 최대 200 m2
  자유공간 91%  벽 박스 150개
  방 출입구 45개 (최소 폭 1.2 m)
  바깥 땅 12 m 띠, 건물 76 x 76 m, 외부 출입구 4개 (north, south, west, east)
출입구 목록: .../config/doorways/arena_s3.yaml
편대 매니페스트: .../config/fleet/arena_s3.yaml
  ugv1    ( -14.25,  -28.75)
  ugv2    (   8.25,  -28.25)
  spot1   (   7.25,   27.75)
  drone1  ( -24.75,   28.25)
```

### 3-1. 옵션 — 방·복도·출입구 개수 지정

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--size N` | 100 | 한 변 길이(m) |
| `--seed N` | 1 | **시드가 다르면 지형이 다르다.** 같으면 완전히 같다 |
| `--name NAME` | `arena_s{seed}` | 월드·매니페스트·출입구 파일 이름 |
| `--cell M` | 0.5 | 격자 한 칸(m). 작을수록 세밀하지만 박스가 늘어 Webots가 느려진다 |
| **`--yard M`** | 12 | **건물 둘레에 남길 바깥 땅 폭(m).** `0`이면 건물이 부지를 꽉 채운다 |
| **`--entrances N`** | 0 = 자동 | **바깥에서 건물로 들어오는 출입구 개수.** 복도 끝에만 낸다 |
| `--fence-h H` | 2 | 부지 울타리 높이(m) |
| **`--corridors N`** | 0 = 자동 | **가로 주복도 개수.** 크기에 안 들어가면 줄이고 알려준다 |
| **`--links N`** | -1 = 자동 | **세로 연결복도 개수.** `0` 이면 없음 (주복도가 2개 이상이면 1로 올린다) |
| **`--rooms N`** | 0 = 자동 | **방 개수 목표.** 정하면 폭을 거기 맞춰 나눈다 (근사값) |
| **`--single-room`** | — | **원룸** — 칸막이도 복도도 없는 방 하나. 위 세 옵션은 무시된다 |
| `--room-depth M` | 8 | 방 깊이 기준(m). 주복도 개수를 자동으로 정할 때 쓴다 |
| `--room-min M` | 4 | 방 최소 폭(m) |
| `--corridor M` | 3 | 주복도 기준 폭(m). 실제 폭은 복도마다 흔들린다 |
| `--link-w M` | 2.5 | 연결복도 폭(m) |
| `--door M` | 1.8 | 출입구 폭(m) |
| `--density D` | 6 | 실내 장애물 밀도 (**방 면적** 100 m² 당 개수). `0`이면 빈 방 |
| `--yard-density D` | -1 = 자동 | 마당 장애물 밀도. 자동은 `--density`의 35% |
| `--wall-h H` | 3 | 벽 높이(m) |
| `--min-door M` | 1.2 | 출입구로 인정할 최소 폭(m) |
| `--robot-w M` | 0.72 | 지나다닐 로봇 폭(m). 상자를 놓을 때 옆으로 이만큼 + 여유를 반드시 남긴다 |
| `--no-fleet` | — | 편대 매니페스트를 만들지 않는다 |
| `--no-doorways` | — | 출입구 yaml을 만들지 않는다 (검사·복구는 그대로 한다) |

**`--seed`는 여기서만 지형을 바꾼다.** `gen_world.py`의 `--seed`는 배치가 아니라
편대 스폰 후보만 흔든다 — 이름이 같아서 헷갈리기 쉽다.

**개수는 크기에 맞춰 자동 조정된다.** `--corridors 20`을 100 m에 요청하면 띠가
너무 얇아지므로 들어가는 만큼으로 줄이고 그 사실을 알려준다.
`--rooms`는 목표치라 정확히 맞지는 않는다(요청 40 → 41, 150 → 143 정도).
방마다 용도를 뽑아 크기를 섞기 때문에 정수로 딱 떨어지지 않는다.

| 이렇게 하고 싶으면 | 이렇게 |
|---|---|
| **원룸 하나** (로봇·센서 시험용) | `--single-room --density 0` |
| 원룸에 장애물만 좀 | `--single-room` |
| 복도 하나에 방만 줄지어 | `--corridors 1 --links 0` |
| 넓은 방 위주 (창고·강당) | `--corridors 3 --rooms 40` |
| 잘게 나뉜 사무실 층 | `--corridors 8 --rooms 150` |
| 복도만 있는 빈 건물 | `--density 0` |
| 문을 더 넓게 (큰 로봇) | `--door 2.5 --robot-w 1.2` |
| 실내만 (예전 동작) | `--yard 0` |
| 넓은 야외 작전 + 건물 하나 | `--yard 30 --entrances 2` |
| 사방에서 진입 | `--entrances 8` |

### 3-2. 어떻게 만들어지나

**BSP는 쓰지 않는다.** 공간을 재귀로 쪼개는 BSP는 구획을 만들 뿐 방을 만들지
않는다. 결과가 미로처럼 보이고 복도는 방 중심을 잇는 통로에 그쳐서, 건물로 읽히지
않았다. 실제 건물은 정반대 순서다 — **동선(복도)이 먼저 있고, 방이 거기에 면해서
줄지어 붙는다.** 그래서 이 스크립트도 그 순서로 만든다.

```
1. 부지 안쪽에 건물 자리를 잡는다 (둘레는 바깥 땅으로 남긴다)
2. 가로 주복도를 n 개 놓는다 (층을 가로지르는 긴 복도)
3. 세로 연결복도로 주복도들을 잇는다
4. 복도 끝이 외피와 만나는 자리에 외부 출입구를 뚫는다
5. 복도 양옆 띠를 방으로 잘라 붙인다 — 학교 도면의 교실 줄과 같은 배치
6. 방마다 복도 쪽 벽에 출입구를 뚫는다 (문짝은 없다)
7. 방 안과 바깥 땅에 상자·팔레트를 뿌린다 (복도는 비워 둔다)
8. 격자를 큰 사각형으로 합쳐 Box 로 내보낸다
```

이 순서 덕분에 **연결성이 구조적으로 보장된다.** 모든 방은 복도에 면하고, 모든
복도는 연결복도로 이어져 있으며, 바깥 땅은 복도 끝으로 건물과 이어진다.
나중에 검사해서 고치는 게 아니라 만들 때부터 이어져 있다.

### 3-2-1. 부지와 외부 출입구

건물이 부지를 꽉 채우면 로봇이 실내에만 갇힌다. 그래서 건물을 안쪽으로 물리고
둘레 `--yard` m 를 바깥 땅으로 남긴다.

```
┌─────────────────────────────┐  ← 부지 울타리 (--fence-h, 기본 2 m)
│   마당 (상자 성기게)          │
│   ┌───────────────────┐     │
│   │ 건물              │     │
│   ╡ ← 외부 출입구      │     │   출입구는 복도 끝에만 낸다
│   │                   │     │
│   └───────────────────┘     │
└─────────────────────────────┘
```

**출입구는 복도 끝에만 낸다.** 외피 아무 데나 뚫으면 들어오자마자 방 하나에
갇힌다. 복도 끝을 열면 진입 즉시 동선에 붙어 건물 어디로든 갈 수 있다.
후보는 가로 복도의 동·서 끝과 연결복도의 남·북 끝이고, 한쪽 면에 몰리지 않게
면을 돌아가며 뽑는다.

`--entrances 0`은 허용하지 않는다 — 바깥 땅이 건물과 끊기면 그건 그냥
못 들어가는 건물이다. 최소 1개는 낸다.

두 가지는 취향이 아니라 필요다:

- **마당에도 물건을 뿌린다.** 텅 빈 평면은 라이다에 걸리는 게 없어 SLAM 이
  미끄러진다. 다만 실내 밀도를 그대로 쓰면 마당이 고물상이 되므로 기본 35%만 쓴다
- **부지 둘레에 울타리를 세운다.** 없으면 로봇이 바닥 밖으로 떨어진다

#### 바닥은 두 겹으로 깔지 않는다

마당과 실내를 색으로 가르려면 바닥이 두 장 필요한데, 여기서 두 번 틀렸다.
Box 를 겹칠 때 지켜야 하는 규칙이라 남겨 둔다.

처음에 이렇게 깔았다 — **둘 다 틀렸다**:

```
ground          z ∈ [-0.100, 0.000]   boundingObject 있음
building_floor  z ∈ [-0.030, 0.000]   boundingObject 있음   ← ground 안에 통째로
                         ↑ 윗면이 z=0 으로 정확히 같다
```

1. **충돌체를 둘 다 줬다.** 바퀴가 바닥에 닿을 때마다 같은 지점에서 접촉점이
   두 벌 생겨 물리 솔버가 과잉구속된다
2. **보이는 두 면이 같은 평면이다.** 렌더러가 어느 쪽을 그릴지 못 정해
   z-fighting(깜빡임)이 난다

지금은 이렇다:

```
ground          z ∈ [-0.100, 0.000]   boundingObject 있음  ← 밟는 면은 이것 하나
building_floor  z ∈ [ 0.002, 0.005]   boundingObject 없음  ← 순수 장식
```

- 밟는 면은 **하나면 된다.** 장식용 Solid 는 `solid(..., collide=False)` 로
  만들어 `boundingObject` 를 주지 않는다
- 겹치지 않게 **위에 얹는다.** 1.5 mm 띄운 3 mm 슬래브라 로봇이 잠기는 정도는
  눈에 띄지 않는다

> ⚠️ `boundingObject` 가 없어도 **라이다에는 보인다.** Webots 의 라이다·레인지
> 파인더는 그래픽 지오메트리를 훑기 때문이다. 바닥에 붙은 3 mm 슬래브는 수평
> 라이다 빔에 안 걸리지만, 벽처럼 세우는 것에는 `collide=False` 를 쓰면 안 된다
> — 라이다에는 보이는데 통과해 버리는 유령 벽이 된다.

**벽을 '그린다', 방을 '파내지' 않는다.** 빈 판에서 시작해 선을 그으면 방이 저절로
생긴다. 반대로 벽에서 시작해 방을 파내면 벽 두께를 맞추기가 훨씬 성가시다.

**크기를 어떻게 섞나** — 전부 같은 폭으로 나누면 모텔처럼 보인다. 구간마다 용도를
뽑아 방 개수 배율을 다르게 준다:

| 용도 | 배율 | 대략 |
|---|---|---|
| `hall` | 구간 통째로 방 하나 | 강당·체육관 |
| `large` | 평균의 0.45배 개수 | 도서실·식당 |
| `standard` | 평균 | 교실 |
| `small` | 평균의 2.1배 개수 | 사무실·창고 |

용도 확률은 **띠 깊이에 연동**한다. 깊은 띠(12 m 초과)에는 큰 방을, 얕은 띠에는
작은 방을 많이 넣는다. 이 연동이 없으면 깊은 띠에 폭 4 m · 깊이 15 m 짜리 복도 같은
방이 생긴다. 여기에 더해 방 폭이 깊이의 절반은 되게 묶어 비례를 지킨다.

복도 폭도 하나로 고정하지 않는다. 주동선 하나를 기준 폭의 1.45~1.9배로 넓게 잡고,
나머지는 0.8~1.15배로 흔든다. 띠 깊이도 시드마다 달라서 층마다 인상이 바뀐다.

**왜 격자를 합치나** — 200×200 격자를 셀 하나당 Box 하나로 내보내면 4만 개다.
Webots가 기어간다. 가로로 이어진 칸을 한 줄로 묶고 세로로 늘리는 탐욕 병합을 쓰면
벽 200개 남짓으로 줄어든다. (`gen_world_from_map.py`의 `merge_rectangles`를 그대로 쓴다)

**장애물이 길을 막지 않게 하는 규칙** — 벽까지 여유가 `k`칸인 자리에 반경 `half`짜리
상자를 놓으면 가장 가까운 벽까지 `k - half`칸이 남는다. 그래서

```
half ≤ k - ceil((robot_w + 0.3) / cell)
```

로 묶는다. 이 제약이 없으면 폭 3 m 복도 한가운데 1.5 m 상자가 들어가 양옆에
0.75 m만 남는데, 폭 0.72 m 로봇에게는 사실상 막힌 길이다(실제로 이 버그가 있었고
검증에서 잡혔다). 여기에 더해 출입구 목 주변은 아예 장애물 금지 구역으로 잡는다.

그래도 상자가 방 구석을 막아 주머니를 만들 수 있어서, 마지막에 플러드 필로
고립 구역을 찾아 **그 고립을 만든 상자를 통째로 빼낸다**(칸 단위로 지우면 상자가
반쯤 남는다).

### 3-3. 출입구 yaml

> "방마다 출입할 수 있는 곳은 있어야 한다. 문은 필요 없고 로봇이 진입할 정도 간격이면 된다.
> 그 간격들의 중앙 위치를 따로 yaml에 저장한다."

**문짝은 달지 않는다.** 벽이 끊긴 구간이 곧 출입구다. `config/doorways/NAME.yaml`:

```yaml
world: arena_s3
cell: 0.5
min_door_width: 1.2
yard: 12.00
building: [76.00, 76.00]

# 바깥에서 건물로 들어오는 곳. 복도 끝이라 진입 즉시 동선에 붙는다.
entrances:
  - {x: -14.25, y: 37.75, width: 2.50, side: north}
  ...

# 복도 중심선 — 순찰이나 광역 이동 경로를 짤 때 쓴다
corridors:
  - {center: [0.00, -25.75], size: [75.00, 3.00]}
  ...

rooms:
  - id: room_1
    center: [-28.75, -44.75]
    size: [6.50, 7.50]
    area: 48.8
    doorways:
      - {x: -28.75, y: -41.25, width: 1.50, side: north}
  ...

doorways:            # 방을 뒤지지 않아도 되게 평평한 목록도 같이 둔다
  - {room: room_1, x: -28.75, y: -41.25, width: 1.50, side: north}
  ...
```

- `x`, `y`는 그 틈의 **중앙**. `side`는 방 기준 방향(`west` = -x, `east` = +x,
  `south` = -y, `north` = +y)
- `width`는 실제로 뚫린 폭. 상자가 목을 좁혔으면 그만큼 줄어든 값이 나온다
- **좌표계는 Webots 월드 좌표 = 병합 지도(`/map_merged`) 좌표와 같다.**
  `odom_is_world_absolute: true`라 `world → {ns}/map`이 항등이기 때문이다.
  Nav2 목표점으로 그대로 넣어도 된다

방 사이를 오갈 때 이 점을 경유지로 쓰면 문틀을 긁지 않는다.

**출입구로 세는 조건** — 벽 줄 한 칸만 보면 "벽에 파인 홈"까지 출입구로 세게 된다.
그래서 **안쪽 · 벽 줄 · 바깥쪽이 모두 뚫린 줄**만 관통로로 인정한다.

### 3-4. 검증된 것 / 안 된 것

검증은 **생성기의 격자를 다시 쓰지 않는다**(순환 논증이 된다). Webots가 실제로 읽을
`.wbt`에서 Box를 긁어 0.1 m 격자로 다시 래스터화하고 — 생성기의 0.5 m와 일부러 다른
해상도다 — 로봇 반경 0.36 m만큼 부풀린 뒤 검사한다.

기본값 + 파라미터 스윕(출입구 1/8개, 마당 0/25/45 m, 150 m), **7개 구성 전부 통과**:

- 방 출입구 전부(4~78개) 로봇이 통과 가능. 처음엔 반경 0.36 m 로만 쟀는데
  **세 로봇 다 그보다 크다** — 드론 0.35 / UGV 0.47 / **Spot 0.60**
  (`robot_types.py` 의 `footprint_radius`). 가장 큰 Spot 기준으로 다시 재서 통과했다
- **바깥 땅에서 건물 안까지 이어져 있다** — 부지 모서리에서 플러드 필로 확인
- 자유공간 고립 0칸 — 모든 통행 가능 칸이 하나로 이어져 있다
- 방 100% 도달 가능

Webots 헤드리스 로드도 확인했다 — 노드 368개로 **오류 없이** 뜨고, 매니페스트의
로봇 4대가 그 좌표에 정상 주입된다. 60 물리 스텝을 돌린 뒤 위치를 다시 재니
**0.00~0.01 m** 만 움직였다. 스폰 지점이 실제로 비어 있다는 뜻이다 (벽이나 상자에
겹쳐 있으면 물리 엔진이 밀어낸다).

평면도가 실제로 건물처럼 읽히는지는 `.wbt`의 Box를 PNG로 렌더링해 눈으로 확인했다.
숫자만 보면 "방 120개"라도 전부 같은 크기일 수 있다 — 실제로 처음 판은 그래서
모텔처럼 보였고, 그걸 보고 용도 배합을 넣었다. 마당 밀도를 따로 둔 것도 같은
이유다(처음엔 고물상처럼 나왔다).

**확인하지 않은 것:**

- Nav2가 이 지형에서 실제로 경로를 뽑는지 (탐사·주행 시험은 아직)
- 드론이 벽 높이 3 m 위로 넘어다닐 때의 거동. 울타리는 2 m 라 드론은 부지 밖으로
  나갈 수 있다
- 바깥 땅에서 SLAM 이 실제로 잘 도는지 — 물건을 뿌린 건 그 대비지만 측정은 안 했다

---

## 4. 창고형 생성 (`gen_world.py`)

**결정적**이다. 크기가 같으면 항상 같은 배치가 나온다. 회귀 시험처럼 지형이 변수가
되면 안 되는 경우에 쓴다.

```bash
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world.py --size 150 --name arena150
```

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--size N` | 100 | 한 변 길이(m) |
| `--name NAME` | `warehouse100` | 월드/매니페스트 이름 |
| `--aisle W` | 4.0 | 통로 폭(m). UGV 폭 0.72 + Nav2 여유 |
| `--shelf-h H` | 3.0 | 선반 높이(m) |
| `--seed N` | 7 | **지형이 아니라 편대 스폰 후보만 흔든다** |
| `--no-fleet` | — | 편대 매니페스트를 만들지 않는다 |

---

## 5. 점유격자 → 월드 (`gen_world_from_map.py`)

nav2 지도(`map_saver_cli`가 뱉는 PGM + YAML)를 그대로 3D 월드로 세운다. SLAM으로 딴
실제 지도나 건물 도면을 시뮬레이션으로 되돌릴 때 쓴다.

```bash
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world_from_map.py \
  --map maps/office.yaml --name office --height 2.5
```

| 옵션 | 뜻 |
|---|---|
| `--map PATH` | nav2 map YAML (PGM과 같은 폴더에 있어야 한다) |
| `--name NAME` | 월드 이름 |
| `--height M` | 장애물 높이(m) |
| `--unknown {free,occupied}` | 미탐색(-1) 셀 취급. 기본 `free` |
| `--min-cells N` | 이보다 작은 조각은 버린다 (SLAM 잡음 제거) |
| `--downsample N` | N×N 셀을 하나로 묶어 박스 수를 줄인다 |
| `--stats` | 셀/박스 통계만 출력 |

> ⚠️ **`--unknown occupied`가 안 먹는 것처럼 보이면 `free_thresh`를 보라.** PGM의
> 미탐색 값 205는 확률로 치면 0.196이다. 지도 YAML의 `free_thresh`가 0.25처럼
> 크면 205가 **자유 공간으로 먼저 분류돼** `--unknown` 이 볼 셀이 남지 않는다.
> 스크립트가 이 경우 경고를 띄운다.

---

## 6. 외부 월드 가져오기 (`prepare_world.py`)

Webots 샘플이나 [webots.cloud](https://webots.cloud/) 자산을 그대로 쓰고 싶을 때,
소환에 필요한 선언만 끼워 넣는다.

```bash
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/prepare_world.py \
  --source /usr/local/webots/projects/.../factory.wbt --name factory
```

| 옵션 | 뜻 |
|---|---|
| `--source PATH` | 원본 `.wbt` |
| `--name NAME` | 저장할 이름 |
| `--in-place` | 원본을 직접 고친다 |
| `--check` | 고치지 않고 이미 준비됐는지만 본다 |

`# >>> SPAWNER READY` 마커 사이만 갈아 끼우므로 여러 번 돌려도 중복되지 않는다.

> ⚠️ 샘플 월드 중에는 **Webots 자체를 죽이는 것**이 있다. `factory.wbt`는 이 프로젝트와
> 무관하게 원본 그대로도 크래시했다(`webots-bin.exe`, 0xc0000005). `village`는
> `worlds/forest/*.forest` 캐시를 같이 가져와야 한다. 외부 월드는 **먼저 원본 그대로
> 열어 보고** 멀쩡한지 확인한 뒤 가져오는 게 빠르다.

---

## 7. 월드를 바꾼 뒤 해야 할 일

월드만 바꾸고 끝내면 **조용히 어긋난다** — 소환기는 새 편대를, 로봇 컨테이너는 옛
이름을 쓰게 된다.

**① compose를 새 편대에 맞춘다** (로봇별 서비스와 `fleet:=` 값이 한꺼번에 맞춰진다)

```bash
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_fleet_compose.py --fleet arena_s3.yaml
```

`# >>> FLEET GENERATED` 마커 사이만 갈아 끼우므로 `master`/`fleet` 서비스와 주석은
그대로 남는다. `--check`를 붙이면 고치지 않고 최신인지만 확인한다(CI용).

**② Webots에서 월드 열기** — `File > Open World...` → `worlds/arena_s3.wbt`

**③ 컨테이너 기동**

```bash
# 우분투
docker compose -f docker-configs/ubuntu/docker-compose.yml up -d
# 윈도우
docker compose -f docker-configs/windows/docker-compose.yml up -d
# 맥
docker compose -f docker-configs/mac/docker-compose.yml up -d
```

편대가 뜨기까지 30초쯤 걸린다. 느린 게 아니라
[기동 순서](03_SPAWNER.md#7-기동-순서-왜-드론만-다른가)를 지키는 중이다.

**재빌드가 필요한 때 / 아닌 때**

| 바꾼 것 | 필요한 조치 |
|---|---|
| 월드 `.wbt` | 없음 — Webots가 호스트에서 직접 읽는다 |
| 편대 매니페스트 / 출입구 yaml | 컨테이너 재시작 (마운트되어 있다) |
| compose 서비스 구성 | `up -d` |
| 파이썬 소스 (드라이버·소환기) | `build` 후 `up -d` |

---

## 8. 트러블슈팅

**`In order to import the PROTO 'X', first it must be declared in the IMPORTABLE EXTERNPROTO list.`**
월드에 `IMPORTABLE EXTERNPROTO` 선언이 없다. 일반 `EXTERNPROTO`로는 런타임 주입이
안 된다. `prepare_world.py --check`로 확인하고, 아니면 `--in-place`로 고친다.

**월드는 열리는데 로봇이 안 나온다.**
편대 매니페스트 이름과 compose의 `fleet:=` 값이 다른 경우가 대부분이다.
`gen_fleet_compose.py --check`로 확인한다. 소스를 고쳤다면 이미지 재빌드도 필요하다.

**`docker: Error response from daemon: create $PWD: ... only "[a-zA-Z0-9][a-zA-Z0-9_.-]" are allowed`**
셸이 `$PWD`를 확장하지 못하고 글자 그대로 넘겼다는 뜻이다. cmd.exe면 `%cd%`,
PowerShell이면 `${PWD}`. [2절](#2-os별-실행-방법-중요) 참고.

**방이 너무 잘거나 너무 크다.** `--rooms`로 개수를 직접 정한다. 크기만 키우면
방 개수가 늘어난다(같은 폭으로 더 많이 들어가므로). 넓은 방을 원하면
`--corridors`를 줄여 띠를 깊게 만드는 쪽이 효과가 크다.

**`⚠️ 주복도 N개는 X m 에 안 들어갑니다`**
띠 하나에 복도 + 양쪽 방 최소 깊이가 안 들어간다. 들어가는 만큼으로 줄여서
계속 진행하므로 그냥 둬도 되고, `--size`를 키우거나 `--corridor`를 좁혀도 된다.

**`⚠️ 아직 자유공간 N칸이 고립돼 있습니다`**
장애물이 너무 빽빽해 복구로도 못 풀었다. `--density`를 낮춘다.

**`⚠️ 출입구를 못 찾은 방 N개`**
상자가 문간을 막았다는 뜻이다. `--density`를 낮추거나 `--door`를 키운다.

**`⚠️ 바깥 땅이 너무 넓어 건물이 안 들어갑니다`**
`--yard`가 커서 남는 건물이 20 m 밑이 됐다. 들어가는 만큼으로 줄여서 계속
진행한다. 마당을 정말 넓게 쓰려면 `--size`를 같이 키운다.

**로봇이 건물 안에만 있고 밖으로 못 나간다.** `--yard 0`으로 만든 월드다
(건물이 부지를 꽉 채운다). `--yard`를 주고 다시 만든다.

**드론이 부지 밖으로 날아간다.** 울타리(`--fence-h`, 기본 2 m)가 벽(3 m)보다
낮아서 넘어간다. 막으려면 `--fence-h`를 벽보다 높게 준다.

**`WARNING: The current physics step could not be computed correctly. Your world may be
too complex...`**
물리 솔버가 한 스텝을 못 풀었다는 뜻이다. 흔한 원인은 물체가 서로 깊이 파고들어
있거나, 같은 지점에 접촉 구속이 중복으로 걸리는 경우다.

이 월드에서 실제로 겹친 바닥을 하나 찾아 고쳤다([3-2-1](#3-2-1-부지와-외부-출입구)의
"바닥은 두 겹으로 깔지 않는다"). 다만 **그게 이 경고의 원인이라는 것은 확인하지
못했다** — 아래 조건으로 헤드리스 재현을 시도했지만 네 번 다 경고 0건이었다:

| 시도 | 조건 | 결과 |
|---|---|---|
| 1 | 로봇 4대 주입, 드라이버 없음, 400스텝 | 0건 |
| 2 | 겹친 바닥 **있음** + Spot 12관절 구동, 781스텝 | 0건 |
| 3 | 겹친 바닥 **없음** + Spot 12관절 구동, 781스텝 | 0건 |
| 4 | + UGV 2대 메카넘 휠 주행, `contactProperties` 유무 비교 | 둘 다 0건 |

즉 겹친 바닥과 `contactProperties` 둘 다 이 경고의 원인은 아니다. 재현되면
Nav2·SLAM 까지 붙은 전체 스택이나 GUI 실시간 모드 쪽을 봐야 한다.
당장 급하면 Webots 가 안내하는 대로 `WorldInfo.basicTimeStep` 을 32 에서 16 으로
낮춘다(생성 월드는 32 로 나온다). 시뮬이 느려지는 대신 솔버가 안정된다.

**월드를 재로드하면 뇌들이 죽는다.** `driver` 프로세스가 종료되는데 `ros2 launch`가
되살리지 않는다. `docker compose restart`로 다시 띄운다. `fleet` 컨테이너는
`restart: unless-stopped`라 스스로 돌아온다.

---

## 참고

- [Readme.md](Readme.md) — 전체 구성, OS별 실행
- [03_SPAWNER.md](03_SPAWNER.md) — 만든 월드에 편대를 올리는 쪽 (매니페스트·기동 순서)
- [10_MAP_MERGE.md](10_MAP_MERGE.md) — 여러 로봇의 지도를 하나로 합치는 부분
- [01_INTERFACES.md](01_INTERFACES.md) — 토픽·서비스·프레임 총람
- [04_UGV_SETUP.md](04_UGV_SETUP.md) / [06_SPOT_DRIVER.md](06_SPOT_DRIVER.md) / [08_DRONE_SETUP.md](08_DRONE_SETUP.md) — 로봇별 문서

---

← [01. 인터페이스 총람](01_INTERFACES.md) | [📖 책 목차](Readme.md#-목차) | [03. 로봇 소환](03_SPAWNER.md) →
