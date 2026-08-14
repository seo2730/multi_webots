# 월드 생성 (World Generation)

로봇이 작전할 **환경**을 만드는 방법을 모았다. 로봇을 그 안에 올리는 이야기는
[Readme 12. 로봇 소환](Readme.md#12-로봇-소환-runtime-spawn)에 있다.

- [1. 어떤 방법을 고를까](#1-어떤-방법을-고를까)
- [2. OS별 실행 방법 (중요)](#2-os별-실행-방법-중요)
- [3. 무작위 방·복도 생성 (`gen_world_random.py`)](#3-무작위-방복도-생성-gen_world_randompy)
  - [3-1. 옵션](#3-1-옵션)
  - [3-2. 어떻게 만들어지나](#3-2-어떻게-만들어지나)
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

---

## 2. OS별 실행 방법 (중요)

스크립트는 파이썬 + numpy를 쓴다. **호스트에 파이썬을 깔 필요는 없다** — 프로젝트
도커 이미지 안에서 돌리면 된다. 대신 **"현재 폴더"를 컨테이너에 넘기는 문법이 셸마다
다르다.** 여기서 두 번 막힌 적이 있으니 자기 셸의 줄을 그대로 복사해 쓰는 게 좋다.

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
# 우분투/맥 기준. 다른 셸은 2장 참고
docker run --rm -v "$PWD:/w" -w /w windows-master python3 \
  src/webots_robot_spawner/scripts/gen_world_random.py --size 100 --seed 3 --name arena_s3
```

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
  주복도 5개  연결복도 2개  방 73개  장애물 218개
  방 면적 최소 20 / 중앙 81 / 최대 290 m2
  자유공간 89%  벽 박스 231개
  출입구 73개 (최소 폭 1.2 m)
출입구 목록: .../config/doorways/arena_s3.yaml
편대 매니페스트: .../config/fleet/arena_s3.yaml
  ugv1    ( -25.25,  -23.75)
  ugv2    (  35.75,  -22.75)
  spot1   (   3.25,   23.75)
  drone1  ( -14.75,   23.25)
```

### 3-1. 옵션

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--size N` | 100 | 한 변 길이(m) |
| `--seed N` | 1 | **시드가 다르면 지형이 다르다.** 같으면 완전히 같다 |
| `--name NAME` | `arena_s{seed}` | 월드·매니페스트·출입구 파일 이름 |
| `--cell M` | 0.5 | 격자 한 칸(m). 작을수록 세밀하지만 박스가 늘어 Webots가 느려진다 |
| **`--corridors N`** | 0 = 자동 | **가로 주복도 개수.** 크기에 안 들어가면 줄이고 알려준다 |
| **`--links N`** | 0 = 자동 | **세로 연결복도 개수.** 주복도들을 잇는다 |
| **`--rooms N`** | 0 = 자동 | **방 개수 목표.** 정하면 폭을 거기 맞춰 나눈다 (근사값) |
| `--room-depth M` | 8 | 방 깊이 기준(m). 주복도 개수를 자동으로 정할 때 쓴다 |
| `--room-min M` | 4 | 방 최소 폭(m) |
| `--corridor M` | 3 | 주복도 기준 폭(m). 실제 폭은 복도마다 흔들린다 |
| `--link-w M` | 2.5 | 연결복도 폭(m) |
| `--door M` | 1.8 | 출입구 폭(m) |
| `--density D` | 6 | 장애물 밀도 (**방 면적** 100 m² 당 개수). `0`이면 빈 방 |
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
| 넓은 방 위주 (창고·강당) | `--corridors 3 --rooms 40` |
| 잘게 나뉜 사무실 층 | `--corridors 8 --rooms 150` |
| 복도만 있는 빈 건물 | `--density 0` |
| 문을 더 넓게 (큰 로봇) | `--door 2.5 --robot-w 1.2` |

### 3-2. 어떻게 만들어지나

**BSP는 쓰지 않는다.** 공간을 재귀로 쪼개는 BSP는 구획을 만들 뿐 방을 만들지
않는다. 결과가 미로처럼 보이고 복도는 방 중심을 잇는 통로에 그쳐서, 건물로 읽히지
않았다. 실제 건물은 정반대 순서다 — **동선(복도)이 먼저 있고, 방이 거기에 면해서
줄지어 붙는다.** 그래서 이 스크립트도 그 순서로 만든다.

```
1. 가로 주복도를 n 개 놓는다 (층을 가로지르는 긴 복도)
2. 세로 연결복도로 주복도들을 잇는다
3. 복도 양옆 띠를 방으로 잘라 붙인다 — 학교 도면의 교실 줄과 같은 배치
4. 방마다 복도 쪽 벽에 출입구를 뚫는다 (문짝은 없다)
5. 방 안에만 상자·팔레트를 뿌린다 (복도는 비워 둔다)
6. 격자를 큰 사각형으로 합쳐 Box 로 내보낸다
```

이 순서 덕분에 **연결성이 구조적으로 보장된다.** 모든 방은 복도에 면하고, 모든
복도는 연결복도로 이어져 있다. 나중에 검사해서 고치는 게 아니라 만들 때부터
이어져 있다.

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

# 복도 중심선 — 순찰이나 광역 이동 경로를 짤 때 쓴다
corridors:
  - {center: [0.00, -37.75], size: [99.00, 3.00]}
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

기본값 + 파라미터 스윕(복도 3/8개, 방 40/150개, 60 m, 150 m), **7개 구성 전부 통과**:

- 출입구 전부(33~143개) 반경 0.36 m 로봇이 통과 가능
- 자유공간 고립 0칸 — 모든 통행 가능 칸이 하나로 이어져 있다
- 방 100% 도달 가능 (33~143개)

Webots 헤드리스 로드도 확인했다 — 생성 월드는 **오류 없이** 뜨고, 매니페스트의
로봇 4대가 그 좌표에 정상 주입된다.

평면도가 실제로 건물처럼 읽히는지는 `.wbt`의 Box를 PNG로 렌더링해 눈으로 확인했다.
숫자만 보면 "방 120개"라도 전부 같은 크기일 수 있다 — 실제로 처음 판은 그래서
모텔처럼 보였고, 그걸 보고 용도 배합을 넣었다.

**확인하지 않은 것:**

- 물리 스텝을 오래 돌렸을 때 로봇이 밀리는지 — 노드가 많아 소프트웨어 렌더링에서
  느려 시간 안에 못 끝냈다. 스폰 지점 여유는 격자 기반으로만 확인했다
- Nav2가 이 지형에서 실제로 경로를 뽑는지 (탐사·주행 시험은 아직)
- 드론이 벽 높이 3 m 위로 넘어다닐 때의 거동

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
[기동 순서](Readme.md#12-3-1-기동-순서-왜-드론만-다른가)를 지키는 중이다.

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
PowerShell이면 `${PWD}`. [2장](#2-os별-실행-방법-중요) 참고.

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

**월드를 재로드하면 뇌들이 죽는다.** `driver` 프로세스가 종료되는데 `ros2 launch`가
되살리지 않는다. `docker compose restart`로 다시 띄운다. `fleet` 컨테이너는
`restart: unless-stopped`라 스스로 돌아온다.

---

## 참고

- [Readme.md](Readme.md) — 전체 구성, 로봇 소환, OS별 실행
- [MAP_MERGE.md](MAP_MERGE.md) — 여러 로봇의 지도를 하나로 합치는 부분
- [drone_setup.md](drone_setup.md) — 드론 기체 구성
