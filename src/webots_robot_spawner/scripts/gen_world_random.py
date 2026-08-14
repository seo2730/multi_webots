#!/usr/bin/env python3
"""실제 건물처럼 생긴 월드를 무작위로 만든다 (복도 + 방 + 장애물).

gen_world.py 의 창고형은 **결정적**이다 — 크기가 같으면 항상 같은 배치가 나온다.
여기서는 시드마다 다른 지형이 나온다. 훈련·벤치마크용 환경을 무한히 뽑거나,
"처음 보는 곳에서 작전"을 시험할 때 쓴다.

**왜 BSP 를 버렸나** — 공간을 재귀로 쪼개는 BSP 는 구획을 만들 뿐 방을 만들지
않는다. 결과가 미로처럼 보이고, 복도가 방 중심을 잇는 통로일 뿐이라 건물로 읽히지
않았다. 실제 건물은 정반대 순서다:

    동선(복도)이 먼저 있고, 방이 거기에 면해서 줄지어 붙는다.

그래서 이 스크립트도 그 순서로 만든다:

    1. 가로 주복도를 n 개 놓는다 (층을 가로지르는 긴 복도)
    2. 세로 연결복도로 주복도들을 잇는다
    3. 복도 양옆 띠를 방으로 잘라 붙인다 — 학교 도면의 교실 줄과 같은 배치
    4. 방마다 복도 쪽 벽에 출입구를 뚫는다 (문짝은 없다)
    5. 방 안에만 상자·팔레트를 뿌린다
    6. 격자를 큰 사각형으로 합쳐 Box 로 내보낸다

이 순서 덕분에 **연결성이 구조적으로 보장된다.** 모든 방은 복도에 면하고, 모든
복도는 연결복도로 이어져 있다. 나중에 검사해서 고치는 게 아니라 만들 때부터
이어져 있다.

방 크기는 한 가지로 고정하지 않는다. 띠 깊이를 시드마다 흔들고 방 폭도 편차를
크게 줘서, 강당처럼 큰 방과 창고처럼 작은 방이 같이 나온다.

출입구에는 **문짝을 달지 않는다** — 벽이 그냥 끊긴 구간이다. 각 구간의 중앙 좌표는
config/doorways/NAME.yaml 로 따로 저장한다. 방 사이를 오가는 경로를 짤 때
이 점들을 경유지로 쓰면 벽을 긁지 않는다.

실행 (호스트에 파이썬이 없어도 되게 도커로 돌린다):

    ⚠️ 현재 경로를 넘기는 문법이 셸마다 다르다. 아래는 전부 같은 뜻이다.
         cmd.exe      -v "%cd%:/w"      (줄 연결은 ^)
         PowerShell   -v "${PWD}:/w"    (줄 연결은 백틱)
         Git Bash 등  -v "$PWD:/w"      (줄 연결은 역슬래시)
       헷갈리면 절대경로를 그냥 적는 편이 확실하다 — 어느 셸에서든 동작한다:
         -v "D:/path/to/webots_multi_robot:/w"

    docker run --rm -v "$PWD:/w" -w /w windows-master python3 \\
        src/webots_robot_spawner/scripts/gen_world_random.py \\
        --size 120 --seed 3 --name arena_s3

    옵션:
      --size N        한 변 길이(m). 기본 100
      --seed N        **시드가 다르면 다른 지형이 나온다.** 같으면 완전히 같다
      --name NAME     worlds/NAME.wbt (매니페스트도 같은 이름)
      --cell M        격자 한 칸(m). 기본 0.5. 작을수록 세밀하지만 박스가 늘어난다

      --corridors N   가로 주복도 개수. 0 = 크기에 맞게 자동
      --links N       세로 연결복도 개수. 0 = 크기에 맞게 자동
      --rooms N       방 개수 목표. 0 = 크기에 맞게 자동.
                      **개수를 정하면 폭을 거기 맞춰 나눈다** (너무 많으면 줄여서 알림)
      --room-depth M  방 깊이 기준(m). 기본 8 — 주복도 개수를 자동으로 정할 때 쓴다
      --room-min M    방 최소 폭(m). 기본 4
      --corridor M    주복도 폭(m). 기본 3 — UGV 폭 0.72 + Nav2 여유
      --link-w M      연결복도 폭(m). 기본 2.5
      --door M        출입구 폭(m). 기본 1.8

      --density D     장애물 밀도 (방 면적 100 m2 당 개수). 기본 6. 0 이면 없음
      --wall-h H      벽 높이(m). 기본 3
      --min-door M    출입구로 인정할 최소 폭(m). 기본 1.2
      --robot-w M     지나다닐 로봇 폭(m). 기본 0.72 (SummitXL). 상자를 놓을 때
                      옆으로 이만큼 + 여유를 반드시 남긴다
      --no-fleet      편대 매니페스트를 만들지 않는다
      --no-doorways   출입구 yaml 을 만들지 않는다 (검사와 복구는 그대로 한다)
"""

import argparse
import pathlib
import random
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve()
REPO = _HERE.parents[3]
WORLDS = REPO / 'src' / 'Webots-SummitXL' / 'workspace' / 'simulator' / 'worlds'
FLEET_DIR = _HERE.parents[1] / 'config' / 'fleet'
DOOR_DIR = _HERE.parents[1] / 'config' / 'doorways'

sys.path.insert(0, str(_HERE.parent))
from prepare_world import prepare              # noqa: E402
from gen_world import HEADER, solid            # noqa: E402
from gen_world_from_map import merge_rectangles  # noqa: E402

MIN_ROOM_DEPTH = 6      # 칸. 이보다 얕으면 방이 아니라 벽장이다

# 구간마다 '용도'를 뽑는다. 전부 같은 폭으로 나누면 방이 다 비슷해서 모텔처럼 보인다.
# 실제 건물은 강당·도서실 같은 큰 방과 사무실·창고 같은 작은 방이 섞여 있다.
# 숫자는 평균 대비 방 개수 배율 — 작을수록 방이 크다. hall 은 구간 전체가 방 하나.
PROGRAMS = (
    ('hall',     0.0),          # 강당/체육관 — 구간을 통째로
    ('large',    0.45),         # 도서실/식당
    ('standard', 1.00),         # 교실
    ('small',    2.10),         # 사무실/창고
)

DEEP_CELLS = 24                 # 12 m 보다 깊은 띠는 '깊다'고 본다
# 깊은 띠에 작은 방을 넣으면 폭 4 m 에 깊이 15 m 짜리 복도 같은 방이 나온다.
# 실제 건물도 깊은 베이엔 강당·도서실을, 얕은 베이엔 사무실을 넣는다.
WEIGHTS_DEEP = (0.20, 0.30, 0.42, 0.08)
WEIGHTS_SHALLOW = (0.05, 0.13, 0.52, 0.30)


def pick_program(rng, deep):
    weights = WEIGHTS_DEEP if deep else WEIGHTS_SHALLOW
    r, acc = rng.random(), 0.0
    for (name, mult), w in zip(PROGRAMS, weights):
        acc += w
        if r < acc:
            return name, mult
    return PROGRAMS[-1]


def split_widths(total, k, min_w, rng):
    """폭 total 을 k 칸으로 나눈다. 각 조각은 min_w 이상.

    균등분할하면 교실만 죽 늘어서서 심심하다. 난수를 제곱해 편차를 키워
    큰 방과 작은 방이 섞이게 한다.
    """
    if k <= 1:
        return [total]
    extra = total - k * min_w
    if extra < 0:
        return None
    w = [rng.random() ** 2 + 0.15 for _ in range(k)]
    s = sum(w)
    parts = [min_w + int(extra * v / s) for v in w]
    parts[-1] += total - sum(parts)         # 반올림 오차는 마지막이 흡수
    return parts


def plan_building(n, rng, n_corr, n_link, target_rooms,
                  cw, lw, door_w, room_min):
    """복도를 먼저 놓고 그 양옆에 방을 붙이는 건물식 배치.

    벽을 '그린다'. 빈 판에서 시작해 선을 그으면 방이 저절로 생긴다. 반대로
    벽에서 시작해 방을 파내면 벽 두께를 맞추기가 훨씬 성가시다.

    돌려주는 것: (grid, rooms, doors, corridors)
      grid       True = 벽
      rooms      [(x0, y0, x1, y1)] 방 내부 (반열림 구간)
      doors      [(방 번호, gy, gx, 폭칸수, 방향)]
      corridors  [(x0, y0, x1, y1)] 복도 내부 — 순찰 경로용
    """
    grid = np.zeros((n, n), dtype=bool)          # False = 자유공간
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = True

    # --- ① 세로 연결복도 자리부터 잡는다 (방을 자를 구간이 여기서 갈린다) ---
    links = []
    for i in range(n_link):
        c = int(round((n - 1) * (i + 1) / (n_link + 1)))
        jit = (n - 2) // (8 * (n_link + 1))
        if jit > 0:
            c += rng.randint(-jit, jit)
        a = max(2, min(n - 2 - lw, c - lw // 2))
        links.append((a, a + lw))
    links.sort()

    # --- ② 가로 주복도: 층을 몇 개의 띠로 나눈다 ---
    # 띠 높이를 시드마다 흔들어 얕은 방(사무실)과 깊은 방(강당)이 섞이게 한다
    weights = [rng.uniform(0.8, 1.35) for _ in range(n_corr)]
    tot = sum(weights)
    bounds, acc = [0], 0.0
    for w in weights:
        acc += w
        bounds.append(int(round((n - 1) * acc / tot)))
    bounds[-1] = n - 1

    # 복도 폭도 하나로 고정하지 않는다 — 넓은 주동선 하나에 좁은 복도 여럿이
    # 실제 건물의 모습이다.
    main_i = rng.randrange(n_corr)
    corridor_rows = set()
    strips = []                     # (윗방 rows, 복도벽 rows, 아랫방 rows)
    for i in range(n_corr):
        r0, r1 = bounds[i], bounds[i + 1]
        cw_i = int(round(cw * (rng.uniform(1.45, 1.9) if i == main_i
                               else rng.uniform(0.8, 1.15))))
        cw_i = max(2, cw_i)
        while cw_i > 2 and r1 - r0 - 1 - cw_i - 2 < 2 * MIN_ROOM_DEPTH:
            cw_i -= 1                           # 띠에 안 들어가면 복도를 좁힌다
        depth = r1 - r0 - 1 - cw_i - 2          # 방으로 쓸 수 있는 행 수 (양쪽 합)
        if depth < 2 * MIN_ROOM_DEPTH:
            return None                         # 띠가 너무 얇다 — 복도 수를 줄여야 한다
        # 위아래 깊이도 살짝 어긋나게 — 한쪽은 교실, 한쪽은 얕은 사무실
        d_a = int(depth * rng.uniform(0.4, 0.6))
        d_a = max(MIN_ROOM_DEPTH, min(depth - MIN_ROOM_DEPTH, d_a))
        wall_a = r0 + 1 + d_a                   # 복도 위쪽 벽
        wall_b = wall_a + 1 + cw_i              # 복도 아래쪽 벽
        corridor_rows.update(range(wall_a + 1, wall_b))
        strips.append(((r0 + 1, wall_a), wall_a, wall_b, (wall_b + 1, r1)))

    def hwall(row):
        """가로 벽 한 줄. 연결복도가 지나는 열은 터놓는다."""
        grid[row, 1:n - 1] = True
        for a, b in links:
            grid[row, a:b] = False

    for i in range(1, n_corr):                  # 띠 경계 (등 맞댄 방 사이 벽)
        hwall(bounds[i])
    for (_, wall_a, wall_b, _) in strips:
        hwall(wall_a)
        hwall(wall_b)

    # 연결복도 옆벽 — 가로 복도와 만나는 행은 터놓아야 갈아탈 수 있다
    for a, b in links:
        for row in range(1, n - 1):
            if row in corridor_rows:
                continue
            grid[row, a - 1] = True
            grid[row, b] = True

    corridors = [(1, wa + 1, n - 1, wb) for (_, wa, wb, _) in strips]
    corridors += [(a, 1, b, n - 1) for a, b in links]

    # --- ③ 방을 자를 x 구간 (연결복도 사이사이) ---
    segments, x = [], 1
    for a, b in links:
        if a - 1 > x:
            segments.append((x, a - 1))
        x = b + 1
    if n - 1 > x:
        segments.append((x, n - 1))

    rows_list = []
    for (row_a, _, _, row_b) in strips:
        rows_list += [(row_a, 'north'), (row_b, 'south')]
    # 'north' = 방이 복도 위쪽에 있다 -> 출입구는 방의 남쪽(아래) 벽

    # 구간마다 용도를 먼저 뽑아 두고, 그다음 목표 방 개수에 맞게 평균 폭을 맞춘다.
    cells = []
    for (ry0, ry1), side in rows_list:
        if ry1 - ry0 < MIN_ROOM_DEPTH:
            continue
        # 깊은 띠에서 방을 잘게 쪼개면 4 x 15 m 짜리 복도 같은 방이 나온다.
        # 폭이 깊이의 절반은 되게 묶어 방다운 비례를 지킨다.
        depth = ry1 - ry0
        min_w = max(room_min, depth // 2)
        for s0, s1 in segments:
            cells.append([(ry0, ry1), side, s0, s1,
                          pick_program(rng, depth >= DEEP_CELLS), min_w])

    frontage = sum(s1 - s0 for _, _, s0, s1, _, _ in cells)
    if target_rooms <= 0:
        target_rooms = max(len(cells), frontage // (room_min * 3))

    def k_of(cell_, avg_):
        (_, _, s0, s1, (_, mult), min_w) = cell_
        if mult <= 0:
            return 1                                      # hall — 구간 통째로
        return max(1, min(int(round((s1 - s0) / avg_ * mult)),
                          (s1 - s0) // min_w))

    # 방 개수는 평균 폭에 반비례한다. 몇 번 되먹임하면 목표에 붙는다.
    avg = max(room_min + 1, frontage / float(target_rooms))
    for _ in range(8):
        got = sum(k_of(c, avg) for c in cells)
        if got == target_rooms:
            break
        avg = max(room_min + 1.0, avg * got / float(target_rooms))

    rooms, doors = [], []
    for cell_ in cells:
        (ry0, ry1), side, s0, s1, _prog, min_w = cell_
        door_row = ry1 if side == 'north' else ry0 - 1   # 복도 쪽 벽
        width = s1 - s0
        k = k_of(cell_, avg)
        parts = split_widths(width, k, min_w, rng)
        while parts is None and k > 1:                    # 최소 폭을 못 맞추면 줄인다
            k -= 1
            parts = split_widths(width, k, min_w, rng)
        if parts is None:
            parts = [width]

        x = s0
        for j, part in enumerate(parts):
            if j > 0:
                grid[ry0:ry1, x] = True                   # 방 사이 벽
                rx0 = x + 1
            else:
                rx0 = x
            rx1 = x + part
            if rx1 - rx0 >= 2:
                idx = len(rooms)
                rooms.append((rx0, ry0, rx1, ry1))
                # 출입구: 방 폭 가운데를 door_w 만큼 튼다
                dw = min(door_w, rx1 - rx0 - 1)
                gx = (rx0 + rx1 - dw) // 2
                grid[door_row, gx:gx + dw] = False
                doors.append((idx, door_row, gx + (dw - 1) / 2.0, dw,
                              'south' if side == 'north' else 'north'))
            x = rx1

    return grid, rooms, doors, corridors


def clearance_map(free, max_k):
    """각 자유 셀이 벽에서 몇 칸 떨어졌는지 (max_k 까지만 센다).

    scipy 없이 침식을 반복해서 구한다. 장애물을 놓아도 되는 '넓은 곳'을 고르는 데 쓴다.
    """
    cl = np.zeros(free.shape, dtype=int)
    cur = free.copy()
    for k in range(1, max_k + 1):
        shrunk = (cur
                  & np.roll(cur, 1, 0) & np.roll(cur, -1, 0)
                  & np.roll(cur, 1, 1) & np.roll(cur, -1, 1))
        shrunk[0, :] = shrunk[-1, :] = shrunk[:, 0] = shrunk[:, -1] = False
        cl[shrunk] = k
        cur = shrunk
        if not cur.any():
            break
    return cl


def largest_component(free):
    """자유공간의 최대 연결 성분을 플러드 필로 찾는다 (4-이웃)."""
    seen = np.zeros(free.shape, dtype=bool)
    best = np.zeros(free.shape, dtype=bool)
    h, w = free.shape
    for sy in range(h):
        for sx in range(w):
            if not free[sy, sx] or seen[sy, sx]:
                continue
            comp = np.zeros(free.shape, dtype=bool)
            stack = [(sy, sx)]
            seen[sy, sx] = True
            while stack:
                y, x = stack.pop()
                comp[y, x] = True
                for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                    if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((ny, nx))
            if comp.sum() > best.sum():
                best = comp
    return best


def _runs(mask):
    """1차원 불리언에서 True 가 이어진 구간 [(시작, 끝+1), ...] 을 뽑는다."""
    out, s = [], None
    for i, v in enumerate(mask):
        if v and s is None:
            s = i
        elif not v and s is not None:
            out.append((s, i))
            s = None
    if s is not None:
        out.append((s, len(mask)))
    return out


def find_doorways(passable, room, cell, size, min_cells):
    """방 사각형의 네 변에서 뚫린 구간(= 출입구)을 찾는다.

    문짝은 없다. 방을 둘러싼 벽 줄에서 통과 가능한 칸이 이어진 구간이 곧 출입구다.
    벽 줄(방 바로 바깥 한 칸)을 보는 이유는, 여기가 실제로 로봇이 몸을 밀어 넣는
    가장 좁은 목이기 때문이다. 방 안쪽을 보면 이미 넓어져서 폭이 과대평가된다.

    설계상 문을 어디에 뚫었는지는 이미 알고 있지만, 여기서는 **완성된 격자를 다시
    재서** 확인한다. 상자가 목을 좁혔으면 그만큼 줄어든 폭이 나와야 맞다.
    """
    rx0, ry0, rx1, ry1 = room
    n_h, n_w = passable.shape
    half = size / 2.0
    res = []

    def world(gx, gy):
        return ((gx + 0.5) * cell - half, (gy + 0.5) * cell - half)

    for side, col, out_col in (('west', rx0 - 1, rx0 - 2), ('east', rx1, rx1 + 1)):
        if not (0 <= col < n_w and 0 <= out_col < n_w):
            continue
        # 벽 줄만 보면 '벽에 파인 홈'도 출입구로 세게 된다. 안쪽·벽줄·바깥쪽이
        # 모두 뚫린 줄만 진짜 관통로다.
        in_col = rx0 if side == 'west' else rx1 - 1
        for s, e in _runs(passable[ry0:ry1, col]
                          & passable[ry0:ry1, out_col]
                          & passable[ry0:ry1, in_col]):
            if e - s < min_cells:
                continue
            gy = ry0 + (s + e - 1) / 2.0
            x, y = world(col, gy)
            res.append({'side': side, 'x': x, 'y': y, 'width': (e - s) * cell,
                        'gx': float(col), 'gy': gy})

    for side, row, out_row in (('south', ry0 - 1, ry0 - 2), ('north', ry1, ry1 + 1)):
        if not (0 <= row < n_h and 0 <= out_row < n_h):
            continue
        in_row = ry0 if side == 'south' else ry1 - 1
        for s, e in _runs(passable[row, rx0:rx1]
                          & passable[out_row, rx0:rx1]
                          & passable[in_row, rx0:rx1]):
            if e - s < min_cells:
                continue
            gx = rx0 + (s + e - 1) / 2.0
            x, y = world(gx, row)
            res.append({'side': side, 'x': x, 'y': y, 'width': (e - s) * cell,
                        'gx': gx, 'gy': float(row)})

    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=float, default=100.0)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--name', default=None)
    ap.add_argument('--cell', type=float, default=0.5)
    ap.add_argument('--corridors', type=int, default=0)
    ap.add_argument('--links', type=int, default=0)
    ap.add_argument('--rooms', type=int, default=0)
    ap.add_argument('--room-depth', type=float, default=8.0)
    ap.add_argument('--room-min', type=float, default=4.0)
    ap.add_argument('--corridor', type=float, default=3.0)
    ap.add_argument('--link-w', type=float, default=2.5)
    ap.add_argument('--door', type=float, default=1.8)
    ap.add_argument('--density', type=float, default=6.0)
    ap.add_argument('--wall-h', type=float, default=3.0)
    ap.add_argument('--min-door', type=float, default=1.2)
    ap.add_argument('--robot-w', type=float, default=0.72)
    ap.add_argument('--no-fleet', action='store_true')
    ap.add_argument('--no-doorways', action='store_true')
    args = ap.parse_args()

    name = args.name or f'arena_s{args.seed}'
    rng = random.Random(args.seed)
    cell = args.cell
    n = int(round(args.size / cell))
    cw = max(2, int(round(args.corridor / cell)))
    lw = max(2, int(round(args.link_w / cell)))
    door_w = max(2, int(round(args.door / cell)))
    room_min = max(3, int(round(args.room_min / cell)))
    min_door = max(1, int(round(args.min_door / cell)))

    # --- 복도 개수: 지정 없으면 방 깊이 기준으로 크기에 맞게 정한다 ---
    depth = max(MIN_ROOM_DEPTH, int(round(args.room_depth / cell)))
    strip = 2 * depth + cw + 3                  # 띠 하나가 먹는 높이
    n_corr = args.corridors or max(1, int(round((n - 1) / float(strip))))
    asked_corr = n_corr
    n_corr = min(n_corr, max(1, (n - 1) // (2 * MIN_ROOM_DEPTH + cw + 3)))
    n_link = args.links or max(1, min(4, int(round(args.size / 45.0))))
    if n_corr > 1:
        n_link = max(1, n_link)                 # 복도끼리 이어져야 한다

    plan = plan_building(n, rng, n_corr, n_link, args.rooms,
                         cw, lw, door_w, room_min)
    while plan is None and n_corr > 1:          # 띠가 얇으면 복도를 줄여 다시
        n_corr -= 1
        plan = plan_building(n, rng, n_corr, n_link, args.rooms,
                             cw, lw, door_w, room_min)
    if plan is None:
        print('  ❌ 이 크기에는 복도 하나도 못 넣습니다. --size 를 키우세요')
        return 1
    if n_corr != asked_corr:
        print(f'  ⚠️ 주복도 {asked_corr}개는 {args.size:.0f} m 에 안 들어갑니다 '
              f'-> {n_corr}개로 줄였습니다')
    grid, rooms, doors, corridors = plan

    free = ~grid
    reach = largest_component(free)
    if reach.sum() < free.sum():
        # 구조상 생길 일이 아니지만, 생기면 떨어진 구역을 벽으로 되돌린다
        lost = int(free.sum() - reach.sum())
        print(f'  ⚠️ 끊긴 자유공간 {lost}칸을 벽으로 되돌렸습니다')
        grid |= free & ~reach
        free = ~grid

    # 출입구 주변은 장애물 금지 구역. 목이 좁아지면 로봇이 못 들어간다.
    keepout = np.zeros(grid.shape, dtype=bool)
    for (_, gy, gx, _, _) in doors:
        gxi, r = int(round(gx)), cw
        keepout[max(0, gy - r):gy + r + 1, max(0, gxi - r):gxi + r + 1] = True

    # --- 장애물 산포: 방 안에만 ---
    room_mask = np.zeros(grid.shape, dtype=bool)
    for (rx0, ry0, rx1, ry1) in rooms:
        room_mask[ry0:ry1, rx0:rx1] = True

    obstacles = []
    taken = np.zeros(grid.shape, dtype=bool)
    if args.density > 0:
        cl = clearance_map(free, 6)
        cl[keepout] = 0                         # 출입구 목은 후보에서 뺀다
        cl[~room_mask] = 0                      # 복도는 비워 둔다
        want = int(room_mask.sum() * cell * cell / 100.0 * args.density)
        # 상자를 놓아도 옆으로 pass 칸만큼은 남아야 한다. clearance k 인 칸에
        # 반경 half 짜리 상자를 놓으면 가장 가까운 벽까지 (k - half) 칸이 남으므로
        # half <= k - pass 로 묶는다. 이 제약이 없으면 3 m 복도 한가운데 1.5 m 상자가
        # 들어가 양옆 0.75 m 만 남고, 폭 0.72 m 로봇에겐 사실상 막힌 길이 된다.
        pass_cells = max(2, int(np.ceil((args.robot_w + 2 * 0.15) / cell)))
        cand = np.argwhere(cl >= pass_cells + 1)
        idx = list(range(len(cand)))
        rng.shuffle(idx)
        # owner[y, x] = 그 칸을 차지한 장애물 번호 (-1 = 없음).
        # 나중에 연결성을 고칠 때 **장애물 단위로** 빼야 해서 필요하다.
        owner = np.full(free.shape, -1, dtype=np.int32)
        for i in idx:
            if len(obstacles) >= want:
                break
            gy, gx = cand[i]
            k = int(cl[gy, gx])
            half = rng.randint(1, k - pass_cells)
            y0, y1 = gy - half, gy + half + 1
            x0, x1 = gx - half, gx + half + 1
            if (owner[y0 - pass_cells:y1 + pass_cells,
                      x0 - pass_cells:x1 + pass_cells] >= 0).any():
                continue
            owner[y0:y1, x0:x1] = len(obstacles)
            obstacles.append((
                (gx + 0.5) * cell - args.size / 2,
                (gy + 0.5) * cell - args.size / 2,
                (x1 - x0) * cell, (y1 - y0) * cell,
                rng.uniform(0.4, 1.2)))

        # --- 연결성 복구 ---
        # 장애물이 방 구석을 막아 주머니를 만들 수 있다. 경고만 하고 넘어가면 로봇이
        # 도달 못 하는 구역이 남으므로, 고립을 만든 장애물을 실제로 빼낸다.
        removed = set()
        for _ in range(20):
            blocked = free & (owner < 0)
            iso = blocked & ~largest_component(blocked)
            if not iso.any():
                break
            near = (iso
                    | np.roll(iso, 1, 0) | np.roll(iso, -1, 0)
                    | np.roll(iso, 1, 1) | np.roll(iso, -1, 1))
            hit = set(np.unique(owner[near & (owner >= 0)]).tolist())
            if not hit:
                break                           # 벽 때문에 고립된 것이라 손댈 수 없다
            for o in hit:
                owner[owner == o] = -1
            removed |= hit

        if removed:
            obstacles = [o for i, o in enumerate(obstacles) if i not in removed]
            print(f'  연결성 복구: 고립을 만든 장애물 {len(removed)}개 제거')

        blocked = free & (owner < 0)
        lost = int(blocked.sum() - largest_component(blocked).sum())
        if lost:
            print(f'  ⚠️ 아직 자유공간 {lost}칸이 고립돼 있습니다 (--density 를 낮추세요)')
        taken = owner >= 0

    # --- 벽 격자를 사각형으로 합치기 ---
    rects = merge_rectangles(grid, min_cells=1)

    parts = [solid('floor', 0, 0, -0.05, args.size, args.size, 0.1, (0.42, 0.44, 0.47))]
    for i, (c0, r0, c1, r1) in enumerate(rects):
        sx = (c1 - c0 + 1) * cell
        sy = (r1 - r0 + 1) * cell
        cx = (c0 + c1 + 1) / 2.0 * cell - args.size / 2
        cy = (r0 + r1 + 1) / 2.0 * cell - args.size / 2
        parts.append(solid(f'wall_{i}', cx, cy, args.wall_h / 2,
                           sx, sy, args.wall_h, (0.70, 0.70, 0.68)))

    for i, (cx, cy, sx, sy, hgt) in enumerate(obstacles):
        # 상자/팔레트 느낌으로 색을 조금 흔든다
        c = 0.45 + (i % 5) * 0.05
        parts.append(solid(f'crate_{i}', cx, cy, hgt / 2, sx, sy, hgt,
                           (c, c * 0.75, c * 0.45)))

    head = HEADER.format(
        info=f'{args.size:.0f}x{args.size:.0f} m 무작위 건물 (seed {args.seed})',
        title=name,
        vx=-args.size * 0.35, vy=-args.size * 0.35, vz=args.size * 0.45)
    text, notes = prepare(head + '\n'.join(parts) + '\n', name)

    WORLDS.mkdir(parents=True, exist_ok=True)
    out = WORLDS / f'{name}.wbt'
    out.write_text(text, encoding='utf-8')

    areas = sorted((rx1 - rx0) * (ry1 - ry0) * cell * cell for rx0, ry0, rx1, ry1 in rooms)
    free_pct = 100.0 * free.sum() / free.size
    print(f'월드 생성: {out}')
    print(f'  {args.size:.0f} x {args.size:.0f} m, 격자 {n}x{n} @ {cell} m, seed {args.seed}')
    print(f'  주복도 {n_corr}개  연결복도 {n_link}개  방 {len(rooms)}개  '
          f'장애물 {len(obstacles)}개')
    if areas:
        print(f'  방 면적 최소 {areas[0]:.0f} / 중앙 {areas[len(areas) // 2]:.0f} / '
              f'최대 {areas[-1]:.0f} m2')
    print(f'  자유공간 {free_pct:.0f}%  벽 박스 {len(rects)}개')
    for nt in notes:
        print(f'  {nt}')

    # --- 출입구 목록 ---
    # 장애물을 뺀 '실제로 지나갈 수 있는' 칸으로 다시 잰다.
    passable = free & ~taken
    per_room = [find_doorways(passable, r, cell, args.size, min_door) for r in rooms]
    blocked_rooms = [i for i, d in enumerate(per_room) if not d]
    n_doors = sum(len(d) for d in per_room)
    print(f'  출입구 {n_doors}개 (최소 폭 {args.min_door:.1f} m)')
    if blocked_rooms:
        print(f'  ⚠️ 출입구를 못 찾은 방 {len(blocked_rooms)}개 — --density 를 낮추거나 '
              f'--door 를 키워 보세요')

    if not args.no_doorways:
        DOOR_DIR.mkdir(parents=True, exist_ok=True)
        half = args.size / 2
        dlines = [
            f'# {name} 월드의 방과 출입구 — gen_world_random.py 가 함께 생성했다 '
            f'(seed {args.seed}).',
            '#',
            '# 문짝은 없다. 벽이 끊긴 구간이 곧 출입구이고, 좌표는 그 구간의 중앙이다.',
            '# 좌표계는 Webots 월드 좌표 = 병합 지도(map) 좌표와 같다 '
            '(odom_is_world_absolute).',
            '# 방 사이를 오갈 때 이 점을 경유지로 쓰면 문틀을 긁지 않는다.',
            '#',
            '# side 는 방을 기준으로 한 방향이다: west = -x, east = +x, '
            'south = -y, north = +y',
            '',
            f'world: {name}',
            f'cell: {cell}',
            f'min_door_width: {args.min_door}',
            '',
            '# 복도 중심선 — 순찰이나 광역 이동 경로를 짤 때 쓴다',
            'corridors:',
        ]
        for (cx0, cy0, cx1, cy1) in corridors:
            ax = (cx0 + cx1) / 2.0 * cell - half
            ay = (cy0 + cy1) / 2.0 * cell - half
            dlines.append(
                f'  - {{center: [{ax:.2f}, {ay:.2f}], '
                f'size: [{(cx1 - cx0) * cell:.2f}, {(cy1 - cy0) * cell:.2f}]}}')

        dlines += ['', 'rooms:']
        for i, (room, dws) in enumerate(zip(rooms, per_room)):
            rx0, ry0, rx1, ry1 = room
            cx = (rx0 + rx1) / 2.0 * cell - half
            cy = (ry0 + ry1) / 2.0 * cell - half
            dlines += [
                f'  - id: room_{i}',
                f'    center: [{cx:.2f}, {cy:.2f}]',
                f'    size: [{(rx1 - rx0) * cell:.2f}, {(ry1 - ry0) * cell:.2f}]',
                f'    area: {(rx1 - rx0) * (ry1 - ry0) * cell * cell:.1f}',
                '    doorways:' if dws else '    doorways: []',
            ]
            for d in dws:
                dlines.append(f'      - {{x: {d["x"]:.2f}, y: {d["y"]:.2f}, '
                              f'width: {d["width"]:.2f}, side: {d["side"]}}}')

        # 방마다 찾아 들어가지 않아도 되게 평평한 목록도 같이 둔다
        dlines += ['', 'doorways:']
        for i, dws in enumerate(per_room):
            for d in dws:
                dlines.append(f'  - {{room: room_{i}, x: {d["x"]:.2f}, y: {d["y"]:.2f}, '
                              f'width: {d["width"]:.2f}, side: {d["side"]}}}')
        dlines.append('')

        door_out = DOOR_DIR / f'{name}.yaml'
        door_out.write_text('\n'.join(dlines), encoding='utf-8')
        print(f'출입구 목록: {door_out}')

    if args.no_fleet:
        return 0

    # --- 편대 매니페스트: 넓고 서로 먼 자유 셀 4곳 ---
    placeable = passable.copy()
    cl2 = clearance_map(passable, 4)
    placeable &= cl2 >= 2
    ys, xs = np.nonzero(placeable)
    if len(xs) < 4:
        print('  ⚠️ 빈 자리가 부족해 편대 매니페스트는 만들지 않았습니다')
        return 0

    picks, mid = [], n / 2
    for qx, qy in ((0, 0), (1, 0), (1, 1), (0, 1)):
        sel = ((xs >= mid * qx) & (xs < mid * (qx + 1)) &
               (ys >= mid * qy) & (ys < mid * (qy + 1)))
        idx = np.nonzero(sel)[0]
        src = idx if len(idx) else np.arange(len(xs))
        j = src[len(src) // 2]
        picks.append(((xs[j] + 0.5) * cell - args.size / 2,
                      (ys[j] + 0.5) * cell - args.size / 2))

    edge = args.size / 2 - 2
    lines = [
        f'# {name} 월드용 편대 — gen_world_random.py 가 함께 생성했다 (seed {args.seed}).',
        '#',
        '# 좌표는 격자에서 벽·장애물과 떨어진 셀 중 사분면별로 하나씩 골랐다.',
        '# 지형이 시드마다 다르므로 손으로 고르면 벽 속에 로봇을 놓기 쉽다.',
        '',
        'fleet:',
    ]
    for (rid, rtype), (x, y) in zip(
            [('ugv1', 'ugv'), ('ugv2', 'ugv'), ('spot1', 'spot'), ('drone1', 'drone')],
            picks):
        lines.append(f'  - {{type: {rtype}, id: {rid}, x: {x:.2f}, y: {y:.2f}, yaw: 0.0}}')
    lines += ['', f'spawn_area: [{-edge:.1f}, {-edge:.1f}, {edge:.1f}, {edge:.1f}]', '']

    (FLEET_DIR / f'{name}.yaml').write_text('\n'.join(lines), encoding='utf-8')
    print(f'편대 매니페스트: {FLEET_DIR / (name + ".yaml")}')
    for (rid, _), (x, y) in zip(
            [('ugv1', 0), ('ugv2', 0), ('spot1', 0), ('drone1', 0)], picks):
        print(f'  {rid:7s} ({x:7.2f}, {y:7.2f})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
