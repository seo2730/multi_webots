#!/usr/bin/env python3
"""방·복도가 무작위로 생성되는 월드 (장애물 산포 포함).

gen_world.py 의 창고형은 **결정적**이다 — 크기가 같으면 항상 같은 배치가 나온다.
여기서는 시드마다 다른 지형이 나온다. 훈련·벤치마크용 환경을 무한히 뽑거나,
"처음 보는 곳에서 작전"을 시험할 때 쓴다.

만드는 방법:
    1. 격자를 전부 벽으로 채운다
    2. BSP(이진 공간 분할)로 영역을 재귀적으로 쪼개고, 잎마다 방을 파낸다
    3. 쪼갤 때마다 양쪽 자식의 방 중심을 L 자 복도로 잇는다
       -> **연결성이 구조적으로 보장된다.** 나중에 검사해서 고치는 게 아니라
          만들 때부터 이어져 있다
    4. 방마다 출입구(벽이 뚫린 구간)가 로봇 폭 이상으로 있는지 확인하고, 없으면 뚫는다
    5. 방 안쪽(벽에서 충분히 떨어진 곳)에만 상자·팔레트를 뿌린다
    6. 격자를 사각형으로 합쳐 Box 로 내보낸다
       (gen_world_from_map.py 의 병합 로직을 그대로 재사용 — 셀 하나당 박스 하나면
        Webots 가 기어간다)

장애물을 방 안쪽에만 두는 이유는 복도를 막지 않기 위해서다. 벽까지 여유(clearance)가
로봇이 지나갈 만큼 되는 셀만 후보로 삼고, 마지막에 실제로 다 이어져 있는지
플러드 필로 확인한다.

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
      --min-room M    더 이상 쪼개지 않을 최소 방 한 변(m). 기본 8
      --corridor M    복도 폭(m). 기본 3 — UGV 폭 0.72 + Nav2 여유
      --density D     장애물 밀도 (자유공간 100 m2 당 개수). 기본 6. 0 이면 없음
      --wall-h H      벽 높이(m). 기본 3
      --min-door M    출입구로 인정할 최소 폭(m). 기본 1.2 (UGV 폭 0.72 + 여유)
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


def carve_rooms(grid, rng, x0, y0, x1, y1, min_room, corridor, rooms, depth=0):
    """BSP 로 쪼개며 방을 파고, 자식들을 복도로 잇는다. 방 중심을 돌려준다.

    복도를 '쪼갠 직후 양쪽 자식 사이'에 놓기 때문에 트리 전체가 하나로 이어진다.
    나중에 연결성을 고치려 들면 훨씬 복잡해진다.

    잎에서 판 방의 사각형을 rooms 에 (x0, y0, x1, y1) 로 쌓아 둔다 — 반열림 구간이라
    내부는 [x0, x1) x [y0, y1) 이다. 나중에 출입구를 찾을 때 이 사각형이 기준이 된다.
    """
    w, h = x1 - x0, y1 - y0
    # 더 쪼개면 방이 최소 크기보다 작아지는 경우 여기서 멈춘다
    can_split_x = w >= 2 * min_room + corridor
    can_split_y = h >= 2 * min_room + corridor

    if depth < 8 and (can_split_x or can_split_y):
        # 긴 쪽을 자른다. 둘 다 가능하면 약간의 무작위를 준다.
        if can_split_x and can_split_y:
            vertical = rng.random() < (w / float(w + h))
        else:
            vertical = can_split_x

        if vertical:
            lo, hi = x0 + min_room, x1 - min_room
            cut = rng.randint(lo, hi)
            a = carve_rooms(grid, rng, x0, y0, cut, y1, min_room, corridor, rooms, depth + 1)
            b = carve_rooms(grid, rng, cut, y0, x1, y1, min_room, corridor, rooms, depth + 1)
        else:
            lo, hi = y0 + min_room, y1 - min_room
            cut = rng.randint(lo, hi)
            a = carve_rooms(grid, rng, x0, y0, x1, cut, min_room, corridor, rooms, depth + 1)
            b = carve_rooms(grid, rng, x0, cut, x1, y1, min_room, corridor, rooms, depth + 1)

        carve_corridor(grid, a, b, corridor)
        return a if rng.random() < 0.5 else b

    # --- 잎: 방 하나를 판다 ---
    pad_x = rng.randint(1, max(1, w // 6))
    pad_y = rng.randint(1, max(1, h // 6))
    rx0, ry0 = x0 + pad_x, y0 + pad_y
    rx1, ry1 = x1 - pad_x, y1 - pad_y
    if rx1 - rx0 < 2 or ry1 - ry0 < 2:      # 너무 얇으면 패딩을 포기한다
        rx0, ry0, rx1, ry1 = x0 + 1, y0 + 1, x1 - 1, y1 - 1
    grid[ry0:ry1, rx0:rx1] = False          # False = 자유공간
    rooms.append((rx0, ry0, rx1, ry1))
    return ((rx0 + rx1) // 2, (ry0 + ry1) // 2)


def carve_corridor(grid, a, b, width):
    """두 점을 L 자로 잇는다. 가로 먼저 간 뒤 세로."""
    (ax, ay), (bx, by) = a, b
    half = max(1, width // 2)
    x0, x1 = sorted((ax, bx))
    y0, y1 = sorted((ay, by))
    grid[ay - half:ay + half + 1, x0:x1 + 1] = False   # 가로 구간
    grid[y0:y1 + 1, bx - half:bx + half + 1] = False   # 세로 구간


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

    돌려주는 좌표는 그 구간의 **중앙**을 월드 좌표로 옮긴 것이다.
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


def ensure_doorway(grid, room, min_cells, width, reach):
    """출입구가 하나도 없는 방에 하나 뚫는다. 뚫었으면 True.

    BSP 구조상 모든 잎 방은 부모에서 복도 끝점이 되므로 여기까지 올 일은 거의 없다.
    다만 '거의'로 두면 로봇이 못 들어가는 방이 남을 수 있어서, 실제로 확인하고
    막혔으면 뚫는다. 네 방향으로 곧게 나가 보고 가장 가까운 자유공간 쪽을 연다.
    """
    rx0, ry0, rx1, ry1 = room
    n_h, n_w = grid.shape
    cy, cx = (ry0 + ry1) // 2, (rx0 + rx1) // 2
    half = max(1, width // 2)
    best = None                              # (거리, 방향, 자를 구간)

    # (바깥 방향, 시작 벽 줄) — 방 밖으로 한 칸씩 나가며 자유공간을 만나는지 본다
    probes = (
        ('west',  lambda d: (cy, rx0 - 1 - d)),
        ('east',  lambda d: (cy, rx1 + d)),
        ('south', lambda d: (ry0 - 1 - d, cx)),
        ('north', lambda d: (ry1 + d, cx)),
    )
    for side, at in probes:
        for d in range(reach):
            y, x = at(d)
            if not (0 < y < n_h - 1 and 0 < x < n_w - 1):
                break                        # 외벽에 닿았다 — 이쪽으로는 못 나간다
            if not grid[y, x]:               # 자유공간을 만났다
                if best is None or d < best[0]:
                    best = (d, side, at)
                break

    if best is None:
        return False

    d, side, at = best
    for k in range(d + 1):                   # 벽 줄부터 만난 자유공간까지 이어서 판다
        y, x = at(k)
        if side in ('west', 'east'):
            grid[max(1, y - half):min(n_h - 1, y + half + 1), x] = False
        else:
            grid[y, max(1, x - half):min(n_w - 1, x + half + 1)] = False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=float, default=100.0)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--name', default=None)
    ap.add_argument('--cell', type=float, default=0.5)
    ap.add_argument('--min-room', type=float, default=8.0)
    ap.add_argument('--corridor', type=float, default=3.0)
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
    min_room = max(3, int(round(args.min_room / cell)))
    corridor = max(2, int(round(args.corridor / cell)))
    min_door = max(1, int(round(args.min_door / cell)))

    # 전부 벽으로 시작해서 파낸다
    grid = np.ones((n, n), dtype=bool)
    rooms = []
    carve_rooms(grid, rng, 1, 1, n - 1, n - 1, min_room, corridor, rooms)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = True   # 외벽

    free = ~grid
    reach = largest_component(free)
    if reach.sum() < free.sum():
        # 구조상 생길 일이 아니지만, 생기면 떨어진 구역을 벽으로 되돌린다
        grid |= free & ~reach
        free = ~grid

    # --- 방마다 출입구가 있는지 확인하고, 없으면 뚫는다 ---
    opened = 0
    for room in rooms:
        if find_doorways(free, room, cell, args.size, min_door):
            continue
        if ensure_doorway(grid, room, min_door, corridor, reach=2 * min_room):
            opened += 1
    if opened:
        free = ~grid
        print(f'  출입구 없는 방 {opened}개를 뚫었습니다')

    # 출입구 주변은 장애물 금지 구역으로 잡아 둔다. 목이 좁아지면 로봇이 못 들어간다.
    keepout = np.zeros(grid.shape, dtype=bool)
    for room in rooms:
        for d in find_doorways(free, room, cell, args.size, min_door):
            gy, gx = int(round(d['gy'])), int(round(d['gx']))
            r = corridor
            keepout[max(0, gy - r):gy + r + 1, max(0, gx - r):gx + r + 1] = True

    # --- 장애물 산포 ---
    obstacles = []
    taken = np.zeros(grid.shape, dtype=bool)
    if args.density > 0:
        cl = clearance_map(free, 6)
        cl[keepout] = 0                     # 출입구 목은 후보에서 뺀다
        free_area = free.sum() * cell * cell
        want = int(free_area / 100.0 * args.density)
        # 벽에서 3칸 이상 떨어진 곳만 후보 — 복도와 문간을 막지 않는다
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
            if (owner[y0 - corridor:y1 + corridor, x0 - corridor:x1 + corridor] >= 0).any():
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
            # 고립 구역에 맞닿은 장애물을 찾는다
            near = (iso
                    | np.roll(iso, 1, 0) | np.roll(iso, -1, 0)
                    | np.roll(iso, 1, 1) | np.roll(iso, -1, 1))
            hit = set(np.unique(owner[near & (owner >= 0)]).tolist())
            if not hit:
                break                      # 벽 때문에 고립된 것이라 손댈 수 없다
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
        info=f'{args.size:.0f}x{args.size:.0f} m 무작위 방/복도 (seed {args.seed})',
        title=name,
        vx=-args.size * 0.35, vy=-args.size * 0.35, vz=args.size * 0.45)
    text, notes = prepare(head + '\n'.join(parts) + '\n', name)

    WORLDS.mkdir(parents=True, exist_ok=True)
    out = WORLDS / f'{name}.wbt'
    out.write_text(text, encoding='utf-8')

    free_pct = 100.0 * free.sum() / free.size
    print(f'월드 생성: {out}')
    print(f'  {args.size:.0f} x {args.size:.0f} m, 격자 {n}x{n} @ {cell} m, seed {args.seed}')
    print(f'  자유공간 {free_pct:.0f}%  벽 박스 {len(rects)}개  장애물 {len(obstacles)}개')
    for nt in notes:
        print(f'  {nt}')

    # --- 출입구 목록 ---
    # 장애물을 뺀 '실제로 지나갈 수 있는' 칸으로 다시 잰다. 상자가 목을 좁히면
    # 그만큼 좁은 폭이 나와야 맞다.
    passable = free & ~taken
    per_room = [find_doorways(passable, r, cell, args.size, min_door) for r in rooms]
    blocked = [i for i, d in enumerate(per_room) if not d]
    n_doors = sum(len(d) for d in per_room)
    print(f'  방 {len(rooms)}개, 출입구 {n_doors}개 (최소 폭 {args.min_door:.1f} m)')
    if blocked:
        print(f'  ⚠️ 출입구를 못 찾은 방 {len(blocked)}개 — --density 를 낮추거나 '
              f'--min-door 를 줄여 보세요')

    if not args.no_doorways:
        DOOR_DIR.mkdir(parents=True, exist_ok=True)
        dlines = [
            f'# {name} 월드의 방과 출입구 — gen_world_random.py 가 함께 생성했다 '
            f'(seed {args.seed}).',
            '#',
            '# 문짝은 없다. 벽이 끊긴 구간이 곧 출입구이고, 좌표는 그 구간의 중앙이다.',
            '# 좌표계는 Webots 월드 좌표 = 병합 지도(map) 좌표와 같다 '
            '(odom_is_world_absolute).',
            '# 방 사이를 오갈 때 이 점을 경유지로 쓰면 문틀을 긁지 않는다.',
            '#',
            f'# side 는 방을 기준으로 한 방향이다: west = -x, east = +x, '
            f'south = -y, north = +y',
            '',
            f'world: {name}',
            f'cell: {cell}',
            f'min_door_width: {args.min_door}',
            '',
            'rooms:',
        ]
        for i, (room, doors) in enumerate(zip(rooms, per_room)):
            rx0, ry0, rx1, ry1 = room
            cx = (rx0 + rx1) / 2.0 * cell - args.size / 2
            cy = (ry0 + ry1) / 2.0 * cell - args.size / 2
            dlines += [
                f'  - id: room_{i}',
                f'    center: [{cx:.2f}, {cy:.2f}]',
                f'    size: [{(rx1 - rx0) * cell:.2f}, {(ry1 - ry0) * cell:.2f}]',
                '    doorways:' if doors else '    doorways: []',
            ]
            for d in doors:
                dlines.append(f'      - {{x: {d["x"]:.2f}, y: {d["y"]:.2f}, '
                              f'width: {d["width"]:.2f}, side: {d["side"]}}}')

        # 방마다 찾아 들어가지 않아도 되게 평평한 목록도 같이 둔다
        dlines += ['', 'doorways:']
        for i, doors in enumerate(per_room):
            for d in doors:
                dlines.append(f'  - {{room: room_{i}, x: {d["x"]:.2f}, y: {d["y"]:.2f}, '
                              f'width: {d["width"]:.2f}, side: {d["side"]}}}')
        dlines.append('')

        door_out = DOOR_DIR / f'{name}.yaml'
        door_out.write_text('\n'.join(dlines), encoding='utf-8')
        print(f'출입구 목록: {door_out}')

    if args.no_fleet:
        return 0

    # --- 편대 매니페스트: 넓고 서로 먼 자유 셀 4곳 ---
    placeable = free.copy()
    if args.density > 0:
        cl2 = clearance_map(free, 4)
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

    half = args.size / 2 - 2
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
    lines += ['', f'spawn_area: [{-half:.1f}, {-half:.1f}, {half:.1f}, {half:.1f}]', '']

    (FLEET_DIR / f'{name}.yaml').write_text('\n'.join(lines), encoding='utf-8')
    print(f'편대 매니페스트: {FLEET_DIR / (name + ".yaml")}')
    for (rid, _), (x, y) in zip(
            [('ugv1', 0), ('ugv2', 0), ('spot1', 0), ('drone1', 0)], picks):
        print(f'  {rid:7s} ({x:7.2f}, {y:7.2f})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
