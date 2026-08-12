#!/usr/bin/env python3
"""점유격자(SLAM 맵)를 Webots 월드로 바꾼다.

로봇이 만든 맵으로 월드를 되만들거나, 실제 건물 도면을 시뮬레이션에 들여올 때 쓴다.

이 프로젝트에서 유독 유리한 점이 하나 있다 — **맵 좌표가 곧 Webots 월드 좌표다.**
world -> {ns}/map 이 항등변환이라(MAP_MERGE.md 2장) 좌표 정합이라는 제일 성가신
단계가 통째로 없다. 맵의 (x, y)를 그대로 translation 에 넣으면 된다.

핵심 작업은 **셀을 사각형으로 합치는 것**이다. 100x100 m 를 0.1 m 로 나누면 100만 셀,
점유 셀만 해도 수만 개다. 셀 하나당 Box 하나를 만들면 Webots 가 기어간다.
가로로 이어붙인 뒤 같은 구간이 세로로 반복되면 다시 합치는 방식으로 보통 수백 개까지
줄어든다 (--stats 로 몇 배 줄었는지 볼 수 있다).

입력은 nav2 map_server 형식이다 (map_saver_cli 가 뽑는 그것):

    ros2 run nav2_map_server map_saver_cli -t /map_merged -f maps/scan
    docker run --rm -v "$PWD:/w" -w /w windows-master python3 \\
        src/webots_robot_spawner/scripts/gen_world_from_map.py \\
        --map maps/scan.yaml --name scan_world

    ⚠️ 현재 경로를 넘기는 문법이 셸마다 다르다. 아래는 전부 같은 뜻이다.
         cmd.exe      -v "%cd%:/w"      (줄 연결은 ^)
         PowerShell   -v "${PWD}:/w"    (줄 연결은 백틱)
         Git Bash 등  -v "$PWD:/w"      (줄 연결은 역슬래시)
       헷갈리면 절대경로를 그냥 적는 편이 확실하다 — 어느 셸에서든 동작한다:
         -v "D:/path/to/webots_multi_robot:/w"

잃는 것을 알고 써야 한다:
  의미   의자도 소파도 냉장고도 전부 무명 박스가 된다
  높이   점유격자는 2D 다. --height 로 가정할 수밖에 없다
  미관측 LiDAR 평면 밖의 것(유리, 낮은 턱, 천장 구조물)은 아예 안 생긴다
  미탐색 -1 셀을 벽으로 볼지 빈 곳으로 볼지는 --unknown 으로 정한다. 둘 다 틀릴 수 있다

그래서 이건 원본 월드의 대체가 아니라 **왕복 검증 도구**로 보는 게 맞다 —
로봇이 만든 맵으로 월드를 되만들어 항법이 같게 동작하는지 보는 용도.
"""

import argparse
import pathlib
import re
import sys

import numpy as np
import yaml

_HERE = pathlib.Path(__file__).resolve()
REPO = _HERE.parents[3]
WORLDS = REPO / 'src' / 'Webots-SummitXL' / 'workspace' / 'simulator' / 'worlds'
FLEET_DIR = _HERE.parents[1] / 'config' / 'fleet'

sys.path.insert(0, str(_HERE.parent))
from prepare_world import prepare  # noqa: E402
from gen_world import HEADER, solid  # noqa: E402  (같은 표현을 쓰려고 재사용)


def read_pgm(path):
    """P5(이진) / P2(아스키) PGM 을 numpy 배열로 읽는다.

    PIL 을 쓰지 않는 이유: 이 스크립트는 맨 파이썬 + numpy 로 돌아야 컨테이너 이미지를
    가리지 않는다. PGM 은 헤더가 단순해서 직접 읽는 편이 의존성을 늘리는 것보다 낫다.
    """
    data = path.read_bytes()

    # 헤더 토큰 3~4개를 읽는다. 주석(#)은 건너뛴다.
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i] != 0x0A:
                i += 1
            continue
        start = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1  # 헤더 뒤 공백 하나

    if magic == b'P5':
        dtype = np.uint8 if maxval < 256 else '>u2'
        arr = np.frombuffer(data, dtype=dtype, count=width * height, offset=i)
    elif magic == b'P2':
        arr = np.array(data[i:].split(), dtype=int)[:width * height]
    else:
        raise SystemExit(f'{path}: P5/P2 PGM 이 아닙니다 ({magic!r})')
    return arr.reshape(height, width), maxval


def occupancy_mask(img, maxval, meta, unknown):
    """PGM 픽셀을 '장애물인가' 불리언 배열로 바꾼다 (nav2 규약).

    negate=0 이면 어두울수록 점유다: p = (max - value) / max
    """
    p = img.astype(float) / maxval
    if not int(meta.get('negate', 0)):
        p = 1.0 - p
    free_thresh = float(meta.get('free_thresh', 0.25))
    occ = p > float(meta.get('occupied_thresh', 0.65))
    free = p < free_thresh
    unk = ~occ & ~free

    # ⚠️ map_saver 는 미탐색을 픽셀값 205 로 쓴다. 그 값은 p = (255-205)/255 = 0.196 인데,
    # 고전 ROS 의 free_thresh 기본값이 정확히 0.196 이라 "미탐색"으로 걸리게 돼 있었다.
    # nav2 기본값은 0.25 라 **205 가 빈 곳으로 분류된다.** 임계값을 그대로 따르는 것이
    # 규약상 맞지만, 그러면 --unknown 옵션이 조용히 아무 일도 안 한 것처럼 보인다.
    # 그래서 그런 맵이면 알려 준다.
    classic_unknown = int(((p > 0.19) & (p < 0.21) & free).sum())
    if classic_unknown > 0 and free_thresh > 0.196:
        print(f'  ⚠️ 미탐색으로 보이는 픽셀 {classic_unknown}개가 free_thresh='
              f'{free_thresh} 때문에 "빈 곳"으로 분류됐습니다. '
              '미탐색으로 다루려면 맵 YAML 의 free_thresh 를 0.196 으로 낮추세요.')

    if unknown == 'occupied':
        occ = occ | unk
    return occ


def merge_rectangles(mask, min_cells):
    """점유 셀을 사각형으로 합친다.

    가로로 최대한 이어붙인 뒤, 바로 아래 줄이 **같은 구간**이면 계속 아래로 늘린다.
    셀 하나당 박스 하나를 만드는 것보다 보통 수십~수백 배 적어진다.

    Returns: [(col0, row0, col1, row1), ...]  (양끝 포함)
    """
    h, w = mask.shape
    used = np.zeros_like(mask)
    rects = []
    for y in range(h):
        x = 0
        while x < w:
            if not mask[y, x] or used[y, x]:
                x += 1
                continue
            x2 = x
            while x2 + 1 < w and mask[y, x2 + 1] and not used[y, x2 + 1]:
                x2 += 1
            y2 = y
            while (y2 + 1 < h
                   and mask[y2 + 1, x:x2 + 1].all()
                   and not used[y2 + 1, x:x2 + 1].any()):
                y2 += 1
            used[y:y2 + 1, x:x2 + 1] = True
            if (x2 - x + 1) * (y2 - y + 1) >= min_cells:
                rects.append((x, y, x2, y2))
            x = x2 + 1
    return rects


def cell_to_world(col0, row0, col1, row1, res, ox, oy, rows):
    """셀 사각형을 world 좌표의 (중심, 크기)로 바꾼다.

    PGM 은 위쪽이 0행인데 ROS 맵은 왼쪽아래가 origin 이라 행을 뒤집어야 한다.
    이걸 놓치면 맵이 상하로 뒤집힌 월드가 나온다.
    """
    sx = (col1 - col0 + 1) * res
    sy = (row1 - row0 + 1) * res
    cx = ox + (col0 + col1 + 1) / 2.0 * res
    cy = oy + (rows - 1 - (row0 + row1) / 2.0 + 0.5) * res
    return cx, cy, sx, sy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='nav2 map YAML (map_saver_cli 출력)')
    ap.add_argument('--name', required=True)
    ap.add_argument('--height', type=float, default=2.5, help='장애물 높이(m)')
    ap.add_argument('--unknown', choices=['free', 'occupied'], default='free',
                    help='미탐색(-1) 셀 취급. 기본 free')
    ap.add_argument('--min-cells', type=int, default=2,
                    help='이보다 작은 조각은 버린다 (SLAM 잡음 제거)')
    ap.add_argument('--downsample', type=int, default=1,
                    help='N x N 셀을 하나로 묶어 해상도를 낮춘다 (박스 수 감소)')
    ap.add_argument('--no-fleet', action='store_true')
    ap.add_argument('--stats', action='store_true')
    args = ap.parse_args()

    map_yaml = pathlib.Path(args.map)
    if not map_yaml.is_absolute():
        map_yaml = REPO / map_yaml
    if not map_yaml.is_file():
        raise SystemExit(f'맵 YAML 을 찾을 수 없습니다: {map_yaml}')
    meta = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))

    img_path = pathlib.Path(meta['image'])
    if not img_path.is_absolute():
        img_path = map_yaml.parent / img_path
    if not img_path.is_file():
        raise SystemExit(f'맵 이미지를 찾을 수 없습니다: {img_path}')

    img, maxval = read_pgm(img_path)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    mask = occupancy_mask(img, maxval, meta, args.unknown)

    if args.downsample > 1:
        n = args.downsample
        h = mask.shape[0] // n * n
        w = mask.shape[1] // n * n
        # 묶음 안에 점유 셀이 하나라도 있으면 점유로 본다 (장애물을 지우지 않는 쪽)
        mask = mask[:h, :w].reshape(h // n, n, w // n, n).any(axis=(1, 3))
        res *= n

    rows, cols = mask.shape
    occupied = int(mask.sum())
    rects = merge_rectangles(mask, args.min_cells)

    print(f'맵: {img_path.name}  {cols} x {rows} 셀 @ {res:.3f} m'
          f'  ({cols * res:.1f} x {rows * res:.1f} m)')
    print(f'  점유 셀 {occupied} -> 박스 {len(rects)}개'
          + (f'  ({occupied / max(len(rects), 1):.0f}배 압축)' if rects else ''))
    if not rects:
        raise SystemExit('점유 셀이 없습니다. --unknown occupied 를 고려하세요.')

    # --- 월드 조립 ---
    parts = []
    world_w, world_h = cols * res, rows * res
    parts.append(solid('floor', ox + world_w / 2, oy + world_h / 2, -0.05,
                       world_w, world_h, 0.1, (0.45, 0.45, 0.48)))
    for i, (c0, r0, c1, r1) in enumerate(rects):
        cx, cy, sx, sy = cell_to_world(c0, r0, c1, r1, res, ox, oy, rows)
        parts.append(solid(f'obs_{i}', cx, cy, args.height / 2,
                           sx, sy, args.height, (0.60, 0.58, 0.55)))

    head = HEADER.format(
        info=f'{img_path.name} 점유격자에서 생성 (gen_world_from_map.py)',
        title=args.name,
        vx=ox + world_w * 0.15, vy=oy + world_h * 0.15,
        vz=max(world_w, world_h) * 0.5)
    text, notes = prepare(head + '\n'.join(parts) + '\n', args.name)

    WORLDS.mkdir(parents=True, exist_ok=True)
    out = WORLDS / f'{args.name}.wbt'
    out.write_text(text, encoding='utf-8')
    print(f'월드 생성: {out}')
    for n in notes:
        print(f'  {n}')

    if args.stats:
        areas = [(c1 - c0 + 1) * (r1 - r0 + 1) for c0, r0, c1, r1 in rects]
        print(f'  박스 크기(셀): 최소 {min(areas)}  중앙값 {int(np.median(areas))}  '
              f'최대 {max(areas)}')

    if args.no_fleet:
        return 0

    # --- 편대 매니페스트: 맵에서 확실히 빈 곳을 고른다 ---
    # 장애물에서 충분히 떨어진 셀만 후보로 둔다. 맵 기반이라 gen_world 의
    # "설계상 빈 곳"보다 근거가 낫다.
    clear = ~mask
    need = max(1, int(round(1.0 / res)))     # 사방 1 m 여유
    ok = clear.copy()
    for dy in range(-need, need + 1):
        for dx in range(-need, need + 1):
            ok &= np.roll(np.roll(clear, dy, axis=0), dx, axis=1)
    ok[:need, :] = ok[-need:, :] = False
    ok[:, :need] = ok[:, -need:] = False

    ys, xs = np.nonzero(ok)
    if len(xs) < 4:
        print('  ⚠️ 여유 있는 빈 자리가 부족해 편대 매니페스트는 만들지 않았습니다')
        return 0

    # 서로 멀리 떨어지도록 사분면에서 하나씩 고른다
    picks = []
    midx, midy = cols / 2, rows / 2
    for qx, qy in ((0, 0), (1, 0), (1, 1), (0, 1)):
        sel = ((xs >= midx * qx) & (xs < midx * (qx + 1) if qx == 0 else xs >= midx)
               & (ys >= midy * qy) & (ys < midy * (qy + 1) if qy == 0 else ys >= midy))
        idx = np.nonzero(sel)[0]
        src = idx if len(idx) else np.arange(len(xs))
        j = src[len(src) // 2]
        cx = ox + (xs[j] + 0.5) * res
        cy = oy + (rows - 1 - ys[j] + 0.5) * res
        picks.append((cx, cy))

    lines = [
        f'# {args.name} 월드용 편대 — gen_world_from_map.py 가 함께 생성했다.',
        '#',
        f'# 원본 맵: {img_path.name} ({cols}x{rows} @ {res:.3f} m)',
        '# 좌표는 맵에서 사방 1 m 가 비어 있는 셀 중에서 사분면별로 하나씩 골랐다.',
        '',
        'fleet:',
    ]
    for (rid, rtype), (x, y) in zip(
            [('ugv1', 'ugv'), ('ugv2', 'ugv'), ('spot1', 'spot'), ('drone1', 'drone')],
            picks):
        lines.append(f'  - {{type: {rtype}, id: {rid}, x: {x:.2f}, y: {y:.2f}, yaw: 0.0}}')
    lines += [
        '',
        f'spawn_area: [{ox + res:.1f}, {oy + res:.1f}, '
        f'{ox + world_w - res:.1f}, {oy + world_h - res:.1f}]',
        '',
    ]
    fleet_out = FLEET_DIR / f'{args.name}.yaml'
    fleet_out.write_text('\n'.join(lines), encoding='utf-8')
    print(f'편대 매니페스트: {fleet_out}')
    for (rid, _), (x, y) in zip(
            [('ugv1', 0), ('ugv2', 0), ('spot1', 0), ('drone1', 0)], picks):
        print(f'  {rid:7s} ({x:7.2f}, {y:7.2f})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
