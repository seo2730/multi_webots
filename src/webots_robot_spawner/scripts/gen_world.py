#!/usr/bin/env python3
"""넓은 작전 지역 월드를 생성한다 (창고형 대공간).

왜 가져오지 않고 만드는가:
    Webots 샘플 월드를 가져다 쓰는 경로(prepare_world.py)는 동작하지만, 큰 샘플들은
    숨은 의존성을 갖고 있어 실측에서 계속 걸렸다.

      - PROTO 기본값 컨트롤러: factory 의 valve_turner(파이썬), village 의
        generic_traffic_light x10. 월드 파일만 봐서는 안 보인다.
      - 월드 옆 캐시 디렉터리: village 의 worlds/forest/<name>/*.forest 수백 개
      - 개발 PC 의 Webots 가 factory 샘플을 **원본 그대로도** 못 연다
        (webots-bin.exe 액세스 위반)

    여기서 만드는 월드는 **plain Solid + Box 만 쓴다.** 컨트롤러도, 캐시도,
    다운로드가 필요한 PROTO 도 없다. 크기와 밀도를 원하는 대로 정할 수 있고,
    어디서 열어도 같은 결과가 나온다.

레이아웃 (창고형):
    ┌─────────────────────────────┐  외벽
    │  ← 둘레 통로 (로봇 순환) →  │
    │  ▓▓▓▓▓▓   ▓▓▓▓▓▓   ▓▓▓▓▓▓  │  선반 블록 (세그먼트)
    │  ▓▓▓▓▓▓ ▪ ▓▓▓▓▓▓ ▪ ▓▓▓▓▓▓  │  ▪ = 기둥
    │         ↑ 십자 통로          │
    └─────────────────────────────┘

실행 (호스트에 파이썬이 없어도 되게 도커로 돌린다):

    ⚠️ 현재 경로를 넘기는 문법이 셸마다 다르다. 아래는 전부 같은 뜻이다.
         cmd.exe      -v "%cd%:/w"      (줄 연결은 ^)
         PowerShell   -v "${PWD}:/w"    (줄 연결은 백틱)
         Git Bash 등  -v "$PWD:/w"      (줄 연결은 역슬래시)
       헷갈리면 절대경로를 그냥 적는 편이 확실하다 — 어느 셸에서든 동작한다:
         -v "D:/path/to/webots_multi_robot:/w"

    docker run --rm -v "$PWD:/w" -w /w windows-master python3 \\
        src/webots_robot_spawner/scripts/gen_world.py --size 100 --name warehouse100

    옵션:
      --size N        한 변 길이(m). 기본 100
      --name NAME     worlds/NAME.wbt 로 저장 (매니페스트도 같은 이름으로)
      --aisle W       통로 폭(m). 기본 4.0 — UGV 폭 0.72 + Nav2 여유
      --shelf-h H     선반 높이(m). 기본 3.0
      --no-fleet      편대 매니페스트를 만들지 않는다
      --seed N        편대 배치 후보 좌표를 흔드는 난수 시드

⚠️ 지금 **배치 자체는 무작위가 아니다.** 크기와 통로 폭이 같으면 선반과 기둥은 항상
   같은 자리에 놓인다. --seed 는 편대 배치 후보를 몇 십 cm 흔드는 데만 쓰인다.
   진짜 무작위 환경(방/복도 생성, 장애물 산포)이 필요하면 build() 의 좌표 계산만
   바꾸면 된다 — solid() 와 파일 쓰기는 그대로 쓸 수 있다.
"""

import argparse
import pathlib
import random
import sys

_HERE = pathlib.Path(__file__).resolve()
REPO = _HERE.parents[3]
WORLDS = REPO / 'src' / 'Webots-SummitXL' / 'workspace' / 'simulator' / 'worlds'
FLEET_DIR = _HERE.parents[1] / 'config' / 'fleet'

sys.path.insert(0, str(_HERE.parent))
from prepare_world import prepare  # noqa: E402  (같은 폴더의 준비 로직 재사용)

WALL_H = 4.0
WALL_T = 0.3
PILLAR = 0.5


def solid(name, x, y, z, sx, sy, sz, color, collide=True):
    """정적 장애물 하나. physics 를 안 붙이므로 고정된다.

    boundingObject 를 반드시 준다 — 없으면 LiDAR 에는 보여도 충돌하지 않아서
    로봇이 통과해 버린다.

    예외는 collide=False 로 넘기는 **순수 장식**이다. 다른 충돌체와 면이 겹치는
    슬래브에 충돌체를 또 주면, 같은 지점에서 접촉점이 두 벌 생겨 물리 솔버가
    과잉구속된다 ("physics step could not be computed correctly").
    """
    r, g, b = color
    bounding = f'''  boundingObject Box {{
    size {sx:.3f} {sy:.3f} {sz:.3f}
  }}
''' if collide else ''
    return f'''Solid {{
  translation {x:.3f} {y:.3f} {z:.3f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {r} {g} {b}
        roughness 0.9
        metalness 0
      }}
      geometry Box {{
        size {sx:.3f} {sy:.3f} {sz:.3f}
      }}
    }}
  ]
  name "{name}"
{bounding}}}'''


def build(size, aisle, shelf_h, seed):
    """월드 본문과 '확실히 빈' 좌표 목록을 만든다."""
    rng = random.Random(seed)
    half = size / 2.0
    parts = []
    free = []          # 편대 배치에 쓸 빈 좌표

    # --- 바닥 ---
    parts.append(solid('floor', 0, 0, -0.05, size, size, 0.1, (0.45, 0.45, 0.48)))

    # --- 외벽 4장 ---
    for name, x, y, sx, sy in (
        ('wall_n', 0, half, size, WALL_T),
        ('wall_s', 0, -half, size, WALL_T),
        ('wall_e', half, 0, WALL_T, size),
        ('wall_w', -half, 0, WALL_T, size),
    ):
        parts.append(solid(name, x, y, WALL_H / 2, sx, sy, WALL_H, (0.72, 0.72, 0.70)))

    # --- 선반 블록 ---
    # 둘레 통로를 남기고 안쪽에만 배치한다. 로봇이 벽을 따라 돌 수 있어야
    # 탐색이 막히지 않는다.
    margin = max(aisle * 1.5, 6.0)
    inner = half - margin
    shelf_d = 2.0                       # 선반 깊이
    row_pitch = shelf_d + aisle         # 한 줄이 차지하는 y 폭
    seg_len = 16.0                      # 선반 한 토막 길이
    cross = aisle + 1.0                 # 십자 통로 폭

    rows = int((2 * inner) // row_pitch)
    for r in range(rows):
        y = -inner + row_pitch / 2 + r * row_pitch
        if abs(y) < cross / 2:          # 가운데 십자 통로는 비운다
            continue
        x = -inner
        seg = 0
        while x + seg_len <= inner:
            # 세그먼트 사이에 통로를 둬서 줄을 가로지를 수 있게 한다
            if abs(x + seg_len / 2) > cross / 2:
                parts.append(solid(
                    f'shelf_{r}_{seg}', x + seg_len / 2, y, shelf_h / 2,
                    seg_len, shelf_d, shelf_h, (0.55, 0.42, 0.28)))
            x += seg_len + aisle
            seg += 1

    # --- 기둥 ---
    pitch = 12.0
    n = int(inner // pitch)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            px, py = i * pitch, j * pitch
            if abs(px) < cross / 2 and abs(py) < cross / 2:
                continue
            parts.append(solid(
                f'pillar_{i}_{j}', px, py, WALL_H / 2,
                PILLAR, PILLAR, WALL_H, (0.35, 0.35, 0.38)))

    # --- 빈 좌표: 둘레 통로와 십자 통로 ---
    ring = half - margin / 2            # 둘레 통로 한가운데
    free += [(-ring, -ring), (ring, -ring), (ring, ring), (-ring, ring)]
    free += [(0.0, -ring), (0.0, ring), (-ring, 0.0), (ring, 0.0)]
    # 십자 통로 위의 점들 (기둥을 피해 살짝 흔든다)
    for d in (-inner * 0.5, inner * 0.5):
        free.append((d + rng.uniform(-1, 1), 0.0))
        free.append((0.0, d + rng.uniform(-1, 1)))

    return '\n'.join(parts), free


HEADER = '''#VRML_SIM R2025a utf8
# gen_world.py 가 생성한 월드 — 직접 고치지 말고 스크립트를 고쳐 다시 생성할 것.
#
# 외부 PROTO 의존이 없다. 바닥·벽·선반·기둥이 전부 plain Solid + Box 라서
# 컨트롤러도, 다운로드도, 월드 옆 캐시 디렉터리도 필요 없다.
# (Webots 샘플 월드들이 그 셋 때문에 계속 걸렸다 — gen_world.py 주석 참고)

WorldInfo {{
  info [ "{info}" ]
  title "{title}"
  basicTimeStep 32
  # my_world.wbt 와 같은 접촉 설정. 빠뜨리면 로봇 거동이 달라진다.
  #
  # SummitXL 은 메카넘 휠이라 롤러가 45도로 누워 있다. 그 롤러 방향으로만 미끄러져야
  # 게걸음(홀로노믹)이 되는데, ContactProperties 가 없으면 Webots 가 등방성 마찰
  # (사방으로 똑같이 안 미끄러짐)을 준다. 그러면 옆으로 못 가고 앞뒤로만 간다.
  # coulombFriction [0, 2, 0] + frictionRotation 이 그 이방성을 만든다.
  #
  # softCFM 은 접촉을 아주 약간 무르게 만든다. 완전 강체 접촉은 솔버가 풀기 어렵다.
  contactProperties [
    ContactProperties {{
      material1 "InteriorWheelMat"
      coulombFriction [ 0, 2, 0 ]
      frictionRotation -0.785398 0
      bounce 0
      forceDependentSlip [ 10, 0 ]
      softCFM 0.0001
    }}
    ContactProperties {{
      material1 "ExteriorWheelMat"
      coulombFriction [ 0, 2, 0 ]
      frictionRotation 0.785398 0
      bounce 0
      forceDependentSlip [ 10, 0 ]
      softCFM 0.0001
    }}
    ContactProperties {{
      material1 "slope"
      coulombFriction [ 0.5 ]
    }}
  ]
}}
Viewpoint {{
  orientation -0.35 0.35 0.87 1.7
  position {vx:.1f} {vy:.1f} {vz:.1f}
}}
DirectionalLight {{
  direction 0.3 -0.4 -1
  intensity 3
  castShadows FALSE
}}
DirectionalLight {{
  direction -0.5 0.3 -0.6
  intensity 1.5
}}
Background {{
  skyColor [ 0.62 0.68 0.74 ]
}}
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=float, default=100.0)
    ap.add_argument('--name', default='warehouse100')
    ap.add_argument('--aisle', type=float, default=4.0)
    ap.add_argument('--shelf-h', type=float, default=3.0)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--no-fleet', action='store_true')
    args = ap.parse_args()

    body, free = build(args.size, args.aisle, args.shelf_h, args.seed)

    head = HEADER.format(
        info=f'{args.size:.0f}x{args.size:.0f} m 창고형 작전 지역 (gen_world.py 생성)',
        title=args.name,
        vx=-args.size * 0.35, vy=-args.size * 0.35, vz=args.size * 0.45)

    text = head + body + '\n'
    # supervisor 노드와 IMPORTABLE 선언은 prepare_world 의 로직을 그대로 쓴다.
    text, notes = prepare(text, args.name)

    WORLDS.mkdir(parents=True, exist_ok=True)
    out = WORLDS / f'{args.name}.wbt'
    out.write_text(text, encoding='utf-8')

    nodes = body.count('Solid {')
    print(f'월드 생성: {out}')
    print(f'  크기 {args.size:.0f} x {args.size:.0f} m, 통로 {args.aisle:.1f} m, '
          f'장애물 {nodes}개')
    for n in notes:
        print(f'  {n}')

    if args.no_fleet:
        return 0

    # --- 편대 매니페스트 ---
    # 생성기는 어디가 비었는지 알고 있으므로 좌표를 직접 써 준다.
    # 손으로 고르면 선반 안에 로봇을 놓기 쉽다.
    picks = free[:4]
    half = args.size / 2.0
    margin = max(args.aisle * 1.5, 6.0)
    area = half - margin / 2 - 1.0
    lines = [
        f'# {args.name} 월드용 편대 — gen_world.py 가 함께 생성했다.',
        '#',
        '# 좌표는 생성기가 아는 빈 곳(둘레 통로)에서 골랐다. 손으로 고르면',
        '# 선반 안에 로봇을 놓기 쉽다.',
        '#',
        f'#   ros2 launch webots_robot_spawner spawner.launch.py fleet:={args.name}.yaml',
        '',
        'fleet:',
    ]
    for (rid, rtype), (x, y) in zip(
            [('ugv1', 'ugv'), ('ugv2', 'ugv'), ('spot1', 'spot'), ('drone1', 'drone')],
            picks):
        lines.append(f'  - {{type: {rtype}, id: {rid}, '
                     f'x: {x:.2f}, y: {y:.2f}, yaw: 0.0}}')
    lines += [
        '',
        '# 무작위 배치 영역: 둘레 통로 안쪽. 맵이 생기기 전에는 로봇 간 간격만 보므로',
        '# 선반이 있는 안쪽까지 넣지 않는다.',
        f'spawn_area: [{-area:.1f}, {-area:.1f}, {area:.1f}, {area:.1f}]',
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
