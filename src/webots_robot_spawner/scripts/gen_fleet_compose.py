#!/usr/bin/env python3
"""편대 매니페스트를 읽어 로봇별 docker compose 서비스를 생성한다.

왜 필요한가:
    로봇의 뇌(driver + SLAM + Nav2)를 fleet 컨테이너 하나에 몰아넣으면 `cpuset` 으로
    로봇별 코어를 못 박을 수 없다. 컨테이너를 나눠도 CPU 총량이 늘지는 않지만
    (제한이 없으면 컨테이너는 호스트 코어를 그냥 공유한다), **코어 고정·자원 상한·
    장애 격리·개별 재시작**은 컨테이너 경계가 있어야 가능하다.

    그렇다고 compose 를 손으로 유지하면 매니페스트와 이중 관리가 된다. 그래서
    매니페스트를 단일 진실로 두고 compose 를 여기서 뽑아낸다.

무엇을 하는가:
    각 플랫폼 compose 파일의 마커 사이를 로봇별 서비스로 갈아 끼운다.

        # >>> FLEET GENERATED — gen_fleet_compose.py 가 관리한다. 직접 고치지 말 것
        ...로봇별 서비스...
        # <<< FLEET GENERATED

    마커 밖은 건드리지 않으므로 master / fleet 서비스와 주석은 그대로 남는다.

역할 분담:
    몸  : 소환기(fleet 컨테이너)가 매니페스트대로 Webots 에 주입
    뇌  : 여기서 생성한 로봇별 컨테이너
    추가: 매니페스트에 없는 런타임 소환은 fleet 컨테이너가 뇌까지 담당

    그래서 생성된 compose 에서는 fleet 서비스에 `manifest_brains:=false` 가 붙는다.

실행 (호스트에 파이썬이 없어도 되게 도커로 돌린다 — 크로스 플랫폼 전제):

    docker run --rm -v "$PWD:/w" -w /w python:3.11-slim \\
        python3 src/webots_robot_spawner/scripts/gen_fleet_compose.py --fleet default.yaml

    옵션:
      --fleet NAME      config/fleet/ 안의 매니페스트 (기본 default.yaml)
      --platform LIST   쉼표 구분 (기본 windows,ubuntu,mac)
      --cpus N          로봇별 CPU 상한. 생략하면 상한 없음
      --check           파일을 고치지 않고 최신인지만 확인 (CI 용, 다르면 종료코드 1)
"""

import argparse
import pathlib
import re
import sys

import yaml

_HERE = pathlib.Path(__file__).resolve()
# scripts / webots_robot_spawner / src / <repo>
REPO = _HERE.parents[3]
FLEET_DIR = _HERE.parents[1] / 'config' / 'fleet'

BEGIN = '# >>> FLEET GENERATED — gen_fleet_compose.py 가 관리한다. 직접 고치지 말 것'
END = '# <<< FLEET GENERATED'

# 플랫폼별로 다른 것만 여기 모은다. 나머지는 compose 의 공통 앵커에서 상속된다.
PLATFORMS = {
    'windows': {'anchor': 'ros-common', 'suffix': 'windows',
                'display': '${DISPLAY:-host.docker.internal:0}'},
    'ubuntu': {'anchor': 'ros-common', 'suffix': 'ubuntu',
               'display': '${DISPLAY:-:0}'},
    'mac': {'anchor': 'mac-common', 'suffix': 'mac', 'display': None},
}

# 로봇 종류 -> (런치 패키지, 런치 파일, 동기화 필요, DEF 필요)
# robot_types.py 와 일치해야 한다. 여기서 import 하지 않는 이유: 이 스크립트는
# ROS 2 없이 맨 파이썬으로 돌아야 하고(도커 python:slim 이미지), robot_types 는
# 그 자체로는 ROS 의존이 없지만 패키지 경로를 맞추는 번거로움이 있어서다.
TYPES = {
    'ugv': ('webots_python', 'single_ugv.launch.py', False, False),
    'spot': ('webots_spot', 'single_spot_launch.py', False, True),
    'drone': ('webots_python', 'single_drone.launch.py', True, False),
}


def expand(manifest_path):
    """매니페스트를 (robot_id, type, x, y, yaw) 목록으로 편다.

    count>1 이거나 random 인 항목은 **건너뛴다.** 이름과 자리가 런타임에 정해지므로
    compose 에 미리 적을 수 없다 — 그런 로봇은 fleet 컨테이너가 뇌까지 담당한다.
    """
    data = yaml.safe_load(open(manifest_path, encoding='utf-8'))
    entries = data['fleet'] if isinstance(data, dict) else data
    out, skipped = [], []
    for i, e in enumerate(entries):
        if not isinstance(e, dict) or 'type' not in e:
            skipped.append(f'#{i} (형식 오류)')
            continue
        if e['type'] not in TYPES:
            skipped.append(f"#{i} (모르는 종류 {e['type']})")
            continue
        if int(e.get('count', 1)) != 1 or e.get('random'):
            skipped.append(f"#{i} {e['type']} (이름/자리가 런타임 결정)")
            continue
        if not e.get('id'):
            skipped.append(f"#{i} {e['type']} (id 없음)")
            continue
        out.append((e['id'], e['type'], float(e.get('x', 0.0)),
                    float(e.get('y', 0.0)), float(e.get('yaw', 0.0))))
    return out, skipped


def service_block(robot, plat, cpus):
    robot_id, rtype, x, y, yaw = robot
    pkg, launch, needs_sync, needs_def = TYPES[rtype]
    cfg = PLATFORMS[plat]
    lines = [
        f'  {robot_id}:',
        f'    <<: *{cfg["anchor"]}',
        f'    container_name: {robot_id}_brain_{cfg["suffix"]}',
        '    restart: unless-stopped',
    ]
    if cpus:
        # 한 로봇이 코어를 다 먹는 것을 막는다. 코어를 고정하려면 cpuset 을 쓴다
        # (compose 에 손으로 추가: cpuset: "0,1").
        # Compose V2 는 서비스 최상위 cpus 를 숫자로 받는다.
        lines.append(f'    cpus: {cpus}')
    lines.append('    environment:')
    if cfg['display']:
        lines.append(f'      - DISPLAY={cfg["display"]}')
    lines += [
        '      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        '      - ROS_LOCALHOST_ONLY=0',
        '      - ROS_DOMAIN_ID=30',
        '      - WEBOTS_HOST=host.docker.internal',
        '      - WEBOTS_PORT=1234',
        f'      - ROBOT_ID={robot_id}',
        f'      - ROBOT_INIT_X={x:.3f}',
        f'      - ROBOT_INIT_Y={y:.3f}',
        f'      - ROBOT_INIT_YAW={yaw:.3f}',
        # Webots 노드 쪽 값과 맞춘다. 드론만 동기 — 자세 루프가 매 물리 스텝
        # 돌지 않으면 뒤집힌다 (drone_setup.md 참고).
        f'      - ROBOT_SYNCHRONIZATION={"true" if needs_sync else "false"}',
    ]
    if needs_def:
        # spot_driver 가 Supervisor 로 자기 몸을 DEF 이름으로 찾는다.
        # 소환기가 붙이는 DEF 와 같아야 한다 (robot_types.RobotType.def_name).
        def_name = ''.join(c if c.isalnum() else '_' for c in robot_id).upper()
        lines.append(f'      - ROBOT_DEF={def_name}')
    lines += [
        '    command: >',
        '      bash -c "source /ros2_ws/install/setup.bash &&',
        f'               ros2 launch {pkg} {launch}"',
        '    depends_on:',
        '      - fleet',
    ]
    return '\n'.join(lines)


def render(robots, plat, cpus, manifest_name):
    head = [
        BEGIN,
        f'  # 매니페스트: config/fleet/{manifest_name}  (로봇 {len(robots)}대)',
        '  #',
        '  # 몸은 fleet 컨테이너의 소환기가 Webots 에 주입하고, 뇌(driver/SLAM/Nav2)는',
        '  # 아래 컨테이너들이 담당한다. depends_on: fleet 인 이유는 몸이 먼저 생기는',
        '  # 편이 자연스럽기 때문이고, 순서가 뒤바뀌어도 extern 컨트롤러가 몸이',
        '  # 나타날 때까지 기다리므로 깨지지는 않는다.',
        '  #',
        '  # 코어를 고정하려면 각 서비스에 cpuset 을 직접 추가한다:  cpuset: "0,1"',
    ]
    body = [service_block(r, plat, cpus) for r in robots]
    return '\n'.join(head) + '\n' + '\n\n'.join(body) + '\n' + END


def patch(path, block):
    text = path.read_text(encoding='utf-8')
    pattern = re.compile(
        re.escape(BEGIN) + r'.*?' + re.escape(END), re.DOTALL)
    if pattern.search(text):
        return pattern.sub(lambda _: block, text)
    # 마커가 없으면 services: 아래 첫 줄에 새로 만든다.
    if '\nservices:\n' not in text:
        raise SystemExit(f'{path}: services: 블록을 찾지 못했습니다')
    return text.replace('\nservices:\n', '\nservices:\n' + block + '\n', 1)


def set_manifest_brains(text):
    """생성된 compose 에서는 fleet 이 매니페스트 로봇의 뇌를 띄우지 않는다."""
    if 'manifest_brains:=' in text:
        return re.sub(r'manifest_brains:=\w+', 'manifest_brains:=false', text)
    return re.sub(
        r'(ros2 launch webots_robot_spawner spawner\.launch\.py[^"\n]*)',
        r'\1 manifest_brains:=false', text, count=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fleet', default='default.yaml')
    ap.add_argument('--platform', default='windows,ubuntu,mac')
    ap.add_argument('--cpus', default='')
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()

    manifest = FLEET_DIR / args.fleet
    if not manifest.is_file():
        raise SystemExit(f'매니페스트를 찾을 수 없습니다: {manifest}')

    robots, skipped = expand(manifest)
    print(f'매니페스트 {args.fleet}: 컨테이너로 뽑을 로봇 {len(robots)}대')
    for r in robots:
        print(f'  + {r[0]:8s} {r[1]:6s} ({r[2]:.2f}, {r[3]:.2f})')
    for s in skipped:
        print(f'  - 건너뜀: {s}  → fleet 컨테이너가 뇌까지 담당')

    stale = []
    for plat in [p.strip() for p in args.platform.split(',') if p.strip()]:
        if plat not in PLATFORMS:
            raise SystemExit(f'모르는 플랫폼: {plat}')
        path = REPO / 'docker-configs' / plat / 'docker-compose.yml'
        if not path.is_file():
            raise SystemExit(f'compose 파일이 없습니다: {path}')
        new = set_manifest_brains(patch(path, render(robots, plat, args.cpus, args.fleet)))
        if new == path.read_text(encoding='utf-8'):
            print(f'  {plat}: 변경 없음')
            continue
        if args.check:
            stale.append(plat)
            print(f'  {plat}: ❌ 최신이 아닙니다')
            continue
        path.write_text(new, encoding='utf-8')
        print(f'  {plat}: 갱신')

    if stale:
        print(f'\n다시 생성해야 합니다: {", ".join(stale)}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
