#!/usr/bin/env python3
"""임의의 Webots 월드를 로봇 소환이 가능한 상태로 만든다.

소환기가 붙으려면 월드에 두 가지가 있어야 한다.

  1. `spawn_supervisor` Robot 노드
     실행 중에 로봇을 추가하는 방법은 Supervisor API 뿐인데, 그 권한을 가진 노드가
     월드에 하나 있어야 한다. 몸도 센서도 없는 유령 로봇이다.

  2. 소환할 PROTO 들의 `IMPORTABLE EXTERNPROTO` 선언
     일반 EXTERNPROTO 로는 런타임 주입이 실패한다. Webots 가 이렇게 거절한다:
       ERROR: In order to import the PROTO 'X', first it must be declared in the
              IMPORTABLE EXTERNPROTO list.

이 스크립트가 그 둘을 넣어 준다. 그래서 **밖에서 가져온 월드도 그대로 쓸 수 있다** —
Webots 샘플, webots.cloud 공유 월드, 직접 만든 월드 무엇이든.

실행 (호스트에 파이썬이 없어도 되게 도커로 돌린다 — 크로스 플랫폼 전제):

    ⚠️ 현재 경로를 넘기는 문법이 셸마다 다르다. 아래는 전부 같은 뜻이다.
         cmd.exe      -v "%cd%:/w"      (줄 연결은 ^)
         PowerShell   -v "${PWD}:/w"    (줄 연결은 백틱)
         Git Bash 등  -v "$PWD:/w"      (줄 연결은 역슬래시)
       헷갈리면 절대경로를 그냥 적는 편이 확실하다 — 어느 셸에서든 동작한다:
         -v "D:/path/to/webots_multi_robot:/w"

    docker run --rm -v "$PWD:/w" -w /w windows-master python3 \\
        src/webots_robot_spawner/scripts/prepare_world.py \\
        --source /usr/local/webots/projects/samples/environments/factory/worlds/factory.wbt \\
        --name factory

    옵션:
      --source PATH   원본 .wbt (컨테이너 안 경로 또는 저장소 상대 경로)
      --name NAME     결과 파일 이름 (worlds/NAME.wbt 로 저장)
      --in-place      원본을 그대로 고친다 (--name 대신)
      --check         고치지 않고 준비됐는지만 확인 (종료코드 1 = 준비 안 됨)

주의:
  - 결과 월드는 반드시 우리 worlds/ 디렉터리에 둔다. 래퍼 PROTO 를 `../protos/` 로
    참조하기 때문이다.
  - 원본에 `controller "<extern>"` 노드가 있으면 경고한다. 그 컨트롤러가 안 붙으면
    Webots 가 시뮬레이션을 멈춘다.
"""

import argparse
import pathlib
import re
import shutil
import sys

_HERE = pathlib.Path(__file__).resolve()
REPO = _HERE.parents[3]
WORLDS = REPO / 'src' / 'Webots-SummitXL' / 'workspace' / 'simulator' / 'worlds'

# 소환 대상 PROTO 들. robot_types.py 의 proto 값과 일치해야 한다.
# 경로는 worlds/ 기준 상대 경로다.
IMPORTABLE = [
    '../protos/Mavic2ProMediumSensorized.proto',
    '../protos/SummitXlSteelSensorized.proto',
    '../protos/SpotSensorized.proto',
]

MARK = '# >>> SPAWNER READY — prepare_world.py 가 넣은 것'
MARK_END = '# <<< SPAWNER READY'

SUPERVISOR = f'''{MARK}
# 로봇 소환 전담 노드. 몸도 센서도 없이 씬 트리를 조작할 권한만 갖는다.
# 실행 중에 로봇을 추가하는 방법이 Supervisor API 뿐이라 이런 노드가 하나 필요하다.
#
# synchronization FALSE 가 중요하다. TRUE 면 Webots 가 매 스텝 이 컨트롤러의 응답을
# 기다리므로, fleet 컨테이너가 안 떠 있으면 시뮬레이션 전체가 멈춘다.
DEF SPAWN_SUPERVISOR Robot {{
  name "spawn_supervisor"
  controller "<extern>"
  supervisor TRUE
  synchronization FALSE
}}
{MARK_END}
'''


def is_prepared(text):
    has_node = 'name "spawn_supervisor"' in text
    has_protos = all(f'IMPORTABLE EXTERNPROTO "{p}"' in text for p in IMPORTABLE)
    return has_node and has_protos


def prepare(text, world_name):
    """월드 텍스트에 IMPORTABLE 선언과 supervisor 노드를 넣는다."""
    notes = []

    # --- 1. IMPORTABLE EXTERNPROTO ---
    # 이미 일반 EXTERNPROTO 로 선언돼 있으면 IMPORTABLE 로 승격한다. 두 줄로 두면
    # Webots 가 중복 선언으로 본다.
    for proto in IMPORTABLE:
        importable_line = f'IMPORTABLE EXTERNPROTO "{proto}"'
        if importable_line in text:
            continue
        plain = f'EXTERNPROTO "{proto}"'
        if plain in text:
            text = text.replace(plain, importable_line, 1)
            notes.append(f'{proto}: EXTERNPROTO -> IMPORTABLE 로 승격')
            continue
        # 새로 넣는다. 헤더(#VRML_SIM ...) 바로 다음 줄에 붙인다.
        lines = text.split('\n')
        insert_at = 1
        for i, line in enumerate(lines):
            if line.startswith('EXTERNPROTO ') or line.startswith('IMPORTABLE EXTERNPROTO '):
                insert_at = i
                break
        lines.insert(insert_at, importable_line)
        text = '\n'.join(lines)
        notes.append(f'{proto}: IMPORTABLE 선언 추가')

    # --- 2. spawn_supervisor 노드 ---
    if 'name "spawn_supervisor"' in text:
        notes.append('spawn_supervisor: 이미 있음')
    else:
        if not text.endswith('\n'):
            text += '\n'
        text += SUPERVISOR
        notes.append('spawn_supervisor: 노드 추가')

    # --- 3. 경고 ---
    externs = re.findall(r'controller\s+"<extern>"', text)
    # supervisor 자신은 빼고 센다.
    if len(externs) > 1:
        notes.append(
            f'⚠️ <extern> 컨트롤러가 {len(externs) - 1}개 더 있습니다. 그 컨트롤러가 '
            '안 붙으면 Webots 가 시뮬레이션을 멈춥니다')

    return text, notes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', required=True)
    ap.add_argument('--name')
    ap.add_argument('--in-place', action='store_true')
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()

    src = pathlib.Path(args.source)
    if not src.is_absolute():
        src = REPO / src
    if not src.is_file():
        raise SystemExit(f'원본 월드를 찾을 수 없습니다: {src}')

    # --check 는 원본(또는 --name 이 가리키는 결과물)이 준비됐는지만 본다.
    # 고치지 않으므로 --name 을 요구하지 않는다.
    if args.check:
        target = WORLDS / f'{args.name}.wbt' if args.name else src
        if not target.is_file():
            print(f'{target}: ❌ 파일이 없습니다')
            return 1
        ok = is_prepared(target.read_text(encoding='utf-8'))
        print(f'{target}: {"준비됨" if ok else "❌ 준비 안 됨"}')
        return 0 if ok else 1

    if args.in_place:
        dst = src
    else:
        if not args.name:
            raise SystemExit('--name 이나 --in-place 중 하나가 필요합니다')
        dst = WORLDS / f'{args.name}.wbt'

    text = src.read_text(encoding='utf-8')

    if dst != src:
        WORLDS.mkdir(parents=True, exist_ok=True)
        if dst.exists():
            print(f'  기존 파일을 덮어씁니다: {dst.name}')
        shutil.copyfile(src, dst)
        text = dst.read_text(encoding='utf-8')
        # 원본 옆에 있던 프로젝트 설정 파일은 따라오지 않는다. 없어도 Webots 가
        # 기본값으로 연다.
        print(f'  {src}  ->  {dst}')

    new_text, notes = prepare(text, dst.stem)
    dst.write_text(new_text, encoding='utf-8')

    for n in notes:
        print(f'  {n}')
    print(f'\n준비 완료: {dst}')
    print('Webots 에서 이 월드를 열고, 편대 매니페스트를 이 월드에 맞게 만드세요.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
