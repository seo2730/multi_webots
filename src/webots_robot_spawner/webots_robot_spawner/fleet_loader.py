"""편대 매니페스트(yaml)를 읽어 로봇들을 한 번에 소환한다.

Phase C 의 핵심이다. 여기까지 오면 로봇은 더 이상 `.wbt` 에 박힌 **구조**가 아니고
yaml 에 적힌 **데이터**가 된다. 로봇 4대를 쓸지, 10대를 쓸지, 아예 안 쓸지가
파일 하나 고르는 문제로 바뀐다.

매니페스트 형식:

    fleet:
      # 자리를 지정하는 방식
      - {type: ugv,   id: ugv1, x: -6.159, y: 1.263, yaw: -2.910}

      # 맵/영역에서 무작위로 고르는 방식. count 로 여러 대를 한 번에.
      - {type: ugv,   count: 3, random: true}
      - {type: drone, count: 2, random: true, clearance: 1.2}

    # random 항목을 배치할 영역 (xmin, ymin, xmax, ymax).
    # 주면 무작위 배치는 항상 이 안에서만 이루어진다. 맵(/map_merged)이 있으면 영역
    # 안에서 장애물까지 피하고, 없으면(냉시동) 로봇 간격만 보고 고른다.
    spawn_area: [-9.0, -6.0, 9.0, 7.0]

항목별 키:
    type       필수. ugv | spot | drone
    id         생략하면 자동 채번 (ugv1, ugv2 ...)
    count      생략하면 1. 2 이상이면 id 는 자동 채번만 쓴다.
    random     true 면 x/y/yaw 무시하고 빈 자리를 찾는다
    x, y, yaw  random 이 아닐 때의 위치 (yaw 생략 시 0)
    clearance  주변에 요구할 여유 반경(m). 생략하면 로봇 종류별 기본값
    force      검사에 실패해도 그 자리에 놓는다 (로봇 겹침 검사까지 무시)
    strict     좌표를 지정한 항목에서 맵 점유 검사를 거절 사유로 볼지. 기본 false.

기본이 `strict: false` 인 이유:
    매니페스트는 부팅 설정이고, 사람이 좌표를 골라 적어 둔 것이다. 실패해도
    되물을 상대가 없다. 그런데 점유격자는 SLAM 파생물이라 낡을 수 있다 —
    월드를 비우고 편대를 올릴 때 이전 세션 맵에 남은 "옛 로봇의 몸" 때문에
    원래 스폰 좌표 4곳이 전부 거절되는 일을 실제로 겪었다.
    맵이 못 미더워도 사람이 적은 좌표는 존중하고 경고만 남긴다.
    **로봇끼리 겹치는 것은 strict 와 무관하게 항상 막는다.**
"""

import pathlib

import yaml


def _as_list(manifest, path):
    """매니페스트에서 로봇 목록을 꺼낸다. 최상위가 리스트인 형태도 받아 준다."""
    if isinstance(manifest, list):
        return manifest
    if isinstance(manifest, dict):
        entries = manifest.get('fleet')
        if entries is None:
            raise ValueError(f"{path} 에 'fleet' 키가 없습니다")
        if not isinstance(entries, list):
            raise ValueError(f"{path} 의 'fleet' 은 목록이어야 합니다")
        return entries
    raise ValueError(f'{path} 형식을 알 수 없습니다 (목록이나 매핑이어야 합니다)')


def load_fleet(node, path: str):
    """매니페스트대로 소환한다. 실패한 항목은 건너뛰고 계속 진행한다.

    한 대가 실패했다고 나머지를 포기하면, 예를 들어 좁은 곳에 6대를 요청했을 때
    아무것도 안 뜨는 결과가 된다. 되는 것부터 띄우고 요약을 남기는 게 낫다.
    """
    manifest_path = pathlib.Path(path).expanduser()
    if not manifest_path.is_file():
        raise FileNotFoundError(f'편대 매니페스트를 찾을 수 없습니다: {manifest_path}')

    with open(manifest_path, encoding='utf-8') as f:
        manifest = yaml.safe_load(f)

    entries = _as_list(manifest, manifest_path)
    area = manifest.get('spawn_area') if isinstance(manifest, dict) else None
    bounds = None
    if area is not None:
        if len(area) != 4:
            raise ValueError(
                f'spawn_area 는 [xmin, ymin, xmax, ymax] 4개여야 합니다: {area}')
        bounds = tuple(float(v) for v in area)

    node.get_logger().info(
        f'편대 소환 시작: {manifest_path.name} — 항목 {len(entries)}개'
        + (f', spawn_area={bounds}' if bounds else ''))

    made, attached, failed = [], [], []

    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            failed.append((f'#{index}', '항목이 매핑이 아닙니다'))
            continue
        if 'type' not in entry:
            failed.append((f'#{index}', "'type' 키가 없습니다"))
            continue

        count = int(entry.get('count', 1))
        for n in range(max(count, 1)):
            # count 가 2 이상이면 id 를 지정할 수 없다. 같은 이름이 겹치기 때문이다.
            robot_id = '' if count > 1 else str(entry.get('id', ''))
            label = robot_id or f"{entry['type']}#{index}.{n}"

            # 이미 몸이 있는 이름이면 몸을 또 만들지 않고 뇌만 붙인다.
            # fleet 컨테이너가 재시작됐을 때 편대를 그대로 복구하기 위한 경로다.
            if robot_id and robot_id in {r for r, _, _ in node._scan_robots()}:
                result = node.attach_brain(entry['type'], robot_id)
                (attached if result.success else failed).append(
                    (label, result.message))
                continue

            result = node.spawn_one(
                type_key=entry['type'],
                robot_id=robot_id,
                random_place=bool(entry.get('random', False)),
                x=float(entry.get('x', 0.0)),
                y=float(entry.get('y', 0.0)),
                yaw=float(entry.get('yaw', 0.0)),
                min_clearance=float(entry.get('clearance', 0.0)),
                force=bool(entry.get('force', False)),
                bounds=bounds,
                strict_map=bool(entry.get('strict', False)),
            )
            if result.success:
                made.append((result.robot_id, result.x, result.y))
            else:
                failed.append((label, result.message))

    node.get_logger().info(
        f'편대 소환 완료: 새로 {len(made)}대, 뇌만 붙임 {len(attached)}대, '
        f'실패 {len(failed)}건')
    for rid, x, y in made:
        node.get_logger().info(f'  + {rid} ({x:.2f}, {y:.2f})')
    for rid, msg in attached:
        node.get_logger().info(f'  = {rid}: {msg}')
    for label, msg in failed:
        node.get_logger().warn(f'  ! {label}: {msg}')

    return made, attached, failed
