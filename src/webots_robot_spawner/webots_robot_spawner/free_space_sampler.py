"""병합 맵(`/map_merged`)에서 로봇을 놓을 만한 빈 자리를 고른다.

`world` 프레임 기준으로 이야기하면 되는 이유는 10_MAP_MERGE.md 2절에 있다.
이 프로젝트에서 `world -> {ns}/map`은 항등변환이라 **병합 맵의 (x, y)가 곧
Webots 월드 좌표**다. 그래서 여기서 고른 좌표를 좌표 변환 없이 그대로
`translation`에 넣을 수 있다.

점유 격자 값 규약(map_merger와 동일): -1 = 미탐색, 0 = 비어있음, 100 = 장애물.
"""

import math

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

UNKNOWN = -1
FREE = 0


def merged_map_qos() -> QoSProfile:
    """map_merger가 `/map_merged`를 내보내는 QoS와 같은 프로파일.

    이게 어긋나면 에러 없이 조용히 아무것도 안 들어온다. 맵이 안 잡힐 때
    제일 먼저 의심할 곳. (map_merger.map_qos()와 같은 값)
    """
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class FreeSpaceSampler:
    """`/map_merged`를 받아 두고, 요청이 오면 빈 자리를 골라 준다.

    맵은 계속 갱신되지만 소환은 가끔 일어나므로, 들어온 맵을 그대로 들고 있다가
    요청 시점에 계산한다. 미리 계산해 두면 맵이 바뀔 때마다 헛일이 된다.
    """

    def __init__(self, node, topic: str = '/map_merged', allow_unknown: bool = False):
        self._node = node
        self._allow_unknown = allow_unknown
        self._grid = None          # 최근 OccupancyGrid
        self._data = None          # (h, w) int8 뷰
        self._free_cells = None    # 비어있는 셀의 (row, col) 인덱스 캐시
        self._stamp_key = None     # 캐시가 어느 맵에 대한 것인지
        self._warned_rotated = False
        self._masks = {}           # 반경(셀) -> 원형 마스크 캐시

        self._sub = node.create_subscription(
            OccupancyGrid, topic, self._on_map, merged_map_qos())

    # ------------------------------------------------------------------ 수신

    def _on_map(self, msg: OccupancyGrid):
        self._grid = msg
        self._data = None
        self._free_cells = None

        q = msg.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if abs(yaw) > 1e-3 and not self._warned_rotated:
            # 이 클래스는 격자가 world 축에 정렬돼 있다고 가정하고 좌표를 환산한다.
            # map_merger는 항상 정렬된 격자를 내보내므로 여기 걸리면 설계가 바뀐 것이다.
            self._warned_rotated = True
            self._node.get_logger().warn(
                f'/map_merged의 origin이 {yaw:.3f} rad 회전돼 있습니다. '
                '이 샘플러는 축 정렬 격자를 가정하므로 좌표가 어긋납니다.')

    @property
    def has_map(self) -> bool:
        return self._grid is not None and self._grid.info.width > 0

    def map_info(self) -> str:
        if not self.has_map:
            return '맵 없음'
        i = self._grid.info
        return (f'{i.width}x{i.height} @ {i.resolution:.3f}m/셀, '
                f'origin=({i.origin.position.x:.2f}, {i.origin.position.y:.2f})')

    # ------------------------------------------------------------------ 내부

    def _ensure_arrays(self):
        """맵이 바뀌었을 때만 numpy 변환과 빈 셀 인덱싱을 다시 한다."""
        if self._data is not None:
            return
        info = self._grid.info
        self._data = np.asarray(self._grid.data, dtype=np.int8).reshape(
            info.height, info.width)
        rows, cols = np.nonzero(self._data == FREE)
        self._free_cells = np.stack((rows, cols), axis=1)

    def _disk_mask(self, radius_cells: int) -> np.ndarray:
        """반경 안쪽 셀만 True인 원형 마스크. 정사각형으로 검사하면 벽 옆
        자리를 필요 이상으로 버리게 되므로 원형을 쓴다."""
        mask = self._masks.get(radius_cells)
        if mask is None:
            span = np.arange(-radius_cells, radius_cells + 1)
            dy, dx = np.meshgrid(span, span, indexing='ij')
            mask = (dx * dx + dy * dy) <= radius_cells * radius_cells
            self._masks[radius_cells] = mask
        return mask

    def world_to_cell(self, x: float, y: float):
        info = self._grid.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return row, col

    def cell_to_world(self, row: int, col: int):
        info = self._grid.info
        # 셀 중심을 쓴다. 모서리를 쓰면 해상도의 절반만큼 치우친다.
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    # ------------------------------------------------------------------ 판정

    def check_robots(self, x: float, y: float, clearance: float, avoid=()):
        """다른 로봇과 충분히 떨어졌는지만 본다.

        맵 검사와 분리한 이유: 이 둘은 신뢰도가 다르다. 로봇 위치는 씬 트리에서
        직접 읽은 **사실**이고, 점유격자는 SLAM이 만든 **파생물**이라 낡거나 틀릴 수
        있다. 그래서 호출하는 쪽이 "로봇 겹침은 절대 안 되지만 맵은 참고만" 같은
        판단을 할 수 있어야 한다.
        """
        for (ax, ay, ar) in avoid:
            need = clearance + ar
            dist = math.hypot(x - ax, y - ay)
            if dist < need:
                return False, (f'기존 로봇과 너무 가깝습니다 '
                               f'(({ax:.2f}, {ay:.2f})에서 {dist:.2f}m, '
                               f'{need:.2f}m 필요)')
        return True, 'ok'

    def check(self, x: float, y: float, clearance: float, avoid=()):
        """로봇 간격과 맵 점유를 모두 본다.

        Returns:
            (ok: bool, reason: str) — 실패 사유를 그대로 서비스 응답에 실어 보낸다.
        """
        ok, reason = self.check_robots(x, y, clearance, avoid)
        if not ok:
            return False, reason
        return self.check_map(x, y, clearance)

    def check_map(self, x: float, y: float, clearance: float):
        """점유격자만 본다. 맵이 없으면 실패로 다룬다(판단 근거가 없으므로)."""
        if not self.has_map:
            return False, '맵(/map_merged)이 아직 없어 빈 공간을 검사할 수 없습니다'

        self._ensure_arrays()
        info = self._grid.info
        row, col = self.world_to_cell(x, y)
        rc = max(1, int(math.ceil(clearance / info.resolution)))

        if (row - rc < 0 or col - rc < 0
                or row + rc >= info.height or col + rc >= info.width):
            return False, f'맵 범위를 벗어납니다 ({self.map_info()})'

        window = self._data[row - rc:row + rc + 1, col - rc:col + rc + 1]
        mask = self._disk_mask(rc)
        cells = window[mask]

        if np.any(cells > FREE):
            return False, f'반경 {clearance:.2f}m 안에 장애물이 있습니다'
        if not self._allow_unknown and np.any(cells == UNKNOWN):
            return False, (f'반경 {clearance:.2f}m 안에 미탐색 영역이 있습니다 '
                           '(allow_unknown=true로 허용할 수 있습니다)')
        return True, 'ok'

    def sample_in_bounds(self, bounds, clearance: float, avoid=(),
                         attempts: int = 200, rng=None):
        """주어진 사각형 안에서 자리를 고른다.

        맵이 있으면 영역 안에서 뽑되 맵 검사(check)까지 통과시킨다.
        맵이 없으면 벽을 피할 근거가 없으므로 **로봇끼리의 간격만 보장한다.**

        후자가 필요한 이유: 월드에 로봇이 하나도 없는 냉시동에서는 SLAM 맵이 존재할
        수 없는데, 편대 매니페스트는 그 상태에서 `random: true`로 여러 대를 요청할 수
        있다. 그래서 spawn_area 는 사용자가 "이 안은 대체로 비어 있다"고 아는 영역이어야
        한다.
        """
        xmin, ymin, xmax, ymax = bounds
        if not (xmax > xmin and ymax > ymin):
            return None, f'spawn_area 가 뒤집혀 있습니다: {bounds}'

        rng = rng or np.random.default_rng()
        last_reason = '알 수 없음'
        for _ in range(attempts):
            x = float(rng.uniform(xmin, xmax))
            y = float(rng.uniform(ymin, ymax))

            if self.has_map:
                ok, reason = self.check(x, y, clearance, avoid)
                if ok:
                    return (x, y), 'ok'
                last_reason = reason
                continue

            # 맵이 없을 때는 로봇 간 간격만 검사한다.
            too_close = None
            for (ax, ay, ar) in avoid:
                need = clearance + ar
                dist = math.hypot(x - ax, y - ay)
                if dist < need:
                    too_close = (f'기존 로봇과 너무 가깝습니다 '
                                 f'(({ax:.2f}, {ay:.2f})에서 {dist:.2f}m, '
                                 f'{need:.2f}m 필요)')
                    break
            if too_close is None:
                return (x, y), 'ok (맵 없음 — 로봇 간격만 검사)'
            last_reason = too_close

        return None, (f'spawn_area {bounds} 안에서 {attempts}번 시도했지만 '
                      f'자리를 못 찾았습니다 (마지막 사유: {last_reason})')

    def sample(self, clearance: float, avoid=(), attempts: int = 200, rng=None,
               bounds=None):
        """빈 자리를 무작위로 하나 고른다. 못 고르면 (None, 사유).

        전체 격자에 거리변환을 돌리는 대신, 비어있는 셀 중에서 무작위로 뽑아
        그 자리만 검사하는 기각 샘플링을 쓴다. 맵이 커져도 비용이 맵 크기가 아니라
        시도 횟수에 비례하고, 판정 로직을 check()와 100% 공유할 수 있다.

        Args:
            bounds: 주어지면 **항상** 이 사각형 안에서만 고른다. 맵이 있으면 그 안에서
                맵 검사(장애물·미탐색)까지 통과시키고, 없으면 로봇 간격만 본다.

                맵이 있을 때 bounds 를 무시하는 쪽으로도 만들 수 있었지만 그렇게 하지
                않았다. spawn_area 를 쓰는 사람의 의도는 "여기 안에 두라"이고, 맵이
                생겼다고 영역 밖에 배치되면 그 의도를 배신한다. 맵은 영역을 넓히는
                근거가 아니라 영역 안을 더 정확히 보는 수단이다.
        """
        if bounds is not None:
            return self.sample_in_bounds(
                bounds, clearance, avoid=avoid, attempts=attempts, rng=rng)

        if not self.has_map:
            return None, ('맵(/map_merged)이 아직 없습니다. 월드가 비어 있는 '
                          '냉시동이면 매니페스트에 spawn_area 를 지정하세요.')

        self._ensure_arrays()
        if len(self._free_cells) == 0:
            return None, f'맵에 비어있는 셀이 하나도 없습니다 ({self.map_info()})'

        rng = rng or np.random.default_rng()
        picks = rng.integers(0, len(self._free_cells), size=attempts)
        last_reason = '알 수 없음'
        for idx in picks:
            row, col = self._free_cells[idx]
            x, y = self.cell_to_world(int(row), int(col))
            ok, reason = self.check(x, y, clearance, avoid)
            if ok:
                return (x, y), 'ok'
            last_reason = reason

        return None, (f'{attempts}번 시도했지만 조건을 만족하는 자리를 못 찾았습니다 '
                      f'(마지막 사유: {last_reason})')
