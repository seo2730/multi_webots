"""마스터 관제 컨테이너에서 로봇별 SLAM 맵을 하나의 전역 맵으로 병합한다.

로봇마다 slam_toolbox가 만드는 맵은 서로 다른 좌표계(`{ns}/map`)를 쓴다.
이 노드는 공통 `world` 프레임을 기준으로 각 로봇 맵을 못 박아(static TF) 두고,
모든 맵을 `world` 격자에 리샘플링해서 `/map_merged` 하나로 합쳐 발행한다.

로봇 목록은 세 경로로 채워지며, 뒤로 갈수록 우선순위가 낮다.
  1) 등록 토픽(`/robot_registry`) — 로봇 컨테이너가 스스로 보내는 명함 + 하트비트
  2) 설정 파일(`config/robots.yaml`) — 미리 알고 있는 초기 위치
  3) 토픽 그래프 자동 탐색 — `/{ns}/map`이 보이면 일단 잡되 초기 위치는 원점 가정

시각은 전부 노드 클럭(use_sim_time=True → Webots 시뮬 시간)을 쓴다.
시뮬을 일시정지하면 병합도 같이 멈추고, 멈춘 동안 로봇이 죽은 것으로
오판하지도 않는다.
"""

import json
import math
import re
from dataclasses import dataclass, field
from functools import partial

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

# 점유 격자 값 규약: -1 = 미탐색, 0 = 비어있음, 100 = 장애물.
# 이 순서가 그대로 크기 순서라서 병합 규칙이 np.maximum 하나로 끝난다.
UNKNOWN = -1


def map_qos() -> QoSProfile:
    """slam_toolbox가 `map`을 내보내는 QoS와 정확히 같은 프로파일.

    이게 안 맞으면 에러도 경고도 없이 그냥 아무 메시지도 안 들어온다.
    병합이 조용히 실패할 때 첫 번째로 의심할 곳.
    """
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def registry_qos() -> QoSProfile:
    """등록 토픽용. 마스터가 늦게 떠도 이미 보낸 명함을 받도록 TRANSIENT_LOCAL."""
    return QoSProfile(
        depth=20,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass
class Robot:
    """병합 대상 로봇 하나의 상태."""

    ns: str
    init_x: float = 0.0
    init_y: float = 0.0
    init_yaw: float = 0.0
    has_map: bool = True
    map_topic: str = ''
    source: str = 'discovery'   # registry | config | discovery
    last_seen_ns: int = 0
    grid: OccupancyGrid = None
    sub: object = field(default=None, repr=False)

    @property
    def map_frame(self) -> str:
        return f'{self.ns}/map'


class MapMerger(Node):

    def __init__(self):
        # robots.yaml의 `robots.<ns>.<field>` 형태를 그대로 파라미터로 받기 위해
        # 자동 선언을 켠다. 그래서 아래 파라미터들은 _param()으로 조건부 선언한다.
        super().__init__('map_merger', automatically_declare_parameters_from_overrides=True)

        self.world_frame = self._param('world_frame', 'world')
        self.merged_topic = self._param('merged_map_topic', '/map_merged')
        self.resolution = float(self._param('resolution', 0.1))
        self.merge_rate = float(self._param('merge_rate', 1.0))
        self.discovery_rate = float(self._param('discovery_rate', 0.5))
        self.timeout_sec = float(self._param('robot_timeout', 15.0))
        self.auto_discovery = bool(self._param('auto_discovery', True))
        self.publish_static_tf = bool(self._param('publish_static_tf', True))
        # 이 프로젝트의 Webots 드라이버는 odom->base_link 를 GPS 원값,
        # 즉 월드 절대좌표로 그대로 발행한다 (robot_driver.py 참고).
        # 그래서 각 로봇의 map 프레임이 이미 world 와 정렬돼 있고,
        # 여기에 스폰 좌표를 또 더하면 정확히 두 배로 어긋난다.
        self.odom_is_world_absolute = bool(self._param('odom_is_world_absolute', True))
        self.padding = float(self._param('padding', 1.0))
        self.max_cells = int(self._param('max_merged_cells', 8_000_000))
        self.map_topic_regex = re.compile(self._param('map_topic_pattern', r'^/([^/]+)/map$'))
        registry_topic = self._param('registry_topic', '/robot_registry')

        self.robots: dict[str, Robot] = {}
        self._tf_signature: tuple = ()

        self._load_robots_from_config()

        self.merged_pub = self.create_publisher(OccupancyGrid, self.merged_topic, map_qos())
        self.tf_static = StaticTransformBroadcaster(self)
        self.create_subscription(String, registry_topic, self._on_registry, registry_qos())

        self.create_timer(1.0 / max(self.discovery_rate, 0.01), self._discover)
        self.create_timer(1.0 / max(self.merge_rate, 0.01), self._merge_and_publish)

        anchor_mode = ('오도메트리가 월드 절대좌표 -> world->{ns}/map 항등변환'
                       if self.odom_is_world_absolute else '스폰 초기위치를 앵커로 사용')
        self.get_logger().info(
            f"맵 병합 시작 | world='{self.world_frame}' -> '{self.merged_topic}' "
            f"| 해상도 {self.resolution}m | 병합 {self.merge_rate}Hz "
            f"| 자동탐색 {'ON' if self.auto_discovery else 'OFF'}")
        self.get_logger().info(f'정렬 기준: {anchor_mode}')

    # ------------------------------------------------------------------
    # 파라미터 / 설정
    # ------------------------------------------------------------------
    def _param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _load_robots_from_config(self):
        """robots.yaml의 `robots:` 블록을 읽어 초기 위치를 미리 채운다."""
        entries: dict[str, dict] = {}
        for key, param in self.get_parameters_by_prefix('robots').items():
            if '.' not in key:
                continue
            ns, field_name = key.split('.', 1)
            entries.setdefault(ns, {})[field_name] = param.value

        for ns, values in entries.items():
            self.robots[ns] = Robot(
                ns=ns,
                init_x=float(values.get('init_x', 0.0)),
                init_y=float(values.get('init_y', 0.0)),
                init_yaw=float(values.get('init_yaw', 0.0)),
                has_map=bool(values.get('has_map', True)),
                map_topic=str(values.get('map_topic', f'/{ns}/map')),
                source='config',
            )
            self.get_logger().info(
                f"[설정] {ns} 초기위치=({values.get('init_x', 0.0)}, "
                f"{values.get('init_y', 0.0)}, yaw {values.get('init_yaw', 0.0)}) "
                f"맵={'있음' if self.robots[ns].has_map else '없음'}")

    # ------------------------------------------------------------------
    # 1단: 등록 토픽 (초기 위치 + 하트비트)
    # ------------------------------------------------------------------
    def _on_registry(self, msg: String):
        try:
            info = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'등록 메시지 파싱 실패: {msg.data[:120]}')
            return

        ns = info.get('robot_id')
        if not ns:
            return

        robot = self.robots.get(ns)
        if robot is None:
            robot = Robot(ns=ns)
            self.robots[ns] = robot
            self.get_logger().info(f'[등록] 새 로봇 합류: {ns}')
        elif robot.source != 'registry':
            self.get_logger().info(f'[등록] {ns} 정보를 등록 토픽 기준으로 갱신 (이전: {robot.source})')

        # 등록 토픽이 설정 파일보다 우선한다. 로봇 자신이 아는 값이 더 정확하다.
        robot.init_x = float(info.get('init_x', robot.init_x))
        robot.init_y = float(info.get('init_y', robot.init_y))
        robot.init_yaw = float(info.get('init_yaw', robot.init_yaw))
        robot.has_map = bool(info.get('has_map', robot.has_map))
        robot.map_topic = str(info.get('map_topic') or f'/{ns}/map')
        robot.source = 'registry'
        robot.last_seen_ns = self.get_clock().now().nanoseconds

    # ------------------------------------------------------------------
    # 2단: 토픽 그래프 자동 탐색 + 죽은 로봇 정리
    # ------------------------------------------------------------------
    def _discover(self):
        if self.auto_discovery:
            for topic, types in self.get_topic_names_and_types():
                if 'nav_msgs/msg/OccupancyGrid' not in types:
                    continue
                match = self.map_topic_regex.fullmatch(topic)
                if not match:
                    continue
                ns = match.group(1)
                if ns in self.robots:
                    continue
                self.robots[ns] = Robot(ns=ns, map_topic=topic, source='discovery')
                self.get_logger().warn(
                    f"[탐색] '{topic}' 발견했지만 초기 위치를 모름 -> 원점(0,0,0) 가정. "
                    f"robots.yaml에 추가하거나 robot_registrar를 띄우면 정확히 맞춰짐.")

        # 맵 토픽 구독은 필요할 때 붙인다 (로봇이 늦게 떠도 이 시점에 연결됨).
        for robot in self.robots.values():
            if robot.has_map and robot.sub is None:
                topic = robot.map_topic or f'/{robot.ns}/map'
                robot.sub = self.create_subscription(
                    OccupancyGrid, topic, partial(self._on_map, robot.ns), map_qos())
                self.get_logger().info(f"[구독] {robot.ns} <- '{topic}'")

        self._drop_stale()
        self._refresh_static_tf()

    def _on_map(self, ns: str, msg: OccupancyGrid):
        robot = self.robots.get(ns)
        if robot is None:
            return
        robot.grid = msg
        robot.last_seen_ns = self.get_clock().now().nanoseconds

    def _drop_stale(self):
        """일정 시간 소식 없는 로봇을 병합 대상에서 제외한다.

        컨테이너를 내렸는데 유령 맵이 계속 남아 있는 걸 막는다.
        설정 파일에 적힌 로봇은 아직 한 번도 안 뜬 것일 수 있으므로,
        한 번이라도 소식이 있었던(last_seen_ns > 0) 로봇만 대상으로 한다.
        """
        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(self.timeout_sec * 1e9)

        for ns, robot in list(self.robots.items()):
            if robot.last_seen_ns == 0:
                continue
            if now_ns - robot.last_seen_ns <= timeout_ns:
                continue

            self.get_logger().warn(
                f'[이탈] {ns} 가 {self.timeout_sec:.0f}초 동안 무응답 -> 병합에서 제외')
            if robot.sub is not None:
                self.destroy_subscription(robot.sub)
            if robot.source == 'discovery':
                del self.robots[ns]          # 탐색으로 잡은 건 흔적 없이 지운다
            else:
                robot.sub = None             # 설정/등록된 로봇은 자리를 남겨 재합류를 기다린다
                robot.grid = None
                robot.last_seen_ns = 0

    # ------------------------------------------------------------------
    # world -> {ns}/map static TF
    # ------------------------------------------------------------------
    def _active(self) -> list[Robot]:
        return [r for r in self.robots.values() if r.last_seen_ns > 0]

    def _anchor_of(self, robot: Robot) -> tuple[float, float, float]:
        """`world -> {ns}/map` 변환값.

        오도메트리가 이미 월드 절대좌표면(이 프로젝트의 기본) 항등변환이다.
        각 로봇의 map 프레임이 이미 같은 원점을 공유하므로 더 보탤 것이 없다.

        실제 로봇처럼 오도메트리가 로봇 자기 출발점 기준(0,0)일 때만
        스폰 좌표를 앵커로 써야 한다.
        """
        if self.odom_is_world_absolute:
            return 0.0, 0.0, 0.0
        return robot.init_x, robot.init_y, robot.init_yaw

    def _refresh_static_tf(self):
        """활성 로봇 목록이 바뀔 때만 static TF 전체를 다시 쏜다."""
        if not self.publish_static_tf:
            return

        active = [r for r in self._active() if r.has_map]
        anchors = {r.ns: self._anchor_of(r) for r in active}
        signature = tuple(sorted(
            (r.ns, ) + tuple(round(v, 4) for v in anchors[r.ns]) for r in active))
        if signature == self._tf_signature:
            return
        self._tf_signature = signature

        transforms = []
        now = self.get_clock().now().to_msg()
        for robot in active:
            ax, ay, ayaw = anchors[robot.ns]
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.world_frame
            tf.child_frame_id = robot.map_frame
            tf.transform.translation.x = ax
            tf.transform.translation.y = ay
            tf.transform.rotation.z = math.sin(ayaw / 2.0)
            tf.transform.rotation.w = math.cos(ayaw / 2.0)
            transforms.append(tf)

        if transforms:
            self.tf_static.sendTransform(transforms)
            self.get_logger().info(
                f"[TF] '{self.world_frame}' -> " + ', '.join(t.child_frame_id for t in transforms))

    # ------------------------------------------------------------------
    # 병합 본체
    # ------------------------------------------------------------------
    def _grid_pose_in_world(self, robot: Robot) -> tuple[float, float, float]:
        """격자 원점(좌하단 셀 모서리)의 world 기준 위치와 회전을 구한다.

        world <- {ns}/map <- 격자원점 두 변환의 합성:
            theta = anchor_yaw + origin_yaw
            t     = R(anchor_yaw) * origin_xy + anchor_xy
        """
        ax, ay, ayaw = self._anchor_of(robot)
        origin = robot.grid.info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        c, s = math.cos(ayaw), math.sin(ayaw)
        tx = c * origin.position.x - s * origin.position.y + ax
        ty = s * origin.position.x + c * origin.position.y + ay
        return tx, ty, ayaw + origin_yaw

    def _merge_and_publish(self):
        usable = [r for r in self._active() if r.has_map and r.grid is not None]
        if not usable:
            return

        # --- 1. 모든 맵을 감싸는 world 기준 축정렬 경계상자 구하기 ---
        min_x = min_y = float('inf')
        max_x = max_y = float('-inf')
        placed = []
        for robot in usable:
            info = robot.grid.info
            w_m, h_m = info.width * info.resolution, info.height * info.resolution
            tx, ty, theta = self._grid_pose_in_world(robot)
            c, s = math.cos(theta), math.sin(theta)
            for gx, gy in ((0.0, 0.0), (w_m, 0.0), (0.0, h_m), (w_m, h_m)):
                wx, wy = c * gx - s * gy + tx, s * gx + c * gy + ty
                min_x, max_x = min(min_x, wx), max(max_x, wx)
                min_y, max_y = min(min_y, wy), max(max_y, wy)
            placed.append((robot, tx, ty, theta))

        min_x -= self.padding
        min_y -= self.padding
        max_x += self.padding
        max_y += self.padding

        res = self.resolution
        width = int(math.ceil((max_x - min_x) / res))
        height = int(math.ceil((max_y - min_y) / res))
        if width <= 0 or height <= 0:
            return
        if width * height > self.max_cells:
            self.get_logger().error(
                f'병합 격자가 너무 큼 ({width}x{height}). 로봇 초기 위치가 잘못됐을 가능성이 큼. '
                f'이번 주기는 건너뜀.')
            return

        # --- 2. 병합 격자의 각 셀 중심 world 좌표 (한 번만 만들고 재사용) ---
        xs = min_x + (np.arange(width, dtype=np.float64) + 0.5) * res
        ys = min_y + (np.arange(height, dtype=np.float64) + 0.5) * res
        world_x, world_y = np.meshgrid(xs, ys)

        merged = np.full((height, width), UNKNOWN, dtype=np.int8)

        # --- 3. 역방향 매핑으로 각 로봇 맵을 샘플링해 겹치기 ---
        # 정방향(원본 셀 -> 병합 셀)으로 하면 해상도/회전 차이 때문에 구멍이 생긴다.
        for robot, tx, ty, theta in placed:
            info = robot.grid.info
            src = np.asarray(robot.grid.data, dtype=np.int8).reshape(info.height, info.width)

            c, s = math.cos(-theta), math.sin(-theta)
            dx, dy = world_x - tx, world_y - ty
            gx = c * dx - s * dy
            gy = s * dx + c * dy

            col = np.floor(gx / info.resolution).astype(np.int32)
            row = np.floor(gy / info.resolution).astype(np.int32)
            inside = ((col >= 0) & (col < info.width) & (row >= 0) & (row < info.height))

            sampled = np.full((height, width), UNKNOWN, dtype=np.int8)
            sampled[inside] = src[row[inside], col[inside]]

            # 미탐색(-1) < 비어있음(0) < 장애물(100) 이므로 최댓값이 곧 병합 규칙이 된다.
            np.maximum(merged, sampled, out=merged)

        # --- 4. 발행 ---
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.world_frame
        out.info.resolution = res
        out.info.width = width
        out.info.height = height
        out.info.origin.position.x = min_x
        out.info.origin.position.y = min_y
        out.info.origin.orientation.w = 1.0
        out.data = merged.reshape(-1).tolist()
        self.merged_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
