"""전역 지도에서 프론티어를 뽑아 **어느 로봇이 어디로 갈지** 배정하는 master 노드.

왜 master 인가
--------------
다중 로봇 할당은 원리적으로 중앙집중이다. 다른 로봇이 어디로 가는지 모르면 겹침을
막을 수 없다. 그래서 로봇 컨테이너가 아니라 전역 지도(`/map_merged`)를 이미 만들고 있는
master 에 둔다 — `map_merger`, `sim_clock_bridge` 와 같은 자리다.

    /map_merged ─┐
    /{ns}/odom   ├─▶ 프론티어 검출 ─▶ **할당** ─▶ /{ns}/goal_pose
    /{ns}/map   ─┘                      ▲
                                        └── 여기가 LLM 이 대체할 자리다

지금은 거리 기준 최적 할당(베이스라인)이 들어 있다. `strategy` 파라미터로 갈아 끼운다.
LLM 할당이 이 베이스라인을 못 이기면 증류할 것이 없다는 뜻이므로, 비교 기준을 먼저 둔다.

편대를 어떻게 아는가
--------------------
`/robot_registry` 를 구독한다. 로봇마다 `robot_registrar` 가 1 Hz 로 자기 명함을 보내므로
**재시작 없이** 런타임 소환/제거를 따라간다. 종류(ugv/spot/drone)는 이름 접두사로 안다 —
소환기가 `robot_types.RobotType.id_prefix` 로 그렇게 채번한다.

실측으로 얻은 규칙 네 가지
--------------------------
1. **너무 가까운 후보는 뺀다.** `dist/sqrt(cells)` 정렬은 가까운 후보가 압도적이라,
   안 빼면 로봇이 0.5 m 앞으로만 반복해 보내지고 탐사가 2라운드 만에 멈춘다.
2. **목표를 유지한다.** 매 주기 새로 배정하면 목표가 흔들려(2→6→2→4→8…) 어디에도
   도달하지 못한다. 드론이 6주기 연속 이동 0.0 m 였다.
3. **포기 조건이 있어야 한다.** 유지만 하면 도달 못 하는 목표에 묶인다. 실제로 ugv1 이
   같은 목표를 반복 재시도하며 제자리에 머물렀다.
4. **창 밖 목표는 가장자리로 자른다.** 드론의 Nav2 static layer 는 30x30 m 롤링 창이라
   그 밖은 "off the global costmap" 으로 계획이 무조건 실패한다.
"""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from webots_goal_bridge.frontier_allocator import (assign_by_distance,
                                                   build_observation)
from webots_goal_bridge.llm_goal_assigner import extract_frontiers

# 🚨 맵은 TRANSIENT_LOCAL 이다. 기본 QoS 로 구독하면 에러 없이 아무것도 안 온다.
MAP_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
REL_QOS = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE)

# 이름 접두사 -> 종류. 소환기의 id_prefix 와 맞춘다.
PREFIXES = (('drone', 'drone'), ('spot', 'spot'), ('ugv', 'ugv'))


def robot_type_of(robot_id):
    for prefix, kind in PREFIXES:
        if robot_id.startswith(prefix):
            return kind
    return 'ugv'


class FrontierAllocatorNode(Node):

    def __init__(self):
        super().__init__('frontier_allocator')

        self.declare_parameter('period', 30.0)
        self.declare_parameter('strategy', 'distance')   # distance | llm
        self.declare_parameter('min_goal_dist', 4.0)
        self.declare_parameter('min_frontier_cells', 8)
        self.declare_parameter('cluster_size', 2.0)
        self.declare_parameter('max_candidates', 12)
        # 배정된 후보끼리 최소 이만큼(m) 떨어져야 한다. 안 그러면 군집 버킷이 2 m 라
        # 1.5 m 떨어진 사실상 같은 지점에 두 로봇이 배정돼 분산 효과가 사라진다(실측).
        self.declare_parameter('min_separation', 10.0)
        # 목표를 유지할 최대 주기 수. 이 안에 못 가면 버리고 다시 배정한다.
        self.declare_parameter('giveup_rounds', 3)
        # 한 주기에 이만큼도 못 움직였으면 "진전 없음"으로 센다.
        self.declare_parameter('progress_min', 1.0)
        # 증류용 학습 데이터. 비우면 안 쓴다.
        self.declare_parameter('dataset_path', '')

        self.period = float(self.get_parameter('period').value)
        self.strategy = str(self.get_parameter('strategy').value)
        self.min_goal_dist = float(self.get_parameter('min_goal_dist').value)
        self.min_cells = int(self.get_parameter('min_frontier_cells').value)
        self.cluster = float(self.get_parameter('cluster_size').value)
        self.max_cand = int(self.get_parameter('max_candidates').value)
        self.min_sep = float(self.get_parameter('min_separation').value)
        self.giveup = int(self.get_parameter('giveup_rounds').value)
        self.progress_min = float(self.get_parameter('progress_min').value)
        self.dataset_path = str(self.get_parameter('dataset_path').value).strip()

        self.merged = None
        self.pose = {}          # ns -> Pose
        self.bounds = {}        # ns -> (x0, y0, x1, y1)  계획 가능 범위
        self.goal_pub = {}
        self.target = {}        # ns -> (x, y)   지속 목표
        self.stuck = {}         # ns -> 진전 없는 주기 수
        self.last_pos = {}

        self.create_subscription(OccupancyGrid, '/map_merged', self._on_map, MAP_QOS)
        self.create_subscription(String, '/robot_registry', self._on_registry, 10)
        self.create_timer(self.period, self.cycle)
        self.get_logger().info(
            f'프론티어 할당기 시작 | 전략 {self.strategy} | 주기 {self.period}s | '
            f'최소 목표거리 {self.min_goal_dist} m | 최소 분리 {self.min_sep} m')

    # ------------------------------------------------------------- 편대 파악
    def _on_registry(self, msg):
        """로봇이 스스로 보내는 명함. 런타임 소환을 재시작 없이 따라간다."""
        try:
            info = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        ns = info.get('robot_id')
        if not ns or ns in self.goal_pub:
            return
        kind = robot_type_of(ns)
        self.create_subscription(
            Odometry, f'/{ns}/odom',
            lambda m, k=ns: self.pose.__setitem__(k, m.pose.pose), REL_QOS)
        # 드론의 Nav2 static layer 는 map_active(현재 순항 고도 층)다.
        topic = f'/{ns}/map_active' if kind == 'drone' else f'/{ns}/map'
        self.create_subscription(
            OccupancyGrid, topic,
            lambda m, k=ns: self.bounds.__setitem__(k, (
                m.info.origin.position.x, m.info.origin.position.y,
                m.info.origin.position.x + m.info.width * m.info.resolution,
                m.info.origin.position.y + m.info.height * m.info.resolution)),
            MAP_QOS)
        # 🚨 드론도 goal_pose 를 쓴다. goal_pose_3d 는 altitude_selector 를 거치는데,
        #    그 노드는 고른 층에 기체가 도달해야 Nav2 로 넘긴다. 창이 새 영역으로 가서
        #    세 층이 모두 미탐색이면 통과하는 층이 없어 목표를 영영 안 넘긴다(실측).
        self.goal_pub[ns] = self.create_publisher(PoseStamped, f'/{ns}/goal_pose', 10)
        self.get_logger().info(f'편대에 합류: {ns} ({kind})')

    def _on_map(self, msg):
        self.merged = msg

    # ------------------------------------------------------------- 목표 전송
    def clamp(self, ns, x, y):
        """계획 가능 범위 밖이면 로봇→목표 방향으로 범위 안까지만 자른다(경유점)."""
        b = self.bounds.get(ns)
        if not b or ns not in self.pose:
            return x, y
        m = 2.0                      # 가장자리에 딱 붙이면 코스트맵 경계에 걸린다
        x0, y0, x1, y1 = b[0] + m, b[1] + m, b[2] - m, b[3] - m
        if x0 <= x <= x1 and y0 <= y <= y1:
            return x, y
        rx, ry = self.pose[ns].position.x, self.pose[ns].position.y
        dx, dy = x - rx, y - ry
        s = 1.0
        for lo, hi, p, d in ((x0, x1, rx, dx), (y0, y1, ry, dy)):
            if abs(d) > 1e-6:
                s = min(s, ((hi if d > 0 else lo) - p) / d)
        s = max(0.0, min(1.0, s))
        return rx + dx * s, ry + dy * s

    def send_goal(self, ns, x, y):
        gx, gy = self.clamp(ns, x, y)
        g = PoseStamped()
        # 🚨 'map' 이 아니라 '{ns}/map'. 틀리면 에러 없이 무시된다.
        g.header.frame_id = f'{ns}/map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = float(gx), float(gy)
        g.pose.orientation.w = 1.0
        self.goal_pub[ns].publish(g)
        return gx, gy

    # ------------------------------------------------------------- 주기 처리
    def cycle(self):
        if self.merged is None:
            self.get_logger().warn(
                '/map_merged 를 아직 못 받았다 (QoS 를 먼저 의심할 것 — TRANSIENT_LOCAL)',
                throttle_duration_sec=30.0)
            return
        active = [ns for ns in self.goal_pub if ns in self.pose]
        if not active:
            self.get_logger().info('배정할 로봇이 없다', throttle_duration_sec=30.0)
            return

        info = self.merged.info
        grid = np.asarray(self.merged.data, dtype=np.int16).reshape(
            info.height, info.width)
        robots = [{'id': ns, 'type': robot_type_of(ns),
                   'x': self.pose[ns].position.x,
                   'y': self.pose[ns].position.y} for ns in sorted(active)]

        fr = extract_frontiers(grid, info.origin.position.x, info.origin.position.y,
                               info.resolution, robots[0]['x'], robots[0]['y'],
                               min_cells=self.min_cells, bucket=self.cluster,
                               max_n=self.max_cand * 4)
        # 어느 로봇에게도 너무 가깝지 않은 것만 (규칙 1)
        fr = [f for f in fr
              if min(math.hypot(f['x'] - r['x'], f['y'] - r['y']) for r in robots)
              >= self.min_goal_dist][:self.max_cand]
        for i, f in enumerate(fr):
            f['id'] = i
        if not fr:
            self.get_logger().info('프론티어 후보가 없다 — 탐사 완료이거나 맵이 비었다')
            return

        obs = build_observation(grid, info.origin.position.x, info.origin.position.y,
                                info.resolution, robots, fr)
        result = assign_by_distance(obs, min_separation=self.min_sep)  # ← LLM 이 대체할 자리
        fresh = {a['robot']: next(f for f in fr if f['id'] == a['frontier_id'])
                 for a in result['assignments']}

        for r in robots:
            ns = r['id']
            self._update_progress(ns, r)
            tgt = self._pick_target(ns, r, fr, fresh)
            if tgt is None:
                continue
            gx, gy = self.send_goal(ns, *tgt)
            self.get_logger().info(
                f'[{ns}] 목표 ({tgt[0]:.1f}, {tgt[1]:.1f})'
                + (f' -> 경유점 ({gx:.1f}, {gy:.1f})'
                   if abs(gx - tgt[0]) + abs(gy - tgt[1]) > 0.1 else ''))
        self._record(obs, result)

    def _update_progress(self, ns, r):
        """진전이 없으면 센다. 규칙 3 — 포기 조건의 근거."""
        prev = self.last_pos.get(ns)
        now = (r['x'], r['y'])
        if prev is not None:
            moved = math.hypot(now[0] - prev[0], now[1] - prev[1])
            self.stuck[ns] = 0 if moved >= self.progress_min else self.stuck.get(ns, 0) + 1
        self.last_pos[ns] = now

    def _pick_target(self, ns, r, fr, fresh):
        """지속 목표를 유지하되, 도달했거나 진전이 없으면 새로 받는다."""
        tgt = self.target.get(ns)
        if tgt:
            far = math.hypot(tgt[0] - r['x'], tgt[1] - r['y']) > self.min_goal_dist
            still = min((math.hypot(tgt[0] - f['x'], tgt[1] - f['y']) for f in fr),
                        default=1e9) < 6.0
            if far and still and self.stuck.get(ns, 0) < self.giveup:
                return tgt
            if self.stuck.get(ns, 0) >= self.giveup:
                self.get_logger().warn(
                    f'[{ns}] {self.giveup}주기 동안 진전이 없어 목표를 버린다')
                self.stuck[ns] = 0
        f = fresh.get(ns)
        if f is None:
            return None
        self.target[ns] = (f['x'], f['y'])
        return self.target[ns]

    def _record(self, obs, result):
        if not self.dataset_path:
            return
        try:
            with open(self.dataset_path, 'a', encoding='utf-8') as fh:
                fh.write(json.dumps({'observation': obs, 'result': result},
                                    ensure_ascii=False) + '\n')
        except OSError as exc:
            self.get_logger().warn(f'데이터셋 기록 실패: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierAllocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
