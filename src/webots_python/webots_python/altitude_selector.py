"""드론 고도 선택기 — 경로 1(2.5D 레이어드)의 판단부.

Nav2는 2D 플래너라 고도를 계획하지 않는다. 그 한 축을 이 노드가 바깥에서 담당한다.
**Nav2를 고치지 않고 3D 비슷한 회피를 얻는 것**이 경로 1의 요점이다.

    목표가 들어옴
        │
        ├─▶ 각 층의 격자에서 "지금 위치 -> 목표" 통로가 뚫렸는지 검사
        │      (2D A* 가 아니라 회랑 검사다. 아래 '왜 A*가 아닌가' 참고)
        │
        ├─▶ 뚫린 층 중 **가장 낮은 층**을 고른다
        │      낮을수록 라이다가 바닥을 잘 보고, 추락 시 피해가 작다
        │
        └─▶ 그 고도로 올라간(또는 내려간) 뒤 Nav2에 목표를 넘긴다
               고도 변경은 cmd_vel.linear.z (드라이버가 목표 고도를 적분한다)

왜 A* 가 아니라 회랑 검사인가
--------------------------------
층을 고르는 데 필요한 것은 "이 층으로 갈 수 있나" 뿐이고, 실제 경로는 어차피 Nav2가
다시 짠다. 층마다 A*를 돌리면 계획 비용이 층 수만큼 곱해진다 — 군집을 감당하려고
경로 2를 버렸는데 여기서 같은 비용을 도로 만들면 앞뒤가 안 맞는다.
직선 회랑 검사는 층당 수백 칸만 보므로 비용이 사실상 0이다.

  대가: 직선이 막혔지만 우회로는 있는 층을 놓친다(보수적). 그런 경우 Nav2가 현재
        층에서 우회를 시도하고, 그것마저 실패하면 다음 층으로 올라간다.
        즉 놓쳐도 기능이 깨지지 않고 한 번 더 시도하는 쪽으로 퇴화한다.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from std_msgs.msg import Float64, String

OCCUPIED_MIN = 50   # 이 값 이상이면 장애물로 본다
UNKNOWN = -1


class AltitudeSelector(Node):

    def __init__(self):
        super().__init__('altitude_selector')

        self.ns = self.declare_parameter('namespace', 'drone1').value
        self.layer_heights = [float(v) for v in self.declare_parameter(
            'layer_heights', [1.0, 2.0, 3.0]).value]
        # 회랑 반폭(m). 기체 반경 0.35 m + 여유.
        self.corridor_half = float(self.declare_parameter('corridor_half_width', 0.6).value)
        # 미탐색 칸을 얼마나 참아 줄 것인가 (0.0 = 하나도 못 참음, 1.0 = 무시).
        #
        # 처음에는 "미탐색이면 무조건 막힘"(하드 실패)으로 뒀는데, 실측에서 **어느 층도
        # 뚫린 적이 없었다**. 이유는 단순하다 — 라이다 minRange 가 1 m 라 드론 주변은
        # 원래 미탐색이고, 5.8 m 회랑이면 750칸 넘게 보는데 그중 하나만 미탐색이어도
        # 층이 탈락한다. 결과적으로 선택기가 항상 "현재 층"으로 퇴화했다.
        #
        # 그래서 기준을 **비율**로 바꿨다.
        self.max_unknown_ratio = float(
            self.declare_parameter('max_unknown_ratio', 0.35).value)

        # 장애물도 마찬가지로 비율로 본다. 처음에는 "한 칸이라도 있으면 탈락" 이었는데,
        # 잡동사니가 많은 실내에서는 폭 1.2 m × 길이 5 m 회랑이 거의 항상 무언가를
        # 스친다. 실측에서 **108개 방향 중 뚫린 곳이 0개**였다 — 정작 지도를 그려 보니
        # 드론 오른쪽은 5 m 넘게 비어 있었는데도 1.2 m 옆의 가구 한 덩이 때문에
        # 전 방향이 탈락한 것이다.
        #
        # 이 검사의 목적은 "직선이 완벽히 비었나" 가 아니라 **"어느 층이 목표 쪽으로
        # 더 열려 있나"** 다. 실제 경로는 어차피 Nav2 가 장애물을 피해 다시 짠다.
        # 그래서 약간의 스침은 허용하고, 층이 전부 탈락하면 **가장 덜 막힌 층**을 고른다
        # (choose_layer 참고). 안전은 Nav2 의 코스트맵과 인플레이션이 담당한다.
        self.max_occupied_ratio = float(
            self.declare_parameter('max_occupied_ratio', 0.02).value)
        self.climb_rate = float(self.declare_parameter('climb_rate', 0.4).value)
        self.altitude_tolerance = float(
            self.declare_parameter('altitude_tolerance', 0.25).value)

        self.n_layers = len(self.layer_heights)
        self.layers = [None] * self.n_layers
        self.pose = None
        self.pending_goal = None
        self.target_layer = None
        self.last_goal = None       # 중복 목표 무시용 (on_goal 참고)
        # 드라이버의 target_altitude 추정치와 마지막으로 보낸 상승 명령 (tick 참고)
        self.cmd_alt = None
        self.sent_vz = 0.0
        self.last_tick = 0.0

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        for k in range(self.n_layers):
            self.create_subscription(
                OccupancyGrid, f'/{self.ns}/map_layer_{k}',
                lambda m, kk=k: self.layers.__setitem__(kk, m), map_qos)
        self.create_subscription(Odometry, f'/{self.ns}/odom', self.on_odom, 10)

        # 사용자는 여기에 목표를 넣는다. 층을 맞춘 뒤 Nav2의 goal_pose로 넘긴다.
        self.create_subscription(
            PoseStamped, f'/{self.ns}/goal_pose_3d', self.on_goal, 10)

        # 🚨 cmd_vel 을 직접 쏘지 않는다. local_altitude_avoider 가 단독 소유한다 —
        #    둘 다 쏘면 z 를 두고 싸운다. 여기서는 "순항 고도" 만 알려주고,
        #    실제 상승·하강과 지역 회피는 그쪽이 한다.
        self.cruise_pub = self.create_publisher(
            Float64, f'/{self.ns}/cruise_altitude', 10)
        self.goal_pub = self.create_publisher(PoseStamped, f'/{self.ns}/goal_pose', 10)
        self.status_pub = self.create_publisher(String, f'/{self.ns}/altitude_status', 10)

        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'고도 선택기 시작 | 층 {self.layer_heights} | 회랑 반폭 {self.corridor_half} m')

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def corridor_stats(self, grid: OccupancyGrid, sx, sy, gx, gy):
        """(sx,sy) -> (gx,gy) 직선 회랑을 훑어 (장애물 있나, 미탐색 비율) 을 돌려준다.

        층마다 A* 를 돌리지 않는 이유는 모듈 주석에 있다. 여기서 필요한 것은
        "이 층으로 갈 수 있나" 뿐이고, 실제 경로는 어차피 Nav2 가 다시 짠다.
        """
        if grid is None:
            return 1.0, 1.0
        info = grid.info
        data = np.asarray(grid.data, dtype=np.int8).reshape(info.height, info.width)
        res = info.resolution
        length = math.hypot(gx - sx, gy - sy)
        if length < 1e-3:
            return 0.0, 0.0
        n = max(2, int(length / res))
        t = np.linspace(0.0, 1.0, n)
        cx = sx + t * (gx - sx)
        cy = sy + t * (gy - sy)
        # 회랑 폭만큼 좌우로도 확인한다 (진행방향에 수직인 오프셋)
        ux, uy = -(gy - sy) / length, (gx - sx) / length
        offs = np.arange(-self.corridor_half, self.corridor_half + res, res)
        n_occ = n_unknown = n_total = 0
        for o in offs:
            i = ((cx + o * ux - info.origin.position.x) / res).astype(np.int32)
            j = ((cy + o * uy - info.origin.position.y) / res).astype(np.int32)
            ok = (i >= 0) & (i < info.width) & (j >= 0) & (j < info.height)
            if not ok.any():
                continue
            v = data[j[ok], i[ok]]
            n_occ += int((v >= OCCUPIED_MIN).sum())
            n_unknown += int((v == UNKNOWN).sum())
            n_total += int(v.size)
        if n_total == 0:
            return 1.0, 1.0
        return n_occ / n_total, n_unknown / n_total

    def corridor_clear(self, grid: OccupancyGrid, sx, sy, gx, gy) -> bool:
        occ, unk = self.corridor_stats(grid, sx, sy, gx, gy)
        return occ <= self.max_occupied_ratio and unk <= self.max_unknown_ratio

    def choose_layer(self, gx, gy):
        """뚫린 층 중 가장 낮은 층 index. 없으면 None.

        낮은 층을 먼저 보는 이유: 낮을수록 라이다가 바닥을 잘 보고, 떨어졌을 때 피해가
        작고, 올라가는 데 드는 시간도 없다. 넘어야 할 때만 올라간다.
        """
        sx, sy = self.pose[0], self.pose[1]
        order = sorted(range(self.n_layers), key=lambda k: self.layer_heights[k])
        for k in order:
            if self.corridor_clear(self.layers[k], sx, sy, gx, gy):
                return k, True
        # 기준을 통과한 층이 없다 — 포기하지 말고 **가장 덜 막힌 층**을 고른다.
        # 장애물 비율을 먼저 보고, 같으면 미탐색이 적은 쪽, 그래도 같으면 낮은 층.
        best = min(order, key=lambda k: (
            round(self.corridor_stats(self.layers[k], sx, sy, gx, gy)[0], 4),
            round(self.corridor_stats(self.layers[k], sx, sy, gx, gy)[1], 4),
            self.layer_heights[k]))
        return best, False

    def layer_report(self, gx, gy) -> str:
        """왜 그 층을 골랐는지(혹은 왜 다 떨어졌는지) 한 줄로 남긴다.

        이게 없으면 "뚫린 층 없음" 만 보이고 장애물 때문인지 미탐색 때문인지를
        알 수 없다. 실제로 그 구분을 못 해서 한참 헤맸다.
        """
        sx, sy = self.pose[0], self.pose[1]
        parts = []
        for k in range(self.n_layers):
            g = self.layers[k]
            if g is None:
                parts.append(f'{self.layer_heights[k]:.0f}m:맵없음')
                continue
            occ, unk = self.corridor_stats(g, sx, sy, gx, gy)
            ok = occ <= self.max_occupied_ratio and unk <= self.max_unknown_ratio
            parts.append(f'{self.layer_heights[k]:.0f}m:'
                         f'{"OK" if ok else "X"}(장애물{occ*100:.1f}%/미탐색{unk*100:.0f}%)')
        return ' | '.join(parts)

    # ------------------------------------------------------------------
    def on_goal(self, msg: PoseStamped):
        if self.pose is None:
            self.get_logger().warn('odom이 아직 없어 목표를 버립니다')
            return
        gx, gy = msg.pose.position.x, msg.pose.position.y

        # 같은 목표가 여러 번 들어오면 무시한다. 웹/스크립트가 재발행하는 경우가 있고,
        # 그때마다 층을 다시 고르면 이미 Nav2 가 쫓고 있는 목표를 뺏어 고도부터
        # 다시 맞추느라 제자리에 선다 (실측으로 겪었다).
        if self.last_goal is not None:
            if math.hypot(gx - self.last_goal[0], gy - self.last_goal[1]) < 0.3:
                return
        self.last_goal = (gx, gy)

        k, strict = self.choose_layer(gx, gy)
        detail = self.layer_report(gx, gy)
        kind = '기준 통과' if strict else '차선(가장 덜 막힌 층)'
        self.report(f'층 선택: {self.layer_heights[k]:.1f} m — {kind} '
                    f'(현재 {self.pose[2]:.2f} m) [{detail}]')
        self.target_layer = k
        self.pending_goal = msg
        # 고도 명령 적분을 이 목표 기준으로 새로 시작한다
        self.cmd_alt = None
        self.sent_vz = 0.0

    def report(self, text: str):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def tick(self):
        """고도가 목표 층에 닿았는지 보고, 닿으면 Nav2 에 목표를 넘긴다.

        고도를 **움직이는 일 자체는 하지 않는다.** 순항 고도를 회피기에 알려주면
        그쪽이 cmd_vel.linear.z 로 몰고 간다 (그리고 지역 회피까지 겸한다).
        여기서 직접 쏘면 발행자가 둘이 되어 z 를 두고 싸운다.
        """
        if self.pose is None or self.pending_goal is None or self.target_layer is None:
            return
        want = self.layer_heights[self.target_layer]
        self.cruise_pub.publish(Float64(data=want))
        if abs(want - self.pose[2]) > self.altitude_tolerance:
            return
        goal = self.pending_goal
        goal.header.frame_id = f'{self.ns}/map'
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal)
        self.report(f'고도 {want:.1f} m 도달 — Nav2로 목표 전달')
        self.pending_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
