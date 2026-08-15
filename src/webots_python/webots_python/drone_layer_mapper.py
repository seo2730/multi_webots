"""드론용 층별 2D 점유 격자 매퍼.

`pointcloud_to_laserscan` + `slam_toolbox` **두 노드를 이것 하나로 대체한다.**

왜 SLAM을 안 쓰는가
--------------------
이 프로젝트의 드라이버는 GPS 절대좌표를 그대로 odom으로 발행한다. 즉 자세가
이미 정답값이라 스캔매칭으로 추정할 것이 없다. slam_toolbox는 사실상 "점유 격자
누적기"로만 쓰이고 있었고, 그 일은 여기서 훨씬 싸게 한다. 맵 병합이 이미
`odom_is_world_absolute: true`로 같은 가정 위에 서 있으므로 새 가정을 들이는 것도 아니다.

  대가: 루프 클로저와 드리프트 보정이 없다. 실기 이식 때는 3D SLAM이 필요하다.
        (시뮬 한정 단순화 — MAP_MERGE.md의 같은 항목과 짝을 이룬다)

이득은 군집에서 나온다. 드론 1대당 노드가 2개에서 1개로 줄고, 무거운 스캔매칭이
사라진다. 계획 비용은 **지금과 완전히 같은 2D A***로 유지된다 — 3D 플래너를 쓰지
않기로 한 이유가 그것이었다.

무엇을 발행하는가 (토픽 역할이 셋으로 갈린다)
-----------------------------------------------
    /{ns}/map            층 **합집합**       -> 맵 병합기 · 관제
    /{ns}/map_active     **현재 순항 고도**  -> 드론 자신의 Nav2 static layer
    /{ns}/map_layer_{k}  후보 층             -> altitude_selector

🚨 이 분리가 이 노드의 존재 이유다. 합집합을 드론 Nav2에 주면 **다른 고도의
   장애물 때문에 지금 고도에서는 뻥 뚫린 공간을 못 지나간다.** 고도를 바꿔
   장애물을 넘으려고 만든 기능이 오히려 지금보다 나빠진다.

   반대로 병합기에는 합집합을 줘야 한다. 병합 규칙이 `np.maximum`(장애물이 이긴다)
   이라, 드론이 3 m에서 본 "빈 곳"(0)은 UGV가 0.8 m에서 본 책상(100)에 어차피 진다.
   즉 합집합은 아무것도 지우지 않으면서 "드론이 어느 고도에서든 장애물을 본 곳"을
   정확히 표현한다. 현재 층만 올리면 다른 고도에서 본 장애물이 평면도에서 사라진다.

   `/{ns}/map`이라는 이름을 유지하는 것도 의도다. 병합기의
   `map_topic_pattern: '^/([^/]+)/map$'`에 그대로 걸리므로 **병합기는 한 줄도 안 고친다.**

입력
----
    /{ns}/Velodyne_VLP_16/point_cloud   수평 라이다 (±15°, minRange 1 m)
    /{ns}/down_depth/point_cloud        하향 뎁스 (발밑 — 라이다가 못 보는 곳)
    /{ns}/odom                          기체 자세 (월드 절대)
"""

import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


def yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2에서 xyz만 뽑아 (N,3) float32 배열로 돌려준다.

    sensor_msgs_py.point_cloud2.read_points는 구조화 배열을 만들면서 파이썬 루프에
    가까운 비용을 내므로, 필드 오프셋을 직접 읽어 numpy view로 처리한다. 라이다 한
    프레임이 57600점이라 이 차이가 그대로 CPU가 된다.
    """
    off = {f.name: f.offset for f in msg.fields if f.name in ('x', 'y', 'z')}
    if len(off) != 3:
        return np.empty((0, 3), dtype=np.float32)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    if n == 0 or msg.point_step == 0:
        return np.empty((0, 3), dtype=np.float32)
    buf = buf[:n * msg.point_step].reshape(n, msg.point_step)
    cols = [buf[:, o:o + 4].copy().view(np.float32).reshape(-1) for o in
            (off['x'], off['y'], off['z'])]
    pts = np.stack(cols, axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


class DroneLayerMapper(Node):

    def __init__(self):
        super().__init__('drone_layer_mapper')

        self.ns = self.declare_parameter('namespace', 'drone1').value
        self.resolution = float(self.declare_parameter('resolution', 0.1).value)
        # 격자 범위(월드 좌표, m). 맵 병합 해상도(0.1)와 맞춘다.
        self.origin_x = float(self.declare_parameter('origin_x', -10.0).value)
        self.origin_y = float(self.declare_parameter('origin_y', -8.0).value)
        self.width = int(self.declare_parameter('width', 200).value)
        self.height = int(self.declare_parameter('height', 160).value)

        # 후보 순항 고도. 층 간격은 기체 지름(0.7 m)보다 넉넉해야 한다.
        self.layer_heights = [float(v) for v in self.declare_parameter(
            'layer_heights', [1.0, 2.0, 3.0]).value]
        # 각 층의 두께(±). 기체가 그 층을 날 때 부딪힐 수 있는 높이 범위다.
        self.layer_half = float(self.declare_parameter('layer_half_height', 0.5).value)

        # 라이다 자기 몸 반사 방지 (minRange 1 m 와 같은 이유)
        self.min_range = float(self.declare_parameter('min_range', 1.05).value)
        self.max_range = float(self.declare_parameter('max_range', 20.0).value)
        # 군집에서 드론마다 붙는 비용이라 점을 솎아 쓴다. 격자 해상도보다 촘촘한
        # 점들은 어차피 같은 칸에 들어가므로 정보가 아니라 비용이다.
        self.stride = int(self.declare_parameter('cloud_stride', 4).value)
        self.publish_period = float(self.declare_parameter('publish_period', 1.0).value)
        # 점유 점수: 맞으면 +hit_gain, 광선이 통과하면 -1, 범위는 ±odds_limit.
        # hit_gain 이 크면 잘 안 잊고, 작으면 빨리 잊는다. 드론이라 장애물 쪽에
        # 조금 무게를 두되(3), 한계를 낮게 잡아(12) 몇 번만 통과해도 지워지게 한다.
        self.hit_gain = int(self.declare_parameter('hit_gain', 3).value)
        self.odds_limit = int(self.declare_parameter('odds_limit', 12).value)
        self.occupied_threshold = int(
            self.declare_parameter('occupied_threshold', 2).value)

        self.n_layers = len(self.layer_heights)
        shape = (self.n_layers, self.height, self.width)
        # 로그 오즈 대신 정수 카운터를 쓴다. 여기서 필요한 것은 확률이 아니라
        # "봤나 / 막혔나" 뿐이고, int8 배열이 훨씬 싸다.
        # 🚨 누적만 하면 **잊지 못한다.** 처음엔 hits/miss 를 따로 세고 hits>=2 면
        #    영구히 장애물로 뒀는데, 그러면 가구를 옮겨도 옛 자리가 계속 벽으로 남고
        #    회피하며 여러 고도를 지난 흔적까지 쌓여 **드론이 스스로를 가둔다.**
        #    실측: 목표 5개 중 3개가 ABORTED 였고, 직접 cmd_vel 을 주면 자유롭게
        #    움직였다 — 물리적으로 낀 게 아니라 지도가 막고 있었다.
        #
        #    그래서 점수 하나로 바꾼다. 맞으면 올리고, 광선이 통과하면 내린다.
        #    치운 물건 자리는 광선이 지나가면서 점수가 떨어져 저절로 비워진다.
        self.odds = np.zeros(shape, dtype=np.int16)
        self.seen = np.zeros(shape, dtype=bool)

        self.pose = None            # (x, y, z, yaw)
        self.active_layer = self.closest_layer(2.0)

        self.create_subscription(Odometry, f'/{self.ns}/odom', self.on_odom, 10)
        cloud_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, f'/{self.ns}/Velodyne_VLP_16/point_cloud',
            self.on_lidar, cloud_qos)
        self.create_subscription(
            PointCloud2, f'/{self.ns}/down_depth/point_cloud',
            self.on_down, cloud_qos)

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub_union = self.create_publisher(OccupancyGrid, f'/{self.ns}/map', map_qos)
        self.pub_active = self.create_publisher(
            OccupancyGrid, f'/{self.ns}/map_active', map_qos)
        self.pub_layers = [
            self.create_publisher(OccupancyGrid, f'/{self.ns}/map_layer_{k}', map_qos)
            for k in range(self.n_layers)]

        self.create_timer(self.publish_period, self.publish_all)
        self.create_subscription(
            OccupancyGrid, f'/{self.ns}/map_active_request', self.on_active_request, 1)

        self.get_logger().info(
            f'층별 매퍼 시작 | 층 {self.layer_heights} (±{self.layer_half} m) | '
            f'격자 {self.width}x{self.height} @ {self.resolution} m | stride {self.stride}')

    # ------------------------------------------------------------------
    def closest_layer(self, z: float) -> int:
        return int(np.argmin([abs(z - h) for h in self.layer_heights]))

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z, yaw_from_quaternion(msg.pose.pose.orientation))
        # 순항 고도가 바뀌면 Nav2에 주는 층도 따라 바뀐다.
        self.active_layer = self.closest_layer(p.z)

    def on_active_request(self, _msg):
        pass  # 예약: 선택기가 층을 강제하고 싶을 때 쓸 자리

    # ------------------------------------------------------------------
    def world_from_body(self, pts: np.ndarray) -> np.ndarray:
        """센서 프레임 점들을 월드 좌표로 옮긴다.

        TF 조회 대신 odom을 직접 쓴다. 드라이버가 GPS 절대좌표를 그대로 발행하므로
        odom이 곧 월드 자세이고, 회전은 yaw 하나면 된다 (호버 중 roll/pitch는 2° 안쪽 —
        drone_setup.md의 실측 참고). tf2 조회를 매 클라우드마다 하는 비용이 사라진다.
        """
        x, y, z, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        wx = c * pts[:, 0] - s * pts[:, 1] + x
        wy = s * pts[:, 0] + c * pts[:, 1] + y
        wz = pts[:, 2] + z
        return np.stack([wx, wy, wz], axis=1)

    def to_cells(self, wx: np.ndarray, wy: np.ndarray):
        i = ((wx - self.origin_x) / self.resolution).astype(np.int32)
        j = ((wy - self.origin_y) / self.resolution).astype(np.int32)
        ok = (i >= 0) & (i < self.width) & (j >= 0) & (j < self.height)
        return i, j, ok

    def integrate(self, world_pts: np.ndarray, trace_free: bool):
        """월드 좌표 점들을 층별 격자에 넣는다."""
        if world_pts.shape[0] == 0:
            return
        rx, ry = self.pose[0], self.pose[1]
        for k, h in enumerate(self.layer_heights):
            band = np.abs(world_pts[:, 2] - h) <= self.layer_half
            if not band.any():
                continue
            px, py = world_pts[band, 0], world_pts[band, 1]
            i, j, ok = self.to_cells(px, py)
            np.add.at(self.odds[k], (j[ok], i[ok]), self.hit_gain)
            self.seen[k][j[ok], i[ok]] = True

            if not trace_free:
                continue
            # 자유 공간: 센서에서 각 점까지 직선을 샘플링해 비었다고 표시한다.
            # 정식 Bresenham 대신 균등 샘플링을 쓴다 — 격자 한 칸(0.1 m)보다 촘촘히
            # 뽑으면 빠지는 칸이 없고, 전부 벡터 연산이라 훨씬 싸다.
            d = np.hypot(px - rx, py - ry)
            steps = int(min(self.max_range, float(d.max())) / self.resolution) + 1
            steps = max(1, min(steps, 400))
            t = np.linspace(0.0, 1.0, steps, dtype=np.float32)[:-1]  # 끝점 제외(=장애물)
            fx = rx + np.outer(t, px - rx)
            fy = ry + np.outer(t, py - ry)
            fi, fj, fok = self.to_cells(fx.reshape(-1), fy.reshape(-1))
            np.add.at(self.odds[k], (fj[fok], fi[fok]), -1)
            self.seen[k][fj[fok], fi[fok]] = True
        np.clip(self.odds, -self.odds_limit, self.odds_limit, out=self.odds)

    def on_lidar(self, msg: PointCloud2):
        if self.pose is None:
            return
        pts = read_xyz(msg)[::self.stride]
        if pts.shape[0] == 0:
            return
        d = np.linalg.norm(pts[:, :2], axis=1)
        pts = pts[(d >= self.min_range) & (d <= self.max_range)]
        if pts.shape[0] == 0:
            return
        # 라이다는 회전 없이 base_link 위 0.12 m 에 달려 있다 (래퍼 PROTO 참고).
        # 즉 점들이 이미 기체 축과 같은 방향이라 오프셋만 더하면 된다.
        pts = pts.copy()
        pts[:, 2] += 0.12
        self.integrate(self.world_from_body(pts), trace_free=True)

    def on_down(self, msg: PointCloud2):
        """하향 뎁스 — 라이다의 발밑 사각(반경 3.73h)을 메운다.

        🚨 이 센서는 **회전해서** 달려 있다. PROTO 의 `rotation 0 1 0 1.5708` 이
           센서의 +x(시선 방향)를 기체의 -z(아래)로 보낸다. 그래서 점을 그대로
           기체 좌표로 쓰면 안 된다.

           y 축 +90° 회전은  (sx, sy, sz) -> (sz, sy, -sx)  이다.
           확인: 센서 정면 (1,0,0) -> (0,0,-1) = 똑바로 아래. 맞다.

           이걸 빠뜨리면 **발밑의 바닥이 "정면 1.9 m 앞의 벽"으로 찍힌다.**
           층 지도마다 유령 장애물이 생기고 Nav2 가 제자리에서 못 움직인다
           (실측으로 겪었다 — linear.x 0.5 를 15초 줘도 0.33 m 밖에 못 갔다).

        자유공간 레이트레이싱은 하지 않는다. 아래를 보는 센서라 광선이 수평면을
        거의 지나가지 않아서, 2D 층 격자에 그릴 "빈 곳"이 사실상 없다.
        여기서 필요한 정보는 "발밑 어느 높이에 무엇이 있나" 뿐이다.
        """
        if self.pose is None:
            return
        pts = read_xyz(msg)[::self.stride]
        if pts.shape[0] == 0:
            return
        # 센서 축 -> 기체 축
        body = np.stack([pts[:, 2], pts[:, 1], -pts[:, 0]], axis=1)
        # 장착 위치: base_link 기준 (-0.12, 0, -0.12)
        body[:, 0] -= 0.12
        body[:, 2] -= 0.12
        self.integrate(self.world_from_body(body), trace_free=False)

    # ------------------------------------------------------------------
    def mark_self_free(self):
        """드론이 지금 있는 자리를 비어 있다고 표시한다.

        라이다 minRange 가 1 m 라 **드론은 자기 주변 1 m 를 영원히 관측하지 못한다.**
        그대로 두면 기체가 늘 미탐색 구멍 한가운데 앉아 있게 되고, 두 군데가 망가진다.

          - 회랑 검사가 출발점부터 미탐색이라 층을 못 고른다
          - Nav2 코스트맵에서 로봇이 미탐색 공간에 있어 플래너가 흔들린다

        드론이 그 자리에 떠 있다는 사실 자체가 "거긴 비어 있다" 는 관측이다.
        (지나온 자리도 같은 근거로 남는다 — 누적 격자라 지워지지 않는다)
        """
        if self.pose is None:
            return
        r = int(math.ceil(0.7 / self.resolution))   # 기체 반경 0.35 m 의 두 배
        i0 = int((self.pose[0] - self.origin_x) / self.resolution)
        j0 = int((self.pose[1] - self.origin_y) / self.resolution)
        k = self.active_layer
        i_lo, i_hi = max(0, i0 - r), min(self.width, i0 + r + 1)
        j_lo, j_hi = max(0, j0 - r), min(self.height, j0 + r + 1)
        if i_lo < i_hi and j_lo < j_hi:
            self.odds[k][j_lo:j_hi, i_lo:i_hi] -= 1
            self.seen[k][j_lo:j_hi, i_lo:i_hi] = True

    def grid_of(self, k: int) -> np.ndarray:
        """층 k의 점유 격자를 만든다."""
        g = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)
        g[self.seen[k]] = FREE
        g[self.odds[k] >= self.occupied_threshold] = OCCUPIED
        return g

    def msg_of(self, grid: np.ndarray) -> OccupancyGrid:
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = f'{self.ns}/map'
        m.info.resolution = self.resolution
        m.info.width = self.width
        m.info.height = self.height
        m.info.origin.position.x = self.origin_x
        m.info.origin.position.y = self.origin_y
        m.info.origin.orientation.w = 1.0
        m.data = grid.reshape(-1).tolist()
        return m

    def publish_all(self):
        if self.pose is None:
            return
        self.mark_self_free()
        grids = [self.grid_of(k) for k in range(self.n_layers)]
        for k, g in enumerate(grids):
            self.pub_layers[k].publish(self.msg_of(g))

        self.pub_active.publish(self.msg_of(grids[self.active_layer]))

        # 합집합: 장애물 OR. 병합기의 np.maximum 규칙과 같은 연산이라
        # "드론이 어느 고도에서든 장애물을 본 곳"이 그대로 보존된다.
        union = grids[0]
        for g in grids[1:]:
            union = np.maximum(union, g)
        self.pub_union.publish(self.msg_of(union))


def main(args=None):
    rclpy.init(args=args)
    node = DroneLayerMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
