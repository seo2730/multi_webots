"""드론 지역(local) 고도 회피 — 앞이 막히면 넘어간다.

전역 3D 경로계획(경로 2)과 다른 문제다. 여기서는 **계획을 하지 않는다.**
앞을 보고 "막혔나 / 위가 비었나" 만 판단해 `linear.z` 를 얹는다.
그래서 비용이 군집에서 대수만큼 곱해지지 않는다 — 계획 호출이 없기 때문이다.

    Nav2 ──▶ /{ns}/cmd_vel_nav ──┐
                                  ├──▶ [이 노드] ──▶ /{ns}/cmd_vel ──▶ 드라이버
    라이다 클라우드 ─────────────┘        linear.x/y/angular.z 는 Nav2 것 그대로,
    층별 지도 ───────────────────┘        linear.z 만 여기서 채운다

왜 이게 성립하나 (세 가지가 이미 맞물려 있다)
-----------------------------------------------
1. 드라이버가 `linear.x/y/z` 를 **동시에** 받는다. 그리고 Nav2 는 `linear.z` 를
   항상 0 으로 둔다 — 즉 z 축은 통째로 비어 있어서 충돌 없이 쓸 수 있다.
2. `drone_layer_mapper` 의 `map_active` 가 **고도를 따라 자동으로 바뀐다.**
   올라가면 Nav2 의 static layer 에서 그 장애물이 사라지므로, 우회를 시도하다 말고
   그대로 직진한다. 수평 회피와 수직 회피가 서로 싸우지 않는다.
3. 라이다가 ±15° 라 전방 몇 m 구간의 **위아래를 이미 보고 있다.** 따로 센서를
   더 달 필요가 없다.

판단 방식
---------
전방 룩어헤드 상자 안의 점을 센다 (기체 좌표, 이미 축이 맞아 있다).

    x ∈ [minRange, lookahead]        앞쪽
    |y| < half_width                 폭
    z ∈ [-block_below, block_above]  **기체가 실제로 닿는 높이**

높이 대역이 기체 크기여야 하는 이유가 중요하다. ±0.4 m 로 뭉뚱그리면 윗면이
0.38 m 아래인 낮은 가구까지 위험으로 보고 **닿지도 않을 것을 피한다.**

막혔으면 장애물 윗면을 라이다로 재서 `현재고도 + 윗면 + clearance` 로 올라간다.
층 눈금(1 m)에 스냅시키면 20 cm 턱을 넘으려고 1 m 를 오른다. 다만 라이다 수직
시야가 ±15° 라 거리 d 에서 0.268d 까지만 보이므로, 꼭대기가 그 한계에 닿으면
못 잰 것으로 보고 층 단위로 물러난다.

세 가지 함정이 있고 전부 실측으로 겪었다.

1. **미탐색을 막힘으로 보면 안 된다.** 안 가 본 곳의 위층은 대부분 미탐색이라
   "넘어갈 층 없음" 만 나온다. 지역 반응은 낙관적으로 보고 0.1 초 뒤 다시 판단한다.
2. **진행 중일 때만 회피한다.** 감지 상자가 기체와 함께 돌기 때문에, 제자리 요잉만으로도
   주변을 훑어 사방의 장애물을 잡는다 — 겉보기에 360° 를 다 보는 것처럼 되고 정지
   중에도 회피를 시작한다.
3. **복귀 조건은 "지금 비었나" 가 아니라 "순항 고도로 내려가도 되나" 다.** 전자로 두면
   올라가는 순간 스스로 "트였다" 고 판정하고 내려왔다가 다시 올라가는 **리밋 사이클**에
   빠져 위아래로 계속 왔다갔다한다.

⚠️ 고도를 바꾸는 동안에는 **수평 속도를 줄인다.** 안 그러면 다 올라가기 전에
   장애물에 닿는다. 실기에서 흔히 쓰는 방식이고, 여기서도 필요하다.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String

OCCUPIED_MIN = 50


def read_xyz(msg: PointCloud2) -> np.ndarray:
    off = {f.name: f.offset for f in msg.fields if f.name in ('x', 'y', 'z')}
    if len(off) != 3:
        return np.empty((0, 3), dtype=np.float32)
    n = msg.width * msg.height
    if n == 0 or msg.point_step == 0:
        return np.empty((0, 3), dtype=np.float32)
    buf = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step]
    buf = buf.reshape(n, msg.point_step)
    cols = [buf[:, o:o + 4].copy().view(np.float32).reshape(-1)
            for o in (off['x'], off['y'], off['z'])]
    pts = np.stack(cols, axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


class LocalAltitudeAvoider(Node):

    def __init__(self):
        super().__init__('local_altitude_avoider')

        self.ns = self.declare_parameter('namespace', 'drone1').value
        self.layer_heights = [float(v) for v in self.declare_parameter(
            'layer_heights', [1.0, 2.0, 3.0]).value]
        self.cruise = float(self.declare_parameter('cruise_altitude', 2.0).value)

        # 룩어헤드 상자
        self.lookahead = float(self.declare_parameter('lookahead', 3.0).value)
        self.half_width = float(self.declare_parameter('half_width', 0.7).value)
        # 부딪히는 높이 범위. **기체의 실제 수직 크기 + 여유**여야 한다.
        #
        # 처음에 ±0.4 m 로 뭉뚱그려 뒀더니 **닿지도 않을 것을 피했다.**
        # 실측 로그: "전방 2.5 m 에 높이 -0.38 m 장애물 → 회피" — 윗면이 드론보다
        # 0.38 m 아래라 그냥 지나가면 되는데 0.3 m 를 올라갔다.
        #
        # PROTO 기준 기체의 수직 범위는 이렇다.
        #   라이다 윗면      +0.156 m  (장착 0.12 + 퍽 높이 절반 0.036)
        #   랜딩기어 아랫면  -0.138 m
        # 여기에 자세 기울기(순항 중 2° 안쪽 → 2.5 m 앞에서 0.09 m)와 제어 오차를
        # 더해 위아래 각각 0.1 m 정도 여유를 준다.
        self.block_above = float(self.declare_parameter('block_above', 0.25).value)
        self.block_below = float(self.declare_parameter('block_below', 0.24).value)
        self.min_range = float(self.declare_parameter('min_range', 1.05).value)
        # 이 개수 이상이면 장애물로 본다. 라이다 한 점은 노이즈일 수 있다.
        self.hit_threshold = int(self.declare_parameter('hit_threshold', 8).value)
        # 룩어헤드가 이만큼 연속으로 비어야 순항 고도로 돌아간다 (히스테리시스).
        self.clear_hold = int(self.declare_parameter('clear_hold', 12).value)

        # 지역 고도 회피를 할 것인가 (nav_mode 가 정한다 — single_drone.launch.py 참고).
        #
        # False 여도 이 노드는 계속 돈다. cmd_vel 을 단독 발행하고 순항 고도를 잡는 일,
        # 그리고 **발밑 안전 바닥**은 경로계획 모드와 무관한 기본 기능이기 때문이다.
        # 끄는 것은 "앞이 막히면 넘어가는" 판단뿐이다.
        self.avoid_enabled = bool(self.declare_parameter('avoid_enabled', True).value)
        # 전진 명령이 이보다 작으면 "안 가고 있다" 로 보고 회피를 시작하지 않는다.
        # 감지 상자가 기체와 함께 돌기 때문에, 이게 없으면 제자리 요잉만으로도
        # 주변을 훑으며 회피를 시작한다 (tick 주석 참고).
        self.move_threshold = float(self.declare_parameter('move_threshold', 0.05).value)
        # 장애물 윗면 위로 얼마나 여유를 두고 넘을 것인가. 상승량을 직접 정한다.
        self.clearance = float(self.declare_parameter('clearance', 0.5).value)
        # 발밑 표면 위로 반드시 남겨 둘 여유. 랜딩기어가 -0.138 m 이므로 그보다 커야 하고,
        # 하강 중 오버슈트와 자세 기울기까지 감안해 넉넉히 잡는다.
        self.ground_clearance = float(
            self.declare_parameter('ground_clearance', 0.45).value)
        # 발밑 판정에 쓸 기체 발자국 반경(m). 이보다 밖의 화소는 옆 바닥을 본다.
        self.foot_radius = float(self.declare_parameter('foot_radius', 0.5).value)
        self.climb_rate = float(self.declare_parameter('climb_rate', 0.5).value)
        self.slow_gain = float(self.declare_parameter('slow_gain', 1.2).value)

        self.n_layers = len(self.layer_heights)
        self.layers = [None] * self.n_layers
        self.pose = None            # (x, y, z, yaw)
        self.nav_twist = Twist()
        self.desired_alt = self.cruise
        self.clear_count = 0
        self.state = 'cruise'
        self.blocked = False
        self.block_x = None
        self.block_top = None
        self.block_top_reliable = False
        self.cruise_blocked = False
        self.surface_below = None   # 발밑 표면의 월드 높이 (없으면 None)

        cloud_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, f'/{self.ns}/odom', self.on_odom, 10)
        self.create_subscription(Twist, f'/{self.ns}/cmd_vel_nav', self.on_nav, 10)
        # 🚨 `cmd_vel` 은 **이 노드만** 발행한다. altitude_selector 가 직접 쏘면
        #    두 발행자가 z 를 두고 싸운다. 선택기는 순항 고도만 여기로 알려준다.
        self.create_subscription(
            Float64, f'/{self.ns}/cruise_altitude', self.on_cruise, 10)
        self.create_subscription(
            PointCloud2, f'/{self.ns}/Velodyne_VLP_16/point_cloud',
            self.on_cloud, cloud_qos)
        # 🚨 **발밑**. 이게 없으면 장애물 위에 내려앉는다 (on_down 주석 참고).
        self.create_subscription(
            PointCloud2, f'/{self.ns}/down_depth/point_cloud',
            self.on_down, cloud_qos)
        for k in range(self.n_layers):
            self.create_subscription(
                OccupancyGrid, f'/{self.ns}/map_layer_{k}',
                lambda m, kk=k: self.layers.__setitem__(kk, m), map_qos)

        self.cmd_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, f'/{self.ns}/avoid_status', 10)

        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'고도 제어기 시작 | 지역 회피 {"ON" if self.avoid_enabled else "OFF (2d 모드)"} '
            f'| 순항 {self.cruise} m | 룩어헤드 {self.lookahead} m | 층 {self.layer_heights}')

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, p.z, yaw)

    def on_nav(self, msg: Twist):
        self.nav_twist = msg

    def on_cruise(self, msg: Float64):
        """altitude_selector 가 고른 순항 고도. 회피 중이 아니면 여기로 간다."""
        if abs(msg.data - self.cruise) < 1e-3:
            return
        self.cruise = float(msg.data)
        if self.state == "cruise":
            self.desired_alt = self.cruise
        self.report(f"순항 고도 갱신 -> {self.cruise:.1f} m")

    def on_cloud(self, msg: PointCloud2):
        """전방 상자 안의 점을 세서 '앞이 막혔나' 를 갱신한다."""
        if self.pose is None:
            return
        pts = read_xyz(msg)[::4]
        if pts.shape[0] == 0:
            return
        # 라이다는 회전 없이 base_link 위 0.12 m — 축이 이미 기체와 같다.
        z = pts[:, 2] + 0.12
        ahead = ((pts[:, 0] > self.min_range) & (pts[:, 0] < self.lookahead) &
                 (np.abs(pts[:, 1]) < self.half_width))
        # 기체가 실제로 닿는 높이대에 있는 점만 "막힘" 으로 센다.
        # 이 대역 아래로 지나가는 것(낮은 가구)이나 위로 지나가는 것은 위협이 아니다.
        box = ahead & (z > -self.block_below) & (z < self.block_above)
        hits = int(box.sum())
        self.blocked = hits >= self.hit_threshold
        self.block_x = float(pts[box, 0].min()) if hits else None

        # 🚨 **순항 고도로 내려가도 되는가** 를 따로 센다.
        #
        # 복귀 판정을 "지금 고도에서 비었나" 로 하면 리밋 사이클에 빠진다.
        # 올라가면 장애물이 대역 밖으로 나가므로 스스로 "트였다" 고 판정하고,
        # 내려오면 다시 들어와서 또 올라간다 — 위아래로 계속 왔다갔다한다
        # (실측으로 겪었다). 물어야 할 것은 "지금 비었나" 가 아니라
        # **"내려가도 되나"** 이고, 그건 순항 고도 기준으로 세야 안다.
        dz = self.cruise - self.pose[2]      # 순항 고도가 지금 기준 어디인가
        cruise_box = (ahead & (z > dz - self.block_below) &
                      (z < dz + self.block_above))
        self.cruise_blocked = int(cruise_box.sum()) >= self.hit_threshold

        # --- 장애물이 얼마나 높은가 ---
        #
        # 층 눈금(1 m 간격)으로 올라가면 20 cm 턱을 넘으려고 1 m 를 오른다.
        # 그래서 라이다로 장애물 윗면을 직접 재서 **딱 그만큼만** 오른다.
        #
        # ⚠️ 단, 이 라이다는 수직 시야가 ±15° 라 거리 d 에서 위로 0.268d 까지밖에
        #    못 본다. 장애물 꼭대기가 그 한계에 닿아 있으면 "더 위는 안 보이는 것"이지
        #    "거기가 끝인 것"이 아니다. 그때는 재는 것을 포기하고 층으로 물러난다.
        self.block_top = None
        self.block_top_reliable = False
        if hits and self.block_x is not None:
            # 윗면을 잴 때는 대역보다 아래까지 본다 — 막고 있는 물체의 몸통 전체를
            # 봐야 꼭대기를 알 수 있다. 다만 바닥 반사는 빼야 하므로 1 m 아래까지만.
            col = ahead & (z > -1.0)
            if col.any():
                top = float(z[col].max())
                cone = 0.268 * float(pts[col, 0].max())   # 그 거리에서 볼 수 있는 최대 높이
                self.block_top = top
                self.block_top_reliable = bool(top < cone - 0.15)

    def on_down(self, msg: PointCloud2):
        """하향 뎁스 — **발밑에 무엇이 있는지.** 내려가도 되는지를 이게 정한다.

        🚨 왜 필요한가 (실측으로 겪은 문제)
           복귀 판정("장애물 지나감")은 전방 룩어헤드 상자로만 한다. 그런데 장애물이
           드론 **바로 아래**로 들어오는 순간 그 상자에서 빠진다 — x > minRange(1.05 m)
           앞쪽만 보기 때문이다. 그러면 "지나갔다" 고 판정하고 내려가는데 실제로는
           아직 그 위다. 라이다는 minRange 와 ±15° 원뿔 때문에 발밑을 못 본다.
           결과: **장애물 위에 착지한다.**

           하향 센서를 달아 두고도 여기 안 쓰고 있었다(층 지도에만 썼다). 연결한다.

        ⚠️ 이 클라우드는 센서 프레임이고 +x 가 시선(아래)이다. 그래서 x 성분이 곧
           수직 낙차이고, (y, z) 가 지면상의 가로 위치다 — drone_layer_mapper 의
           on_down 과 같은 규약이다.
        """
        if self.pose is None:
            return
        pts = read_xyz(msg)[::2]
        if pts.shape[0] == 0:
            self.surface_below = None
            return
        # 기체 발자국 바로 아래만 본다. 비스듬한 가장자리 화소는 옆의 바닥을 보므로
        # 여기 판단에 쓰면 "내려가도 된다" 고 잘못 말한다.
        lateral = np.hypot(pts[:, 1], pts[:, 2])
        near = lateral < self.foot_radius
        if not near.any():
            self.surface_below = None
            return
        drop = float(pts[near, 0].min())          # 가장 가까운 표면까지의 낙차
        # 센서는 base_link 아래 0.12 m 에 있다.
        self.surface_below = self.pose[2] - 0.12 - drop

    # ------------------------------------------------------------------
    def cell_free(self, layer_idx: int, x: float, y: float, r: float = 0.7) -> bool:
        """층 지도에서 (x,y) 주변에 **장애물이 없는가.**

        🚨 미탐색을 '막힘' 으로 보면 안 된다. 처음에 그렇게 뒀더니 회피기가 계속
           "넘어갈 층 없음" 만 냈다 — 당연하다. 아직 안 가 본 곳의 위층은 대부분
           미탐색이고, 라이다 ±15° 로는 3 m 앞의 1 m 위를 볼 수 없다(그 거리에서
           수직 시야가 ±0.8 m 뿐이다). 즉 "위가 비었다는 증거" 는 원래 잘 안 모인다.

           전역 선택기(altitude_selector)는 보수적으로 가는 게 맞지만, 여기는
           **지역 반응**이라 판단 기준이 달라야 한다. 올라가 보고 아니면 0.1초 뒤에
           다시 판단하면 되기 때문이다. 그래서 여기서는 낙관적으로 본다 —
           **아는 장애물만 피하고, 모르는 곳은 가 본다.**
        """
        g = self.layers[layer_idx]
        if g is None:
            return False
        info = g.info
        d = np.asarray(g.data, dtype=np.int8).reshape(info.height, info.width)
        s = int(r / info.resolution)
        i = int((x - info.origin.position.x) / info.resolution)
        j = int((y - info.origin.position.y) / info.resolution)
        if not (s <= i < info.width - s and s <= j < info.height - s):
            return False
        return bool((d[j - s:j + s + 1, i - s:i + s + 1] < OCCUPIED_MIN).all())

    def pick_escape_layer(self):
        """막힌 지점 위(우선) 또는 아래에서 비어 있는 층을 고른다."""
        if self.pose is None or self.block_x is None:
            return None
        x, y, z, yaw = self.pose
        # 막힌 지점을 조금 지난 곳을 본다 — 거기가 비어야 넘어갈 수 있다.
        ahead = self.block_x + 1.0
        bx = x + ahead * math.cos(yaw)
        by = y + ahead * math.sin(yaw)
        cur = int(np.argmin([abs(z - h) for h in self.layer_heights]))
        # 위쪽부터 (낮은 순으로 훑되 현재보다 높은 층만)
        for k in range(cur + 1, self.n_layers):
            if self.cell_free(k, bx, by):
                return k
        for k in range(cur - 1, -1, -1):
            if self.cell_free(k, bx, by):
                return k
        return None

    def report(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def tick(self):
        if self.pose is None:
            return

        # 🚨 **진행 중일 때만** 회피한다.
        #
        # 감지 상자는 기체 +x(정면) 기준이라 원래 정면만 본다. 그런데 드론이 요잉하면
        # 그 상자가 같이 돌아가서 결과적으로 주변을 훑는다 — 겉보기에 360° 를 다
        # 확인하는 것처럼 보이고, 심지어 **정지 중에도** 회피를 시작한다.
        # Nav2 가 전진을 명령하지 않으면 부딪힐 일도 없으므로 그냥 넘긴다.
        # 회피가 꺼져 있으면(2d 모드) 순항 고도만 지킨다. 발밑 안전 바닥은 publish()
        # 안에 있으므로 여기서 빠져나가도 그대로 적용된다.
        if not self.avoid_enabled:
            self.desired_alt = self.cruise
            self.publish()
            return

        moving = abs(self.nav_twist.linear.x) > self.move_threshold
        if not moving:
            # 회피 중이었다면 유지한다(내려오다 부딪히지 않게). 새로 시작만 안 한다.
            self.publish()
            return

        blocked = self.blocked

        if blocked:
            self.clear_count = 0
            if self.state != 'avoid':
                # ① 장애물 윗면을 믿을 수 있게 쟀으면 그만큼만 오른다 (층 눈금 무시).
                if self.block_top_reliable:
                    self.desired_alt = min(
                        self.pose[2] + self.block_top + self.clearance,
                        max(self.layer_heights) + self.clearance)
                    self.state = 'avoid'
                    self.report(
                        f'전방 {self.block_x:.1f} m 에 높이 {self.block_top:+.2f} m 장애물 → '
                        f'{self.desired_alt:.2f} m 로 회피 (필요한 만큼만)')
                    self.publish(); return
                # ② 꼭대기가 시야 밖이라 못 쟀다 — 층으로 물러난다.
                k = self.pick_escape_layer()
                if k is not None:
                    self.desired_alt = self.layer_heights[k]
                    self.state = 'avoid'
                    self.report(f'전방 {self.block_x:.1f} m 막힘 (꼭대기 안 보임) → '
                                f'{self.desired_alt:.1f} m 층으로 회피')
                else:
                    # 넘어갈 층이 없다 — 고도는 그대로 두고 Nav2 의 수평 우회에 맡긴다.
                    self.report(f'전방 {self.block_x:.1f} m 막힘 → '
                                f'넘어갈 층 없음, 수평 우회에 맡김')
                    self.state = 'blocked_no_escape'
        else:
            # 복귀 조건은 "지금 비었나" 가 아니라 **"순항 고도로 내려가도 되나"** 다.
            # (그 이유는 on_cloud 의 cruise_box 주석에 있다 — 리밋 사이클)
            if self.cruise_blocked:
                self.clear_count = 0
            else:
                self.clear_count += 1
            if self.state != 'cruise' and self.clear_count >= self.clear_hold:
                self.state = 'cruise'
                self.desired_alt = self.cruise
                self.report(f'장애물 지나감 → 순항 {self.cruise:.1f} m 로 복귀')

        self.publish()

    def publish(self):
        """수평은 Nav2 것 그대로, 수직만 여기서 채워 내보낸다."""
        out = Twist()
        out.linear.x = self.nav_twist.linear.x
        out.linear.y = self.nav_twist.linear.y
        out.angular.z = self.nav_twist.angular.z

        # 🚨 **발밑 안전 바닥.** 아래에 무엇이 있으면 그 위 ground_clearance 아래로는
        #    절대 내려가지 않는다. 목표 고도가 무엇이든 여기서 잘린다.
        #
        #    이게 없으면 장애물을 넘은 직후(아직 그 위인데) 순항 고도로 복귀하다가
        #    **장애물 위에 착지한다.** 전방만 보는 복귀 판정으로는 막을 수 없고,
        #    라이다도 발밑을 못 보므로 하향 센서만이 알 수 있다.
        target = self.desired_alt
        if self.surface_below is not None:
            floor = self.surface_below + self.ground_clearance
            if target < floor:
                target = floor
                if self.state != 'floor_limited':
                    self.report(f'발밑 {self.surface_below:.2f} m 에 무언가 있음 → '
                                f'{floor:.2f} m 아래로는 안 내려감')
                    self.state = 'floor_limited'
            elif self.state == 'floor_limited':
                self.state = 'cruise'

        err = target - self.pose[2]
        if abs(err) > 0.08:
            out.linear.z = math.copysign(min(self.climb_rate, abs(err) * 1.5), err)
            # 고도를 바꾸는 중에는 수평 속도를 줄인다. 안 그러면 다 올라가기 전에 닿는다.
            scale = max(0.15, 1.0 - self.slow_gain * abs(err))
            out.linear.x *= scale
            out.linear.y *= scale
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LocalAltitudeAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
