"""살아 있는 아무 로봇의 `odom` 시각을 전역 `/clock` 으로 중계한다.

왜 이 노드가 필요한가
---------------------
Webots 자체는 ROS 2 `/clock` 을 내지 않는다. 예전에는 UGV 드라이버 플러그인이
**네임스페이스가 `ugv1` 일 때만** 자기를 시계 마스터로 정해 발행했다.

    if self.namespace == 'ugv1' or self.namespace == '':   # robot_driver.py

그래서 편대에서 `ugv1` 을 빼면(Spot만, 드론만, `ugv2`만 …) `/clock` 발행자가 0개가 되고,
`use_sim_time: True` 인 노드 — SLAM · Nav2 · 맵 병합 — 가 **에러 없이 전부 멈춘다.**
로봇이 늘고 편대가 다양해질수록 이 하드코딩이 걸림돌이 됐다.

왜 소환기가 시계원이 아닌가
--------------------------
소환기(spawn_supervisor)는 Supervisor 라서 `getTime()` 을 직접 갖지만 **시계원으로는
못 쓴다.** 월드의 그 노드는 `synchronization FALSE` 라서 시뮬과 박자가 맞지 않는다
(TRUE 로 바꾸면 fleet 컨테이너가 없을 때 시뮬 전체가 멈춘다 — 그래서 FALSE 다).
게다가 소환기는 교착 시 `step()` 에 갇히는 바로 그 컴포넌트라, 시계가 가장 필요한
순간에 시계가 죽는다.

그래서 이 노드는 Webots 에 붙지 않는다. **이미 sim time 으로 스탬프된 `odom` 을 받아
중계만 한다.** 시뮬을 멈출 수단이 없고(순수 구독자), 특정 로봇에 묶이지도 않는다.

동작
----
1. 주기적으로 토픽 목록을 훑어 `*/odom` 을 **전부** 구독한다. 로봇이 늘거나 빠져도
   따라간다 — 소환으로 로봇이 런타임에 들어오는 구조라 한 번 훑고 끝내면 안 된다.
2. 들어온 스탬프 중 **가장 앞선 값**을 `/clock` 으로 낸다.
3. 🚨 **시각은 절대 뒤로 가지 않게 한다.** 로봇마다 스텝이 조금씩 어긋나는데 그대로
   흘리면 `/clock` 이 되돌아가고, rclcpp 가 "jump back in time" 으로 타이머를 리셋하며
   tf2 캐시가 무효화된다. 마지막으로 낸 값보다 이른 스탬프는 버린다.
"""

import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class OdomClockBridge(Node):

    def __init__(self):
        super().__init__('sim_clock_bridge')

        # 훑는 주기(초). 런타임 소환을 따라가야 하므로 계속 다시 본다.
        self.declare_parameter('scan_period', 2.0)
        # 비우면 모든 로봇을 따라간다. 특정 로봇만 쓰고 싶으면 이름을 준다.
        self.declare_parameter('robot_id', os.environ.get('CLOCK_ROBOT_ID', ''))

        self._only = str(self.get_parameter('robot_id').value).strip()
        self._subs = {}          # 토픽 이름 -> Subscription
        self._last_ns = -1       # 마지막으로 발행한 시각(ns). 역행 방지용

        # /clock 은 rclcpp 의 ClockQoS(=KeepLast(1), BEST_EFFORT)로 구독된다.
        # BEST_EFFORT 로 내야 그 구독자와 맞는다.
        self._pub = self.create_publisher(
            Clock, '/clock',
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT))

        self.create_timer(float(self.get_parameter('scan_period').value),
                          self._rescan)
        self._rescan()

        target = self._only or '모든 로봇'
        self.get_logger().info(f'시계 브릿지 시작 — {target} 의 odom 을 /clock 으로 중계')

    def _rescan(self):
        """`*/odom` 을 찾아 아직 구독하지 않은 것을 구독한다."""
        for name, types in self.get_topic_names_and_types():
            if 'nav_msgs/msg/Odometry' not in types:
                continue
            if not name.endswith('/odom') or name in self._subs:
                continue
            # /ugv1/odom -> ugv1
            ns = name[1:-len('/odom')]
            if self._only and ns != self._only:
                continue
            self._subs[name] = self.create_subscription(
                Odometry, name, self._on_odom, 10)
            self.get_logger().info(f'  + {name} 구독 (총 {len(self._subs)}개)')

    def _on_odom(self, msg):
        stamp = msg.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if ns <= self._last_ns:
            # 🚨 역행하거나 제자리인 스탬프는 버린다. 위 docstring 참고.
            return
        self._last_ns = ns
        out = Clock()
        out.clock = stamp
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomClockBridge()
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
