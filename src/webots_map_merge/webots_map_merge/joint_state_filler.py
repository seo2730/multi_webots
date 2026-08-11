"""아무도 발행하지 않는 관절의 상태를 0으로 채워주는 마스터 측 보조 노드.

문제
----
robot_state_publisher 는 `joint_states` 를 받아야 움직이는 관절의 TF 를 만든다.
관절 상태가 안 오면 그 관절 아래 링크 전체의 TF 가 아예 안 생기고,
RViz 의 RobotModel 은 링크 하나라도 TF 가 없으면 빨간 에러 상태가 된다.

이 프로젝트에서 실측된 누락:
  - ugv1/ugv2 : 바퀴 관절 4개. Webots 드라이버(robot_driver.py)가 바퀴 모터를
                구동만 하고 위치를 발행하지 않는다.
  - spot1     : 팔/그리퍼 관절 9개. 월드의 Spot 에 팔이 안 달려 있는데
                URDF 에는 팔이 들어 있다.
  - drone1    : 움직이는 관절이 없어 문제 없음.

동작
----
`/{ns}/robot_description` 을 찾아 URDF 의 움직이는 관절 목록을 뽑고,
`/{ns}/joint_states` 를 지켜보다가 **한 번도 안 나타난 관절만** 0으로 발행한다.
드라이버가 발행하는 관절은 절대 건드리지 않으므로 값이 서로 덮어써지지 않는다.
(robot_state_publisher 는 부분 JointState 를 받아 내부에서 합친다.)

한계
----
0 으로 채우는 것이라 UGV 바퀴는 화면에서 돌지 않고, Spot 팔은 접힌 자세로 그려진다.
TF 트리를 온전하게 만드는 것이 목적이고, 이 값들을 쓰는 소비자는 현재 없다
(오도메트리는 GPS 기반이라 바퀴 각도와 무관).
"""

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# rsp 가 TF 를 만들려면 상태가 필요한 관절 타입
MOVABLE = {'revolute', 'continuous', 'prismatic'}


def description_qos() -> QoSProfile:
    """robot_state_publisher 가 robot_description 을 내보내는 QoS."""
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


@dataclass
class RobotJoints:
    ns: str
    movable: set = field(default_factory=set)
    seen: set = field(default_factory=set)
    pub: object = field(default=None, repr=False)
    # 채울 관절은 정착 시간이 지난 뒤 한 번만 정하고 고정한다.
    # 우리도 같은 토픽에 발행하므로, 계속 다시 판단하면 자기가 쏜 메시지를
    # "이미 누가 발행 중"으로 오인해서 발행을 멈춰버린다.
    missing: list = field(default_factory=list)
    frozen: bool = False


class JointStateFiller(Node):

    def __init__(self):
        super().__init__('joint_state_filler')

        self.discovery_rate = float(self._param('discovery_rate', 0.5))
        self.publish_rate = float(self._param('publish_rate', 2.0))
        # 드라이버가 먼저 관절을 발행할 시간을 준 뒤에 판단한다.
        # 너무 짧으면 정상 관절까지 "누락"으로 오해해서 0 을 덮어쓸 수 있다.
        self.settle_sec = float(self._param('settle_time', 8.0))
        self.pattern = re.compile(
            self._param('description_topic_pattern', r'^/([^/]+)/robot_description$'))

        self.robots: dict[str, RobotJoints] = {}
        self.start_ns = None

        self.create_timer(1.0 / max(self.discovery_rate, 0.01), self._discover)
        self.create_timer(1.0 / max(self.publish_rate, 0.01), self._fill)

        self.get_logger().info(
            f'관절 채우기 시작 | 정착 대기 {self.settle_sec}초 | 발행 {self.publish_rate}Hz')

    def _param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    def _discover(self):
        for topic, types in self.get_topic_names_and_types():
            if 'std_msgs/msg/String' not in types:
                continue
            match = self.pattern.fullmatch(topic)
            if not match:
                continue
            ns = match.group(1)
            if ns in self.robots:
                continue

            self.robots[ns] = RobotJoints(ns=ns)
            self.create_subscription(
                String, topic,
                lambda m, n=ns: self._on_description(n, m), description_qos())
            self.create_subscription(
                JointState, f'/{ns}/joint_states',
                lambda m, n=ns: self._on_joint_states(n, m), 10)
            self.robots[ns].pub = self.create_publisher(
                JointState, f'/{ns}/joint_states', 10)
            self.get_logger().info(f'[감시] {ns} 관절 상태 확인 시작')

    def _on_joint_states(self, ns: str, msg: JointState):
        robot = self.robots.get(ns)
        # 판단이 끝난 뒤에는 무시한다. 여기서 계속 받으면 우리가 채워 넣은
        # 관절까지 "이미 발행 중"으로 잡혀 발행이 끊긴다.
        if robot is not None and not robot.frozen:
            robot.seen.update(msg.name)

    def _on_description(self, ns: str, msg: String):
        robot = self.robots.get(ns)
        if robot is None or robot.movable:
            return
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as e:
            self.get_logger().warn(f'[{ns}] URDF 파싱 실패: {e}')
            return
        robot.movable = {
            j.get('name') for j in root.iter('joint')
            if j.get('type') in MOVABLE and j.get('name')
        }

    # ------------------------------------------------------------------
    def _fill(self):
        now = self.get_clock().now()
        if self.start_ns is None:
            self.start_ns = now.nanoseconds
            return
        # 시뮬 시간 기준. Webots 가 멈춰 있으면 판단도 미룬다.
        if (now.nanoseconds - self.start_ns) < self.settle_sec * 1e9:
            return

        for robot in self.robots.values():
            if not robot.movable:
                continue

            if not robot.frozen:
                robot.missing = sorted(robot.movable - robot.seen)
                robot.frozen = True
                if robot.missing:
                    self.get_logger().warn(
                        f'[{robot.ns}] 아무도 발행하지 않는 관절 {len(robot.missing)}개를 '
                        f'0으로 채움: {", ".join(robot.missing[:6])}'
                        f'{" ..." if len(robot.missing) > 6 else ""}')
                else:
                    self.get_logger().info(f'[{robot.ns}] 모든 관절이 정상 발행 중, 채울 것 없음')

            if not robot.missing:
                continue

            msg = JointState()
            msg.header.stamp = now.to_msg()
            msg.name = robot.missing
            msg.position = [0.0] * len(robot.missing)
            robot.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateFiller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
