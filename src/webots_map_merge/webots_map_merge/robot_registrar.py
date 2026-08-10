"""로봇 컨테이너가 마스터에게 자기 명함을 알리는 노드.

마스터의 토픽 자동 탐색만으로는 "누가 있는지"는 알아도
"그 로봇이 월드 어디서 출발했는지"를 알 수 없다. 초기 위치를 모르면
맵을 겹칠 기준이 없으므로, 각 로봇이 스스로 알려주게 한다.

주기적으로(기본 1Hz) 같은 내용을 다시 보내는 것이 하트비트 역할도 겸한다.
마스터는 이 신호가 끊기면 해당 로봇을 병합 대상에서 뺀다.

초기 위치는 docker-compose의 ROBOT_INIT_X / ROBOT_INIT_Y / ROBOT_INIT_YAW
환경 변수로 주입하는 것을 기본으로 한다 (ROBOT_ID를 쓰던 방식 그대로).
"""

import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


class RobotRegistrar(Node):

    def __init__(self):
        super().__init__('robot_registrar')

        default_ns = os.environ.get('ROBOT_ID', 'ugv1')
        self.declare_parameter('robot_id', default_ns)
        self.declare_parameter('init_x', env_float('ROBOT_INIT_X', 0.0))
        self.declare_parameter('init_y', env_float('ROBOT_INIT_Y', 0.0))
        self.declare_parameter('init_yaw', env_float('ROBOT_INIT_YAW', 0.0))
        self.declare_parameter('has_map', True)
        self.declare_parameter('map_topic', '')
        self.declare_parameter('registry_topic', '/robot_registry')
        self.declare_parameter('publish_rate', 1.0)

        self.robot_id = self.get_parameter('robot_id').value
        map_topic = self.get_parameter('map_topic').value or f'/{self.robot_id}/map'
        rate = float(self.get_parameter('publish_rate').value)

        self.payload = {
            'robot_id': self.robot_id,
            'init_x': float(self.get_parameter('init_x').value),
            'init_y': float(self.get_parameter('init_y').value),
            'init_yaw': float(self.get_parameter('init_yaw').value),
            'has_map': bool(self.get_parameter('has_map').value),
            'map_topic': map_topic,
        }

        # 마스터가 늦게 떠도 즉시 받아볼 수 있도록 TRANSIENT_LOCAL.
        qos = QoSProfile(
            depth=20,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(
            String, self.get_parameter('registry_topic').value, qos)

        self.publish_once()
        self.create_timer(1.0 / max(rate, 0.01), self.publish_once)

        self.get_logger().info(
            f"[{self.robot_id}] 마스터에 등록: 초기위치=("
            f"{self.payload['init_x']:.3f}, {self.payload['init_y']:.3f}, "
            f"yaw {self.payload['init_yaw']:.3f}) "
            f"맵={'있음' if self.payload['has_map'] else '없음'}")

    def publish_once(self):
        msg = String()
        msg.data = json.dumps(self.payload)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotRegistrar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
