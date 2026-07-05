import os
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OdomClockBridge(Node):
    def __init__(self):
        super().__init__('odom_clock_bridge')
        
        # 환경변수로 로봇 ID 읽어오기 (없으면 기본값)
        self.robot_id = os.environ.get('ROBOT_ID', 'SummitXLSteel')

        # 🌟 1. 전역(Global) /clock 퍼블리셔 생성
        # 시간 토픽은 끊기면 안 되므로 QoS를 약간 여유 있게 설정합니다.
        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.clock_pub = self.create_publisher(Clock, '/clock', clock_qos)

        # 🌟 2. 로봇의 odom 구독 (여기서 진짜 시뮬레이션 시간을 빼옵니다)
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f"🚀 [{self.robot_id}] Odom 시간 ➔ 전역 /clock 브릿지 가동 시작!")

    def odom_callback(self, msg):
        # Odom 메시지의 이마(header.stamp)에 적힌 진짜 Sim Time을 복사
        odom_time = msg.header.stamp

        # Clock 메시지 껍데기를 만들고 시간을 채워 넣음
        clock_msg = Clock()
        clock_msg.clock = odom_time

        # 전역 /clock 토픽으로 발사!
        self.clock_pub.publish(clock_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomClockBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()