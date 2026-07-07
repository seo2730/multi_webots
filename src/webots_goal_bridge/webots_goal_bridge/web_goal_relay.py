import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped


class WebGoalRelay(Node):
    """웹 클라이언트가 지도 클릭으로 보낸 목표점(PointStamped)을 자기 로봇의 goal_pose로 중계한다.

    frame_id가 '{namespace}/map'인 목표점만 처리하고 나머지는 무시한다.
    """

    def __init__(self):
        super().__init__('web_goal_relay')

        self.declare_parameter('namespace', 'ugv1')
        self.declare_parameter('web_goal_topic', '/web/goal_point')

        self.namespace = self.get_parameter('namespace').value
        web_goal_topic = self.get_parameter('web_goal_topic').value
        self.map_frame = f'{self.namespace}/map'

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.web_goal_sub = self.create_subscription(
            PointStamped, web_goal_topic, self.on_web_goal, 10)

        self.get_logger().info(
            f"[{self.namespace}] '{web_goal_topic}' 구독 시작 "
            f"(frame_id == '{self.map_frame}'인 목표점만 처리)")

    def on_web_goal(self, msg: PointStamped):
        if msg.header.frame_id != self.map_frame:
            return

        goal = PoseStamped()
        goal.header.frame_id = msg.header.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info(
            f"[{self.namespace}] 목표점 수신 -> goal_pose 전달: "
            f"x={msg.point.x:.2f}, y={msg.point.y:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = WebGoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
