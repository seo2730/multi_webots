import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

class FixTimestamp(Node):
    def __init__(self):
        super().__init__('fix_timestamp')
        
        qos_in = QoSProfile(
            depth=10, 
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        qos_out = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,  # slam_toolbox용
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.sub = self.create_subscription(
            PointCloud2,
            '/ugv1/Velodyne_VLP_16/point_cloud',
            self.callback,
            qos_in)
        self.pub = self.create_publisher(
            PointCloud2,
            '/ugv1/Velodyne_VLP_16/point_cloud_fixed',
            qos_out)

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(FixTimestamp())

if __name__ == '__main__':
    main()