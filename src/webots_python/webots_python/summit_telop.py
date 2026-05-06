import os
import sys
import select
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
로봇 제어 모드 (Summit-XL Steel 동특성 적용)
---------------------------
w/s : 전진/후진 가속
a/d : 좌/우 회전 가속
space : 급정거
CTRL-C : 조종 종료
"""

class SummitTeleop(Node):
    def __init__(self):
        super().__init__('summit_teleop')
        self.robot_id = os.environ.get('ROBOT_ID', 'ugv1')
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        
        self.MAX_LIN_VEL = 1.0
        self.MAX_ANG_VEL = 1.0
        self.LIN_ACCEL_STEP = 0.05
        self.ANG_ACCEL_STEP = 0.1
        
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        self.create_timer(0.1, self.publish_smoothed_velocity)
        print(msg)

    def make_simple_profile(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input
        return output

    def publish_smoothed_velocity(self):
        self.current_linear = self.make_simple_profile(self.current_linear, self.target_linear, self.LIN_ACCEL_STEP)
        self.current_angular = self.make_simple_profile(self.current_angular, self.target_angular, self.ANG_ACCEL_STEP)
        
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_pub.publish(twist)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def keyboard_thread(node, settings):
    while rclpy.ok():
        key = get_key(settings)
        if key == 'w':
            node.target_linear = min(node.target_linear + node.LIN_ACCEL_STEP * 2, node.MAX_LIN_VEL)
        elif key == 's':
            node.target_linear = max(node.target_linear - node.LIN_ACCEL_STEP * 2, -node.MAX_LIN_VEL)
        elif key == 'a':
            node.target_angular = min(node.target_angular + node.ANG_ACCEL_STEP * 2, node.MAX_ANG_VEL)
        elif key == 'd':
            node.target_angular = max(node.target_angular - node.ANG_ACCEL_STEP * 2, -node.MAX_ANG_VEL)
        elif key == ' ' or key == 'x':
            node.target_linear = 0.0
            node.target_angular = 0.0
        elif key == '\x03': # CTRL-C
            break

def main(args=None):
    if not sys.stdin.isatty():
        print("에러: 대화형 터미널(TTY)이 아닙니다. docker exec -it 로 접속해서 실행해주세요.")
        return

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = SummitTeleop()
    
    thread = threading.Thread(target=keyboard_thread, args=(node, settings), daemon=True)
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()