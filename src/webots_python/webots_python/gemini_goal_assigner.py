import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from PIL import Image
import os
import json

# ROS 2 메시지 타입
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# Gemini API 라이브러리 (pip install google-generativeai)
import google.generativeai as genai

class GeminiGoalAssigner(Node):
    def __init__(self):
        super().__init__('gemini_goal_assigner')
        
        # 1. 네임스페이스 설정 (예: 'ugv1')
        self.declare_parameter('namespace', 'ugv1')
        self.namespace = self.get_parameter('namespace').value
        
        # 2. Gemini API 설정
        self.api_key = os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE")
        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel('gemini-1.5-pro') # 비전 인식이 뛰어난 모델 사용
        
        # 3. 데이터 저장 변수
        self.current_map = None
        self.current_pose = None
        
        # 4. Subscribers (지도와 위치 정보)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            f'/{self.namespace}/map', 
            self.map_callback, 
            10)
            
        self.odom_sub = self.create_subscription(
            Odometry, 
            f'/{self.namespace}/odom', # TF를 써도 되지만 Odom이 구현하기 더 쉽습니다.
            self.odom_callback, 
            10)
            
        # 5. Action Client (Nav2 목표 전달용)
        self.nav_to_pose_client = ActionClient(
            self, 
            NavigateToPose, 
            f'/{self.namespace}/navigate_to_pose')

        # 6. 주기적으로 판단하는 타이머 (예: 30초마다 새로운 목표 요청)
        self.timer = self.create_timer(30.0, self.planning_cycle)
        self.get_logger().info("Gemini Goal Assigner Node Started.")

    def map_callback(self, msg):
        """맵 데이터를 받아옵니다."""
        self.current_map = msg

    def odom_callback(self, msg):
        """현재 위치를 받아옵니다."""
        self.current_pose = msg.pose.pose

    def convert_map_to_image(self, occupancy_grid):
        """
        ROS OccupancyGrid 데이터를 PIL Image로 변환합니다.
        (Gemini에게 지도를 시각적으로 보여주기 위함)
        -1: 알 수 없음 (회색), 0: 빈 공간 (흰색), 100: 장애물 (검은색)
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = np.array(occupancy_grid.data).reshape((height, width))
        
        # 이미지 픽셀 변환 로직 (예시)
        img_array = np.full((height, width), 128, dtype=np.uint8) # 회색(Unknown)
        img_array[data == 0] = 255   # 흰색(Free)
        img_array[data == 100] = 0   # 검은색(Obstacle)
        
        img = Image.fromarray(img_array)
        img_path = '/tmp/current_map.png'
        img.save(img_path)
        return img_path

    def request_goal_to_gemini(self, map_image_path, pose):
        """Gemini API에 맵 이미지와 현재 위치를 주고 다음 좌표를 얻어옵니다."""
        # 맵의 원점 좌표 등 메타데이터 추출
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        resolution = self.current_map.info.resolution
        
        prompt = f"""
        너는 자율주행 탐사 로봇의 최고 사령관이야.
        첨부된 이미지는 현재 로봇이 파악한 2D 지도(Grid Map)야. 
        - 흰색: 이동 가능 공간
        - 검은색: 장애물
        - 회색: 미탐색 영역
        
        현재 로봇의 위치(ROS 좌표계 기준)는 X: {pose.position.x:.2f}, Y: {pose.position.y:.2f}야.
        지도의 원점은 ({origin_x}, {origin_y})이고, 픽셀당 크기는 {resolution}m 야.
        
        이 지도를 분석해서 로봇이 다음에 탐색해야 할 가장 효율적인 미탐색 영역의 경계점(Frontier) 좌표 X, Y를 정해줘.
        응답은 반드시 아래 JSON 형식으로만 해줘.
        {{"target_x": 1.5, "target_y": 2.3}}
        """
        
        try:
            # 이미지 파일 로드 및 Gemini 호출
            sample_file = genai.upload_file(path=map_image_path)
            response = self.model.generate_content([prompt, sample_file])
            
            # 응답에서 JSON 파싱
            text_response = response.text.replace('```json', '').replace('```', '').strip()
            goal_data = json.loads(text_response)
            
            return goal_data['target_x'], goal_data['target_y']
            
        except Exception as e:
            self.get_logger().error(f"Gemini API 호출 에러: {e}")
            return None, None

    def send_goal(self, x, y):
        """Nav2 Action Server에 목표를 보냅니다."""
        self.get_logger().info(f"Nav2로 목표 전송 중... X: {x}, Y: {y}")
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 Action Server에 연결할 수 없습니다.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0 # 일단 방향은 기본값

        self.nav_to_pose_client.send_goal_async(goal_msg)
        self.get_logger().info('목표 전송 완료!')

    def planning_cycle(self):
        """주기적으로 실행되는 메인 로직"""
        if self.current_map is None or self.current_pose is None:
            self.get_logger().warn("아직 지도나 위치 데이터를 받지 못했습니다.")
            return
            
        self.get_logger().info("Gemini에게 다음 목표를 물어보는 중...")
        
        # 1. 맵을 이미지로 변환
        map_img_path = self.convert_map_to_image(self.current_map)
        
        # 2. Gemini에게 목표 좌표 받기
        target_x, target_y = self.request_goal_to_gemini(map_img_path, self.current_pose)
        
        # 3. 목표 전송
        if target_x is not None and target_y is not None:
            self.send_goal(target_x, target_y)

def main(args=None):
    rclpy.init(args=args)
    node = GeminiGoalAssigner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
