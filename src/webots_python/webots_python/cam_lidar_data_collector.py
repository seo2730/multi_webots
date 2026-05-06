import os
import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.duration import Duration
from cv_bridge import CvBridge
import message_filters

# ROS 2 메시지 타입
from sensor_msgs.msg import Image, PointCloud2
from webots_ros2_msgs.msg import CameraRecognitionObjects
from geometry_msgs.msg import PointStamped
import sensor_msgs_py.point_cloud2 as pc2

# TF2 (좌표 변환용)
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class SparseLIFDataCollector(Node):
    def __init__(self):
        super().__init__('sparselif_data_collector')
        self.bridge = CvBridge()
        
        # 환경변수로 로봇 ID 읽어오기
        self.robot_id = os.environ.get('ROBOT_ID', 'ugv1')
        self.get_logger().info(f"데이터 수집 노드 시작 - 대상: /{self.robot_id}")
        
        # TF2 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 🌟 안전한 절대 경로 설정 (도커 워크스페이스 기준)
        workspace_path = '/ros2_ws'
        package_name = 'webots_python'
        
        # 결과: /ros2_ws/src/webots_python/dataset_output
        self.base_path = os.path.join(workspace_path, 'src', package_name, 'dataset_output')
        
        # 저장 경로 생성
        for folder in ['image_2', 'velodyne', 'label_2']:
            os.makedirs(os.path.join(self.base_path, folder), exist_ok=True)
            
        # 경로가 잘 잡혔는지 터미널에서 확인하기 위한 로그
        self.get_logger().info(f"📁 데이터셋 저장 경로: {self.base_path}")
            
        self.frame_id = 0
        
        # 메시지 동기화 설정
        self.sub_img = message_filters.Subscriber(self, Image, f'/{self.robot_id}/rgb_camera/image_color')
        self.sub_rec = message_filters.Subscriber(self, CameraRecognitionObjects, f'/{self.robot_id}/rgb_camera/recognitions/webots')
        self.sub_lidar = message_filters.Subscriber(self, PointCloud2, f'/{self.robot_id}/Velodyne_VLP_16/point_cloud')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_rec, self.sub_lidar], 
            queue_size=100, slop=0.5
        )
        self.ts.registerCallback(self.sync_callback)

    def sync_callback(self, msg_img, msg_rec, msg_lidar):
        self.frame_id += 1
        frame_name = f"{self.frame_id:06d}"
        try:
            # 🌟 로봇의 네임스페이스가 붙은 odom을 글로벌 기준으로 사용합니다.
            target_frame = f'{self.robot_id}/odom'
            # 소스 프레임: Webots가 보낸 "rgb_camera" 앞에 "SummitXLSteel/"을 강제로 붙임!
            source_frame = f'{self.robot_id}/rgb_camera'
            # source_frame = f'{self.robot_id}/{msg_rec.header.frame_id}'
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                source_frame, 
                msg_rec.header.stamp,
                timeout=Duration(seconds=0.1) 
            )
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")
            return

        # 이미지, 라이다, 라벨 저장 (기존 로직과 동일)
        cv_img = self.bridge.imgmsg_to_cv2(msg_img, desired_encoding='bgr8')
        cv2.imwrite(os.path.join(self.base_path, 'image_2', f"{frame_name}.png"), cv_img)

        points = pc2.read_points(msg_lidar, field_names=("x", "y", "z"), skip_nans=True)
        
        # 🌟 ROS 2 Humble의 구조체 배열에서 x, y, z 열만 추출하여 하나로 병합 (column_stack)
        lidar_array = np.column_stack((points['x'], points['y'], points['z'])).astype(np.float32)
        
        lidar_array.tofile(os.path.join(self.base_path, 'velodyne', f"{frame_name}.bin"))

        with open(os.path.join(self.base_path, 'label_2', f"{frame_name}.txt"), 'w') as f:
            for obj in msg_rec.objects:
                rel_p = obj.pose.pose.position
                
                # ❌ 이전의 수동 축 변환 코드 3줄은 완전히 삭제합니다! (이중 회전 원인)
                # p_stamped.point.x = rel_p.z
                # p_stamped.point.y = -rel_p.x
                # p_stamped.point.z = -rel_p.y
                
                # ✅ 원본 데이터를 그대로 꽂아 넣습니다. (이제 TF가 알아서 해줍니다)
                p_stamped = PointStamped()
                p_stamped.point = rel_p 
                
                # 헤더 정보 주입
                p_stamped.header.frame_id = source_frame # (예: 'SummitXLSteel/rgb_camera')
                
                # 🌟 시간 에러 방지를 위해 방금 받은 메시지의 stamp를 사용하거나,
                # 만약 또 Extrapolation 에러가 나면 p_stamped.header.stamp = Time() 으로 변경
                p_stamped.header.stamp = msg_rec.header.stamp 
                
                # 절대 좌표 변환 실행
                abs_p = do_transform_point(p_stamped, transform).point
                
                x_min = obj.bbox.center.position.x - (obj.bbox.size_x / 2)
                y_min = obj.bbox.center.position.y - (obj.bbox.size_y / 2)
                x_max = obj.bbox.center.position.x + (obj.bbox.size_x / 2)
                y_max = obj.bbox.center.position.y + (obj.bbox.size_y / 2)

                line = (f"{obj.model} {x_min:.1f} {y_min:.1f} {x_max:.1f} {y_max:.1f} "
                        f"{abs_p.x:.3f} {abs_p.y:.3f} {abs_p.z:.3f} "
                        f"{rel_p.x:.3f} {rel_p.y:.3f} {rel_p.z:.3f}\n")
                f.write(line)

        self.get_logger().info(f"Frame {frame_name} 저장 완료")

def main(args=None):
    rclpy.init(args=args)
    node = SparseLIFDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()