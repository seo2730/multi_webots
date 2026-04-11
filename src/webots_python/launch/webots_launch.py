import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

os.environ['USER'] = 'root'

def generate_launch_description():
    package_dir = get_package_share_directory('webots_python')
    
    # =========================================================
    # 1. UGV1 로봇 설정
    # =========================================================
    ugv1_urdf_path = os.path.join(package_dir, 'urdf', 'SummitXlSteel.urdf.xacro')
    ugv1_description = xacro.process_file(
        ugv1_urdf_path,
        mappings={'namespace': 'ugv1'}
    ).toxml()

    ugv1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='ugv1',  # 🌟 [수정] 노드 자체에 네임스페이스 적용
        parameters=[{
            'robot_description': ugv1_description,
            'use_sim_time': True,
            # 💡 xacro에서 수동으로 'ugv1/'을 안 붙였다면, 아래 옵션 주석을 풀어 자동 추가할 수 있습니다.
            # 'frame_prefix': 'ugv1/' 
        }]
    )

    webots_ugv1_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        # namespace='ugv1',  # 🌟 [수정] 드라이버 노드에도 네임스페이스 명시
        additional_env={'WEBOTS_CONTROLLER_URL': 'tcp://host.docker.internal:1234/ugv1'},
        parameters=[{
            'robot_description': ugv1_description,
            'use_sim_time': True,
            'set_robot_state_publisher': True,
        }]
    )

    # =========================================================
    # 2. Spot 로봇 설정
    # =========================================================
    spot_urdf_path = os.path.join(package_dir, 'urdf', 'Spot.urdf')
    
    # 🌟 [수정] 단순 경로가 아닌, URDF 파일 내용을 읽어와서 변수에 저장합니다.
    with open(spot_urdf_path, 'r') as infp:
        spot_description = infp.read()

    # 🌟 [추가] Spot 전용 로봇 상태 퍼블리셔 (TF 뼈대 발행)
    spot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='spot',  # 🌟 스팟 전용 네임스페이스
        parameters=[{
            'robot_description': spot_description,
            'use_sim_time': True,
            # 'frame_prefix': 'spot/' 
        }]
    )

    webots_spot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        namespace='spot',  # 🌟 [수정] 스팟 드라이버도 네임스페이스 분리
        additional_env={'WEBOTS_CONTROLLER_URL': 'tcp://host.docker.internal:1234/Spot'},
        parameters=[{
            'robot_description': spot_description, # 읽어온 URDF 내용 주입
            'use_sim_time': True,
            'set_robot_state_publisher': True,
        }]
    )

    return LaunchDescription([
        ugv1_state_publisher,
        webots_ugv1_driver,
        spot_state_publisher,
        webots_spot_driver
    ])