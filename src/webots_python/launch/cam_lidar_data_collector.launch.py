import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.actions import TimerAction

# 🌟 [추가됨] 조건부 실행을 위한 Launch 패키지 임포트
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # 🌟 1. docker-compose에서 주입한 환경 변수 읽어오기 (기본값: ugv1)
    ns = os.environ.get('ROBOT_ID', 'ugv1')

    # 🌟 [추가됨] 파라미터를 받을 변수 선언
    use_clock_bridge = LaunchConfiguration('use_clock_bridge')
    
    webots_pkg_dir = get_package_share_directory('webots_python')
    navigation_pkg_dir = get_package_share_directory('navigation')
    
    urdf_path = os.path.join(webots_pkg_dir, 'urdf', 'SummitXlSteel.urdf.xacro')
    slam_params_file = os.path.join(webots_pkg_dir, 'config', 'mapper_params_online_async.yaml')
    
    # URDF에 네임스페이스 주입
    robot_description = xacro.process_file(urdf_path, mappings={'namespace': ns}).toxml()

    # ---------------------------------------------------------
    # [A] 뼈대 방송국 (Robot State Publisher)
    # ---------------------------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        parameters=[{'robot_description': robot_description, 'use_sim_time': True, 'frame_prefix': f'{ns}/'}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )

    # ---------------------------------------------------------
    # [B] Webots 드라이버
    # ---------------------------------------------------------
    webots_driver_node = Node(
        package='webots_ros2_driver',
        executable='driver',
        name=f'{ns}',
        additional_env={'WEBOTS_CONTROLLER_URL': f'tcp://host.docker.internal:1234/{ns}'},
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'set_robot_state_publisher': False, # 🚨 필수
            'synchronization': True,
        }],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('joint_states', f'/{ns}/joint_states'),
            ('/Velodyne_VLP_16/point_cloud', f'/{ns}/Velodyne_VLP_16/point_cloud'),
            ('/clock', '/clock')
        ]
    )

    # ---------------------------------------------------------
    # [C] 시계 발전소 (조건부 실행)
    # ---------------------------------------------------------
    clock_bridge_node = Node(
        package='webots_python',
        executable='sim_clock_bridge',
        name=f'{ns}_sim_clock_bridge',
        remappings=[('/clock', '/clock')],
        # 🌟 [추가됨] use_clock_bridge 값이 'true'일 때만 이 노드를 실행합니다!
        condition=IfCondition(use_clock_bridge) 
    )

    data_collector_node = Node(
        package='webots_python',
        executable='cam_lidar_data_collector',
        name=f'{ns}_cam_lidar_data_collector',
        namespace=ns,
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/rgb_camera/image_color', f'/{ns}/rgb_camera/image_color'),
            ('/rgb_camera/recognitions', f'/{ns}/rgb_camera/recognitions'),
            ('/point_cloud', f'/{ns}/point_cloud'),
            ('/cmd_vel', f'/{ns}/cmd_vel'),
            ('/clock', '/clock')
        ]
    )

    print(f"✅ 로봇 [{ns}] : Webots + SLAM + Nav2 세팅 완료")

    return LaunchDescription([
        # 🌟 [추가됨] 터미널에서 입력받을 파라미터 선언 (기본값 설정 가능)
        DeclareLaunchArgument(
            'use_clock_bridge',
            default_value='false', # 우분투/실차 환경을 기본(false)으로 설정
            description='Set to "true" to run sim_clock_bridge (for Mac/Docker environments)'
        ),

        rsp_node,
        webots_driver_node,
        data_collector_node,
        clock_bridge_node
    ])