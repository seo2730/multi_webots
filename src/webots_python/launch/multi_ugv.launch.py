import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

os.environ['ROS_LOCALHOST_ONLY'] = '0'

def launch_setup(context, *args, **kwargs):
    # 1. 런치 인자 값 읽기 (로봇 개수)
    robot_count_str = LaunchConfiguration('count').perform(context)
    robot_count = int(robot_count_str)
    
    # 2. 패키지 경로 가져오기
    webots_pkg_dir = get_package_share_directory('webots_python')
    navigation_pkg_dir = get_package_share_directory('navigation') 
    
    # 설정 파일 경로
    urdf_path = os.path.join(webots_pkg_dir, 'urdf', 'SummitXlSteel.urdf.xacro')
    slam_params_file = os.path.join(webots_pkg_dir, 'config', 'mapper_params_online_async.yaml')
    
    actions = []

    # ========================================================================
    # 🌟 로봇 개수만큼 반복하며 [Webots + SLAM + Nav2] 노드 세트 생성
    # ========================================================================
    for i in range(1, robot_count + 1):
        ns = f'ugv{i}'  # 네임스페이스 생성 (ugv1, ugv2, ...)
        
        # ---------------------------------------------------------
        # [A] Webots 로봇 생성 및 뼈대 연결
        # ---------------------------------------------------------
        robot_description = xacro.process_file(urdf_path, mappings={'namespace': ns}).toxml()

        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        ))

        actions.append(Node(
            package='webots_ros2_driver',
            executable='driver',
            name=f'{ns}',
            output='screen',
            additional_env={'WEBOTS_CONTROLLER_URL': f'tcp://host.docker.internal:1234/{ns}'},
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'set_robot_state_publisher': True,
            }],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
                ('joint_states', f'/{ns}/joint_states') # Webots 이중 네임스페이스 방지
            ]
        ))

        # ---------------------------------------------------------
        # [B] 센서 데이터 변환 (PointCloud -> LaserScan)
        # ---------------------------------------------------------
        actions.append(Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc_to_scan',
            namespace=ns,
            output='screen',
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
                ('cloud_in', 'Velodyne_VLP_16/point_cloud'), 
                ('scan', f'/{ns}/scan'),                            
            ],
            parameters=[{
                'target_frame': f'{ns}/base_link', 
                'transform_tolerance': 1.0,
                'min_height': 0.1,  
                'max_height': 2.0,  
                'angle_min': -3.141592,  
                'angle_max': 3.141592,   
                'angle_increment': 0.0087, 
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 50.0,
                'use_inf': True,
                'use_sim_time': True,
            }]
        ))

        # ---------------------------------------------------------
        # [C] SLAM Toolbox 노드
        # ---------------------------------------------------------
        actions.append(Node(
            parameters=[
              slam_params_file, 
              {
                  'use_sim_time': 'true',
                  'odom_frame': f'{ns}/odom',
                  'base_frame': f'{ns}/base_link',
                  'map_frame': f'{ns}/map',
                  'scan_topic': f'/{ns}/scan', # 🌟 필수!

                  # 🌟 [핵심] 여기서 맵 발행 스위치를 직접 켭니다!
                  'transform_publish_period': 0.02, 
                  'publish_map_to_odom_tf': True,
                  'mode': 'mapping',
              },
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            namespace=ns,
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata'),
            ],
        ))

        # ---------------------------------------------------------
        # [D] Navigation2 (회원님의 nav2.launch.py 직접 사용!)
        # ---------------------------------------------------------
        # actions.append(IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(navigation_pkg_dir, 'launch', 'nav2.launch.py')),
        #     launch_arguments={
        #         'namespace': ns,
        #         'use_sim_time': 'true',
        #         'autostart': 'true',
        #         # 🌟 param_file을 넘기지 않습니다! nav2.launch.py 안에서 알아서 덮어씁니다.
        #     }.items()
        # ))

        # print(f"✅ 로봇 [{ns}] : Webots + SLAM + Nav2 세팅 완료")

    # ========================================================================
    # 🌟 RViz2 마스터 노드 (1개만 실행)
    # ========================================================================
    rviz_config_path = os.path.join(webots_pkg_dir, 'rviz', 'webots_rviz.rviz')
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    ))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'count',
            default_value='1',
            description='Number of robots to spawn (e.g., count:=3)'
        ),
        OpaqueFunction(function=launch_setup)
    ])