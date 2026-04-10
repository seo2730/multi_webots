import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 파라미터 선언
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='ugv1', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo/Webots) clock')

    params_dir = get_package_share_directory('webots_python')
    params_dir_file = os.path.join(params_dir, 'config', 'mapper_params_online_async.yaml')

    # ========================================================================
    # 1. PointCloud to LaserScan 변환 노드 추가
    # 3D 라이다 데이터를 받아서 2D 스캔 데이터로 변환해 줍니다.
    # ========================================================================
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_scan',
        namespace=namespace,
        output='screen',
        remappings=[
            ('cloud_in', 'Velodyne_VLP_16/point_cloud'), # 입력: /ugv1/Velodyne_VLP_16/point_cloud
            ('scan', 'scan')                             # 출력: /ugv1/scan
        ],
        parameters=[{
            'target_frame': [namespace, '/base_link'],   # 2D로 누를 때 기준이 되는 로봇 뼈대 (ugv1/base_link)
            'transform_tolerance': 0.01,
            'min_height': 0.1,  # base_link 기준 0.1m 이상 
            'max_height': 2.0,  # 2.0m 이하의 포인트들만 2D로 압축
            'angle_min': -3.141592,  # -180도
            'angle_max': 3.141592,   # 180도
            'angle_increment': 0.0087, # 해상도 (약 0.5도)
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 50.0,
            'use_inf': True,
            'use_sim_time': use_sim_time
        }]
    )

    # 2. SLAM Toolbox 노드
    start_async_slam_toolbox_node = Node(
        parameters=[
          params_dir_file, 
          {'use_sim_time': use_sim_time},
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ]
    )

    # 3. RViz2 노드 추가
    pkg_share = get_package_share_directory('webots_python')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'webots_rviz.rviz')

    # 2. RViz2 노드 설정
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments 부분에 '-d'와 경로를 리스트로 추가합니다.
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 런치 디스크립션에 노드들 추가
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # 노드 실행 순서대로 추가
    ld.add_action(pc_to_scan_node)              # [추가됨] 3D -> 2D 변환 노드 
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(rviz_node)                    # RViz2 실행

    return ld