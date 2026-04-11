import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # 파라미터 선언
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='ugv1', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo/Webots) clock')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to start RViz2')

    params_dir = get_package_share_directory('webots_python')
    params_dir_file = os.path.join(params_dir, 'config', 'mapper_params_online_async.yaml')

    # ========================================================================
    # 2. PointCloud to LaserScan 변환 노드
    # ========================================================================
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_scan',
        namespace=namespace,
        output='screen',
        remappings=[
            ('cloud_in', 'Velodyne_VLP_16/point_cloud'), 
            ('scan', 'scan')                             
        ],
        parameters=[{
            # 🌟 [수정 핵심 1] 리스트를 사용하여 런타임에 "ugv1/base_link"로 합성되게 만듭니다!
            'target_frame': [namespace, '/base_link'], 
            'transform_tolerance': 0.01,
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
    )

    # ========================================================================
    # 3. SLAM Toolbox 노드
    # ========================================================================
    start_async_slam_toolbox_node = Node(
        parameters=[
          params_dir_file, 
          {
              'use_sim_time': True,
              # 🌟 [수정 핵심 2] yaml 파일의 설정을 무시하고 동적 프레임 이름으로 강제 덮어쓰기!
              'odom_frame': [namespace, '/odom'],
              'base_frame': [namespace, '/base_link'],
              'map_frame': [namespace, '/map'],
          },
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ],
    )

    # ========================================================================
    # 4. RViz2 노드
    # ========================================================================
    pkg_share = get_package_share_directory('webots_python')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'webots_rviz.rviz')

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 런치 디스크립션에 노드들 추가
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # 노드 실행 순서대로 추가
    ld.add_action(pc_to_scan_node)              
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(rviz_node)                    

    return ld