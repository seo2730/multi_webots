"""마스터 관제 컨테이너용 런치.

맵 병합 노드 + RViz2를 함께 띄운다. 기존 master 서비스가 rviz2만 직접
실행하던 것을 이 런치로 대체한다.

  ros2 launch webots_map_merge master.launch.py
  ros2 launch webots_map_merge master.launch.py use_rviz:=false   # 헤드리스
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')

    pkg_dir = get_package_share_directory('webots_map_merge')
    default_params = os.path.join(pkg_dir, 'config', 'robots.yaml')

    # 관제용 RViz 설정: Fixed Frame = world, 전체 병합 맵 + 로봇 위치.
    # 기존 webots_python 쪽 설정은 ugv1 단독 뷰라 그대로 두고 새로 만들었다.
    rviz_config = LaunchConfiguration('rviz_config')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='RViz2를 함께 띄울지 여부')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'master_merged.rviz'),
        description='관제 RViz 설정 파일')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='맵 병합 파라미터 + 로봇 초기 위치 YAML')

    map_merger_node = Node(
        package='webots_map_merge',
        executable='map_merger',
        name='map_merger',
        output='screen',
        parameters=[params_file],
        # 병합 노드는 스스로 world -> {ns}/map static TF를 발행하므로
        # 전역 /tf_static을 그대로 써야 한다.
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    print('✅ 마스터 관제: 맵 병합(map_merger) + RViz2 세팅 완료')

    return LaunchDescription([
        declare_use_rviz,
        declare_rviz_config,
        declare_params_file,
        map_merger_node,
        rviz_node,
    ])
