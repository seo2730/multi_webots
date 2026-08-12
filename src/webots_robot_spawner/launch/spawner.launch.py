"""fleet 컨테이너용 런치 — 로봇 소환 노드를 띄운다.

  ros2 launch webots_robot_spawner spawner.launch.py
  ros2 launch webots_robot_spawner spawner.launch.py auto_launch_brain:=false

이 노드는 Webots extern 컨트롤러이기도 해서, 월드에 `spawn_supervisor`라는 이름의
Robot 노드가 있어야 붙는다 (my_world.wbt 맨 아래). Webots가 아직 안 떠 있으면
접속 대기 상태로 멈춰 있다가 뜨는 순간 붙는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('webots_robot_spawner')
    default_params = os.path.join(pkg_dir, 'config', 'spawner.yaml')

    params_file = LaunchConfiguration('params_file')
    auto_launch_brain = LaunchConfiguration('auto_launch_brain')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='소환 파라미터 YAML')

    declare_auto_brain = DeclareLaunchArgument(
        'auto_launch_brain', default_value='true',
        description='소환 후 ROS 2 뇌(driver/SLAM/Nav2)를 자동으로 띄울지')

    spawner_node = Node(
        package='webots_robot_spawner',
        executable='spawn_supervisor',
        name='spawn_supervisor',
        output='screen',
        parameters=[params_file, {'auto_launch_brain': auto_launch_brain}],
        # 소환한 로봇의 뇌가 전역 /tf를 그대로 쓰도록, 이 노드도 네임스페이스를 두지 않는다.
        additional_env={
            # 리눅스 네이티브 Docker에는 host.docker.internal이 없으므로
            # compose의 extra_hosts로 별칭을 만든다 (docker-configs/ubuntu 참고).
            'WEBOTS_HOST': os.environ.get('WEBOTS_HOST', 'host.docker.internal'),
            'WEBOTS_PORT': os.environ.get('WEBOTS_PORT', '1234'),
        },
    )

    print('✅ 로봇 소환기(spawn_supervisor) 세팅 완료 — 서비스: /spawn_robot')

    return LaunchDescription([
        declare_params_file,
        declare_auto_brain,
        spawner_node,
    ])
