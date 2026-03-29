import os
import pathlib
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController

import xacro
import xml.etree.ElementTree

os.environ['USER'] = 'root'

def generate_launch_description():
    package_dir = get_package_share_directory('webots_python')
    ugv1_urdf_path = os.path.join(package_dir, 'urdf', 'SummitXlSteel.urdf.xacro')
    spot_urdf_path = os.path.join(package_dir, 'urdf', 'Spot.urdf')

    ugv1_description = xacro.process_file(
        ugv1_urdf_path,
        mappings={'namespace': 'ugv1'}
    ).toxml()

    # 기존 webots_controller = WebotsController(...) 부분을 완전히 지우고 아래로 대체합니다.
    webots_ugv1_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        # 🌟 여기가 핵심입니다. 윈도우 IP와 로봇 이름(ugv1)을 환경변수로 강제 주입합니다!
        additional_env={'WEBOTS_CONTROLLER_URL': 'tcp://host.docker.internal:1234/ugv1',
        },
        parameters=[
            {'robot_description': ugv1_description},
            {'use_sim_time': True},
            {'namespace': 'ugv1'}
        ]
    )

    webots_spot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        # 🌟 여기가 핵심입니다. 윈도우 IP와 로봇 이름(ugv1)을 환경변수로 강제 주입합니다!
        additional_env={'WEBOTS_CONTROLLER_URL': 'tcp://host.docker.internal:1234/Spot'},
        parameters=[
            {'robot_description': spot_urdf_path},
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
                              webots_ugv1_driver,
                              webots_spot_driver
                            ])