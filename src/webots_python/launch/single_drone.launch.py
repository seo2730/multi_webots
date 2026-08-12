"""드론 1대용 런치.

현재 단계에서는 [robot_state_publisher + Webots 드라이버]만 띄운다.
드론에 거리 측정 센서가 아직 없어서 SLAM/Nav2를 붙일 수 없기 때문이다
(단계 3에서 뎁스카메라/거리센서를 달고 나면 single_ugv.launch.py처럼 확장한다).

이 상태에서 되는 것: cmd_vel 조종, odom/TF, 카메라 영상.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # 🌟 docker-compose에서 주입한 환경 변수 읽어오기 (기본값: drone1)
    #
    # 소환된 드론(webots_robot_spawner)도 같은 경로로 들어온다. 소환기가 자식 프로세스의
    # 환경 변수에 ROBOT_ID / ROBOT_INIT_* 를 넣어 주므로 이 런치 파일은 정적으로 뜬
    # drone1인지 런타임에 소환된 drone2인지 구분할 필요가 없다.
    #   손으로 띄워 볼 때:  ROBOT_ID=drone2 ros2 launch webots_python single_drone.launch.py
    ns = os.environ.get('ROBOT_ID', 'drone1')

    # Webots가 도는 호스트. 리눅스 네이티브 Docker에는 host.docker.internal이 기본으로
    # 없어 compose의 extra_hosts로 별칭을 만들지만, 그 방법을 못 쓰는 환경(원격 PC의
    # Webots에 붙는 경우 등)을 위해 환경 변수로도 바꿀 수 있게 열어 둔다.
    webots_host = os.environ.get('WEBOTS_HOST', 'host.docker.internal')
    webots_port = os.environ.get('WEBOTS_PORT', '1234')

    webots_pkg_dir = get_package_share_directory('webots_python')
    urdf_path = os.path.join(webots_pkg_dir, 'urdf', 'Mavic2ProMedium.urdf.xacro')

    robot_description = xacro.process_file(urdf_path, mappings={'namespace': ns}).toxml()

    # ---------------------------------------------------------
    # [A] 뼈대 방송국 (Robot State Publisher)
    # ---------------------------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': f'{ns}/',
        }],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )

    # ---------------------------------------------------------
    # [B] Webots 드라이버
    # ---------------------------------------------------------
    webots_driver_node = Node(
        package='webots_ros2_driver',
        executable='driver',
        name=f'{ns}',
        additional_env={'WEBOTS_CONTROLLER_URL': f'tcp://{webots_host}:{webots_port}/{ns}'},
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'set_robot_state_publisher': False,  # 🚨 필수
            # 자세 루프가 매 스텝 돌아야 하므로 기본은 동기화(True)다.
            #
            # 다만 **Webots 노드의 synchronization 필드와 값이 같아야 한다.**
            # 소환된 로봇(webots_robot_spawner)은 노드가 synchronization FALSE로 들어간다.
            # 뇌가 붙기 전까지 시뮬레이션 전체가 멈춰버리는 것을 막기 위해서다
            # (robot_types.spawn_string 주석 참고). 그래서 소환기는 이 환경 변수를
            # 'false'로 넣어 양쪽을 맞춘다. 정적으로 뜨는 drone1은 기본값 그대로 True.
            'synchronization': os.environ.get('ROBOT_SYNCHRONIZATION', 'true').lower() != 'false',
        }],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('joint_states', f'/{ns}/joint_states'),
            ('/clock', '/clock'),
        ]
    )

    # ---------------------------------------------------------
    # [C] 마스터 맵 병합용 등록 (하트비트)
    #     드론은 아직 SLAM이 없어 맵을 못 만든다. has_map=False로 알려서
    #     마스터가 존재만 인지하고 맵 구독은 시도하지 않게 한다.
    #     (센서를 달고 SLAM을 붙이면 True로 바꾸면 끝)
    # ---------------------------------------------------------
    registrar_node = Node(
        package='webots_map_merge',
        executable='robot_registrar',
        namespace=ns,
        parameters=[{'robot_id': ns, 'has_map': False}],
    )

    print(f"✅ 드론 [{ns}] : Webots 드라이버 세팅 완료 (cmd_vel / odom / camera)")

    return LaunchDescription([
        rsp_node,
        webots_driver_node,
        registrar_node,
    ])
