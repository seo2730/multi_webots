import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.actions import TimerAction

def generate_launch_description():
    # 🌟 1. docker-compose에서 주입한 환경 변수 읽어오기 (기본값: ugv1)
    #
    # 소환된 UGV(webots_robot_spawner)도 같은 경로로 들어온다. 소환기가 자식 프로세스의
    # 환경 변수에 ROBOT_ID / ROBOT_INIT_* 를 넣어 주므로 이 런치 파일은 정적으로 뜬
    # ugv1인지 런타임에 소환된 ugv3인지 구분할 필요가 없다.
    #   손으로 띄워 볼 때:  ROBOT_ID=ugv3 ros2 launch webots_python single_ugv.launch.py
    ns = os.environ.get('ROBOT_ID', 'ugv1')

    # Webots가 도는 호스트. 리눅스 네이티브 Docker에는 host.docker.internal이 기본으로
    # 없어 compose의 extra_hosts로 별칭을 만들지만, 원격 PC의 Webots에 붙는 경우를
    # 위해 환경 변수로도 바꿀 수 있게 열어 둔다.
    webots_host = os.environ.get('WEBOTS_HOST', 'host.docker.internal')
    webots_port = os.environ.get('WEBOTS_PORT', '1234')

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
        additional_env={'WEBOTS_CONTROLLER_URL': f'tcp://{webots_host}:{webots_port}/{ns}'},
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'set_robot_state_publisher': False, # 🚨 필수
            # Webots 노드의 synchronization 필드와 값이 같아야 한다. 소환된 로봇은
            # 노드가 synchronization FALSE 로 들어가므로(뇌 접속 전까지 시뮬 전체가
            # 멈추는 것을 막기 위해) 소환기가 이 변수를 'false' 로 넣어 준다.
            'synchronization': os.environ.get('ROBOT_SYNCHRONIZATION', 'true').lower() != 'false',
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
    # [C] 스캔 변환 노드
    # ---------------------------------------------------------
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        namespace=ns,
        remappings=[
            ('cloud_in', f'/{ns}/Velodyne_VLP_16/point_cloud'), 
            ('scan', f'/{ns}/scan'),
            ('/tf', '/tf'), ('/tf_static', '/tf_static')
        ],
        parameters=[{
            'target_frame': f'{ns}/base_link', 
            'transform_tolerance': 5.0,
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

    # ---------------------------------------------------------
    # [D] SLAM Toolbox
    # ---------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace=ns,
        parameters=[
            slam_params_file,
            {
                'use_sim_time': True,
                'odom_frame': f'{ns}/odom',
                'base_frame': f'{ns}/base_link',
                'map_frame': f'{ns}/map',
                'scan_topic': f'/{ns}/scan',
                'transform_publish_period': 0.05, 
                'publish_map_to_odom_tf': True,
                'mode': 'mapping',
            }
        ],
        remappings=[
            ('/tf', '/tf'), ('/tf_static', '/tf_static'),
            ('/map', 'map'), ('/map_metadata', 'map_metadata'),
        ]
    )

    # ---------------------------------------------------------
    # [E] 웹 목표점 중계 (지도 클릭 -> goal_pose)
    # ---------------------------------------------------------
    web_goal_relay_node = Node(
        package='webots_goal_bridge',
        executable='web_goal_relay',
        namespace=ns,
        parameters=[{'namespace': ns}],
    )

    # ---------------------------------------------------------
    # [F] 마스터 맵 병합용 등록 (초기 위치 + 하트비트)
    #     초기 위치는 docker-compose의 ROBOT_INIT_X/Y/YAW 환경변수에서 읽는다.
    # ---------------------------------------------------------
    registrar_node = Node(
        package='webots_map_merge',
        executable='robot_registrar',
        namespace=ns,
        parameters=[{'robot_id': ns, 'has_map': True, 'map_topic': f'/{ns}/map'}],
    )

    # ---------------------------------------------------------
    # 🌟 [G] Navigation2 (Nav2) 추가!
    # ---------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_pkg_dir, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': 'true',
            'autostart': 'true',
            # param_file은 nav2.launch.py 내부의 RewrittenYaml이 알아서 처리합니다.
        }.items()
    )

    # 🌟 Nav2 실행을 3초 지연시킵니다!
    delayed_nav2_launch = TimerAction(
        period=3.0, # 3초 대기
        actions=[nav2_launch]
    )

    print(f"✅ 로봇 [{ns}] : Webots + SLAM + Nav2 세팅 완료")

    return LaunchDescription([
        rsp_node,
        webots_driver_node,
        pc_to_scan_node,
        slam_node,
        web_goal_relay_node,
        registrar_node,
        delayed_nav2_launch # 🌟 지연된 Nav2 사용
    ])