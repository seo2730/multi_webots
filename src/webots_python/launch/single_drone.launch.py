"""드론 1대용 런치.

**2.5D 레이어드 내비게이션**(경로 1)을 돈다. Nav2 자체는 2D 그대로 두고, 고도라는
한 축만 바깥에서 담당해 "장애물을 넘어가는" 동작을 얻는 구조다.

  Velodyne(수평) ─┐
                  ├─▶ drone_layer_mapper ─┬─▶ /{ns}/map         층 합집합 ─▶ 맵 병합기
  down_depth(하향)┘                        ├─▶ /{ns}/map_active 현재 층   ─▶ Nav2
                                           └─▶ /{ns}/map_layer_k        ─▶ 고도 선택기
                                                                              │
                                        /{ns}/goal_pose_3d ─▶ 층 선택 ─▶ 고도 이동 ─▶ Nav2

되는 것: cmd_vel 조종, odom/TF, 카메라, 맵 병합 참여, 목표점 자율비행, 고도 회피.

왜 3D 플래너(경로 2)가 아닌가 — 계획 비용이 군집에서 대수만큼 곱해진다. 이 구조는
**계획을 지금과 똑같은 2D A* 로 유지**하고, 늘어나는 비용은 층 수만큼의 회랑 검사
(층당 수백 칸)뿐이다.

Nav2 파라미터(navigation/param/nav2.yaml)는 UGV 것을 공유한다. 지상 로봇 전제라 못 쓸
줄 알았는데 실측해 보니 그대로 동작했다 — 드론이 고도를 스스로 잡기 때문이다.
Nav2는 linear.x / angular.z 만 쓰고, 드라이버는 linear.z 가 0이면 현재 목표 고도를
유지한다. 그래서 2D 내비게이션이 정속 수평 비행으로 그대로 번역된다.
실측: 4 m 목표 3회 SUCCEEDED, 최종 오차 0.13~0.20 m, 고도는 2.00 m 고정 유지.

⚠️ **1 m 이내는 라이다가 못 본다**(minRange). 발밑은 하향 뎁스센서가 메우지만
   수평 근접은 여전히 사각이다 (08_DRONE_SETUP.md 6장).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

# 후보 순항 고도(m). 층 간격은 기체 지름(0.7 m)보다 넉넉해야 하고, 층 두께(±0.5 m)가
# 서로 겹치지 않아야 한다. 매퍼와 선택기가 **같은 값**을 봐야 하므로 여기서 한 번만 적는다.
LAYER_HEIGHTS = [1.0, 2.0, 3.0]

# ---------------------------------------------------------------------------
# 경로계획 모드 — NAV_MODE 환경 변수로 고른다 (compose 에서 로봇마다 다르게 줄 수 있다).
#
#   NAV_MODE=2d ros2 launch webots_python single_drone.launch.py
#
# 고도를 다루는 축이 둘이라 모드가 그 조합으로 정해진다.
#   ① 전역 층 선택 (altitude_selector) — 목표를 받을 때 순항 고도를 고른다
#   ② 지역 고도 회피 (local_altitude_avoider) — 주행 중 앞이 막히면 넘어간다
#
# 🚨 `3d`(연속 3D 경로계획)는 **구현돼 있지 않다.** 이름만 받아 주고 조용히 2.5d 처럼
#    돌면 나중에 "3d 로 돌렸는데 왜 고도가 계획 안 되지" 로 헤매게 되므로,
#    명시적으로 거부한다. 왜 안 만들었는지는 09_DRONE_NAV.md 3장에 있다.
NAV_MODES = {
    #            (전역 층 선택, 지역 회피)
    '2d':          (False, False),   # 고정 고도. Nav2 만 — 지상 로봇과 같은 동작
    '2.5d_local':  (False, True),    # 고도는 고정하되 앞이 막히면 넘어간다
    '2.5d':        (True,  True),    # 목표마다 층 선택 + 지역 회피 (기본)
}


def generate_launch_description():
    # 🌟 docker-compose에서 주입한 환경 변수 읽어오기 (기본값: drone1)
    #
    # 소환된 드론(webots_robot_spawner)도 같은 경로로 들어온다. 소환기가 자식 프로세스의
    # 환경 변수에 ROBOT_ID / ROBOT_INIT_* 를 넣어 주므로 이 런치 파일은 정적으로 뜬
    # drone1인지 런타임에 소환된 drone2인지 구분할 필요가 없다.
    #   손으로 띄워 볼 때:  ROBOT_ID=drone2 ros2 launch webots_python single_drone.launch.py
    ns = os.environ.get('ROBOT_ID', 'drone1')

    nav_mode = os.environ.get('NAV_MODE', '2.5d').strip().lower()
    if nav_mode == '3d':
        raise RuntimeError(
            "NAV_MODE=3d 는 아직 구현되지 않았습니다. 연속 3D 경로계획은 계획 호출마다 "
            "3D 탐색을 해서 군집에서 비용이 대수만큼 곱해지기 때문에 넣지 않았습니다 "
            "(근거: 09_DRONE_NAV.md 3장). 지금 쓸 수 있는 값: "
            f"{', '.join(NAV_MODES)}")
    if nav_mode not in NAV_MODES:
        raise RuntimeError(
            f"NAV_MODE='{nav_mode}' 를 모릅니다. 쓸 수 있는 값: {', '.join(NAV_MODES)}")
    use_layer_select, use_local_avoid = NAV_MODES[nav_mode]

    # Webots가 도는 호스트. 리눅스 네이티브 Docker에는 host.docker.internal이 기본으로
    # 없어 compose의 extra_hosts로 별칭을 만들지만, 그 방법을 못 쓰는 환경(원격 PC의
    # Webots에 붙는 경우 등)을 위해 환경 변수로도 바꿀 수 있게 열어 둔다.
    webots_host = os.environ.get('WEBOTS_HOST', 'host.docker.internal')
    webots_port = os.environ.get('WEBOTS_PORT', '1234')

    webots_pkg_dir = get_package_share_directory('webots_python')
    navigation_pkg_dir = get_package_share_directory('navigation')
    urdf_path = os.path.join(webots_pkg_dir, 'urdf', 'Mavic2ProMedium.urdf.xacro')
    # (slam_toolbox 는 drone_layer_mapper 로 대체됐다 — [C] 참고. UGV/Spot 은 계속 쓴다)

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
            ('/Velodyne_VLP_16/point_cloud', f'/{ns}/Velodyne_VLP_16/point_cloud'),
            ('/clock', '/clock'),
        ]
    )

    # ---------------------------------------------------------
    # [C] 층별 지도 매퍼  (pointcloud_to_laserscan + slam_toolbox 를 대체한다)
    #
    # 왜 SLAM 을 뺐나 — 드라이버가 GPS 절대좌표를 그대로 odom 으로 발행하므로 자세가
    # 이미 정답값이다. slam_toolbox 는 사실상 점유 격자 누적기로만 쓰이고 있었고,
    # 그 일은 이 노드가 훨씬 싸게 한다. 드론 1대당 노드가 2개 -> 1개로 줄고 무거운
    # 스캔매칭이 사라진다. 군집을 감당하려고 3D 플래너(경로 2)를 버린 것과 같은 이유다.
    #
    # 발행 토픽이 셋으로 갈리는 것이 핵심이다.
    #   /{ns}/map          층 합집합      -> 맵 병합기 (패턴이 그대로 걸려 병합기는 무수정)
    #   /{ns}/map_active   현재 순항 고도 -> 아래 [G] 의 Nav2
    #   /{ns}/map_layer_k  후보 층        -> [H] 의 고도 선택기
    # 합집합을 Nav2 에 주면 안 되는 이유는 drone_layer_mapper.py 모듈 주석에 있다.
    # ---------------------------------------------------------
    layer_mapper_node = Node(
        package="webots_python",
        executable="drone_layer_mapper",
        namespace=ns,
        output="screen",
        parameters=[{
            "namespace": ns,
            "use_sim_time": True,
            "layer_heights": LAYER_HEIGHTS,
            "layer_half_height": 0.5,
            "resolution": 0.1,
            "origin_x": -10.0, "origin_y": -8.0,
            "width": 200, "height": 160,
            "min_range": 1.05,      # 라이다 minRange 1 m 로 잘린 값을 버린다
            "max_range": 20.0,
            "cloud_stride": 4,      # 군집 대비 — 격자보다 촘촘한 점은 비용일 뿐이다
        }],
    )

    # ---------------------------------------------------------
    # [D] map -> odom 정적 변환
    #
    # 원래 slam_toolbox 가 발행하던 링크다. odom 이 이미 월드 절대좌표라 이 변환은
    # **항등**이고, 맵 병합이 world -> {ns}/map 을 항등으로 두는 것과 같은 근거다
    # (robots.yaml 의 odom_is_world_absolute). 보정할 드리프트가 없으니 정적으로 둔다.
    # ---------------------------------------------------------
    map_to_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=ns,
        arguments=["0", "0", "0", "0", "0", "0", f"{ns}/map", f"{ns}/odom"],
        parameters=[{"use_sim_time": True}],
        remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static")],
    )

    # ---------------------------------------------------------
    # [E] 마스터 맵 병합용 등록 (초기 위치 + 하트비트)
    # ---------------------------------------------------------
    registrar_node = Node(
        package='webots_map_merge',
        executable='robot_registrar',
        namespace=ns,
        parameters=[{'robot_id': ns, 'has_map': True, 'map_topic': f'/{ns}/map'}],
    )

    # ---------------------------------------------------------
    # [F] 웹 목표점 중계 (지도 클릭 -> goal_pose)
    # ---------------------------------------------------------
    web_goal_relay_node = Node(
        package='webots_goal_bridge',
        executable='web_goal_relay',
        namespace=ns,
        parameters=[{'namespace': ns}],
    )

    # ---------------------------------------------------------
    # [G] Nav2 — UGV와 같은 파라미터를 공유한다 (single_ugv.launch.py와 동형)
    #
    # 3초 지연도 UGV와 같은 이유다. {ns}/map 프레임이 생기기 전에 Nav2가 뜨면
    # 코스트맵이 TF를 못 찾아 기동에 실패한다.
    #
    # 🚨 map_topic 이 UGV와 다른 유일한 지점이다. 드론의 `/{ns}/map` 은 여러 고도의
    #    **합집합**이라 자기 플래너에 주면 지금 고도에서는 뚫려 있는 곳을 못 지나간다.
    #    고도를 바꿔 장애물을 넘으려고 만든 기능이 오히려 지금보다 나빠진다.
    #    그래서 Nav2 에는 현재 순항 고도 한 층(`map_active`)만 준다.
    # ---------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg_dir, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': 'true',
            'autostart': 'true',
            'map_topic': f'/{ns}/map_active',
            # Nav2 의 속도를 회피기로 돌린다 ([I] 참고)
            'cmd_vel_topic': f'/{ns}/cmd_vel_nav',
        }.items()
    )
    delayed_nav2_launch = TimerAction(period=3.0, actions=[nav2_launch])

    # ---------------------------------------------------------
    # [H] 고도 선택기 — 경로 1(2.5D 레이어드)의 판단부
    #
    # Nav2 는 2D 라 고도를 계획하지 않는다. 그 한 축만 바깥에서 담당한다.
    #   /{ns}/goal_pose_3d 로 목표를 주면
    #     -> 층마다 직선 회랑이 뚫렸는지 보고 **가장 낮은 뚫린 층**을 고른 뒤
    #     -> 그 고도로 이동하고 나서 Nav2 의 /{ns}/goal_pose 로 넘긴다
    # Nav2 를 직접 쓰고 싶으면 예전처럼 /{ns}/goal_pose 로 주면 된다 (고도 고정).
    # ---------------------------------------------------------
    altitude_selector_node = Node(
        package='webots_python',
        executable='altitude_selector',
        namespace=ns,
        output='screen',
        parameters=[{
            'namespace': ns,
            'use_sim_time': True,
            'layer_heights': LAYER_HEIGHTS,
            'corridor_half_width': 0.6,   # 기체 반경 0.35 m + 여유
            # 회랑 판정은 **비율**로 본다. 하드 실패로 두면 실측에서 108개 방향 중
            # 뚫린 곳이 0개였다 (지도상 오른쪽이 5 m 넘게 비어 있었는데도).
            # 실제 회피는 Nav2 코스트맵이 하고, 여기는 층을 고르는 선별기다.
            'max_occupied_ratio': 0.02,
            'max_unknown_ratio': 0.35,
            'climb_rate': 0.4,
            'altitude_tolerance': 0.25,
        }],
    )

    # ---------------------------------------------------------
    # [I] 지역 고도 회피 — Nav2 와 드라이버 사이에 끼는 노드
    #
    #   Nav2 ─▶ /{ns}/cmd_vel_nav ─▶ [회피기] ─▶ /{ns}/cmd_vel ─▶ 드라이버
    #
    # 수평(linear.x/y, angular.z)은 Nav2 것을 그대로 통과시키고 **linear.z 만** 채운다.
    # Nav2 가 linear.z 를 항상 0 으로 두기 때문에 가능한 배선이다.
    #
    # [H]의 고도 선택기가 목표 단위의 **전역** 판단이라면, 이쪽은 룩어헤드 3 m 안의
    # **지역** 반응이다. 앞이 막히면 넘어갈 층으로 올라갔다가 트이면 순항 고도로 돌아온다.
    # 계획을 하지 않으므로 군집에서도 비용이 늘지 않는다.
    # ---------------------------------------------------------
    local_avoider_node = Node(
        package='webots_python',
        executable='local_altitude_avoider',
        namespace=ns,
        output='screen',
        parameters=[{
            'namespace': ns,
            'use_sim_time': True,
            'layer_heights': LAYER_HEIGHTS,
            # 🌟 nav_mode 가 정한다. False 여도 이 노드는 계속 돈다 —
            #    cmd_vel 단독 발행과 발밑 안전 바닥은 모드와 무관한 기본 기능이다.
            'avoid_enabled': use_local_avoid,
            'cruise_altitude': 2.0,
            'lookahead': 3.0,
            'half_width': 0.7,      # 기체 반경 0.35 m + 여유
            # 부딪히는 높이 범위 = 기체 실제 크기 + 여유.
            # 뭉뚱그려 ±0.4 로 두면 **닿지도 않을 것을 피한다** (실측으로 겪었다).
            'block_above': 0.25,   # 라이다 윗면 +0.156 + 여유
            'block_below': 0.24,   # 랜딩기어 아랫면 -0.138 + 여유
            'clearance': 0.5,      # 장애물 윗면 위로 둘 여유 (상승량을 정한다)
            # 발밑 안전 바닥 — 하향 뎁스센서로 잰 표면 위로 반드시 남길 여유.
            # 이게 없으면 장애물을 넘은 직후 복귀하다가 그 위에 착지한다 (실측).
            'ground_clearance': 0.45,
            'foot_radius': 0.5,
            'min_range': 1.05,      # 라이다 minRange 로 잘린 값 제외
            'hit_threshold': 8,     # 점 몇 개부터 장애물로 볼 것인가
            'clear_hold': 15,       # 이만큼 연속으로 "내려가도 됨" 이어야 순항 복귀
            'move_threshold': 0.05, # 전진 명령이 이보다 작으면 회피를 시작하지 않는다
            'climb_rate': 0.5,
        }],
    )

    print(f"✅ 드론 [{ns}] : NAV_MODE={nav_mode} "
          f"(전역 층선택 {'ON' if use_layer_select else 'OFF'}, "
          f"지역 회피 {'ON' if use_local_avoid else 'OFF'}) | 층 {LAYER_HEIGHTS}")

    nodes = [
        rsp_node,
        webots_driver_node,
        layer_mapper_node,
        map_to_odom_node,
        registrar_node,
        web_goal_relay_node,
        delayed_nav2_launch,
        local_avoider_node,
    ]
    # 전역 층 선택은 모드가 요구할 때만 띄운다. 안 띄우면 /{ns}/goal_pose_3d 를 받는
    # 노드가 없으므로, 그 모드에서는 /{ns}/goal_pose 로 목표를 줘야 한다.
    if use_layer_select:
        nodes.append(altitude_selector_node)
    return LaunchDescription(nodes)
