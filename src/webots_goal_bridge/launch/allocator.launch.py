"""다중 로봇 프론티어 할당기 — 전용 컨테이너용 런치.

master 가 아니라 **독립 컨테이너**에 두는 이유
------------------------------------------------
master 는 맵 병합 · RViz · 시계라는 관제 기반을 담당한다. 탐사 할당은 그 위에 얹히는
**정책**이고, 실험 중에 자주 바꿔 끼운다(거리 기준 ↔ LLM). 같은 컨테이너에 두면
전략 하나 바꾸려고 RViz 와 맵 병합까지 재시작해야 하고, 할당기가 죽으면 관제까지
같이 흔들린다. 그래서 컨테이너 경계로 나눈다.

입력은 전부 전역 토픽이라 어디서 돌든 동작이 같다 — 위치는 순전히 운영 편의다.

    /map_merged      전역 지도 (master 의 map_merger 가 발행)
    /robot_registry  편대 (로봇들이 1 Hz 로 자기 이름을 알린다)
    /{ns}/odom, /{ns}/map
        ↓
    /{ns}/goal_pose

🚨 use_sim_time 은 필수다. 없으면 목표 스탬프가 벽시계라 Nav2 가 수백 초 과거로
   TF 를 조회한다. 또한 시계가 안 돌면 타이머 자체가 안 뛰므로 주기 처리가 멈춘다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        ('strategy', 'distance', '할당 전략. distance | llm — LLM 이 대체할 자리'),
        ('period', '30.0', '배정 주기(초)'),
        ('min_goal_dist', '4.0', '이보다 가까운 후보는 제외 (제자리 깨작임 방지)'),
        ('min_separation', '15.0', '배정된 후보끼리 최소 간격. 못 맞추면 단계적으로 낮춘다'),
        ('giveup_rounds', '3', '이만큼 진전이 없으면 목표를 버린다'),
        ('dataset_path', '', '증류용 학습 데이터 JSONL 경로. 비우면 안 쓴다'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc)
         for n, d, desc in args]
        + [Node(
            package='webots_goal_bridge',
            executable='frontier_allocator',
            name='frontier_allocator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'strategy': LaunchConfiguration('strategy'),
                'period': LaunchConfiguration('period'),
                'min_goal_dist': LaunchConfiguration('min_goal_dist'),
                'min_separation': LaunchConfiguration('min_separation'),
                'giveup_rounds': LaunchConfiguration('giveup_rounds'),
                'dataset_path': LaunchConfiguration('dataset_path'),
            }],
        )])
