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
        # strategy:=llm 일 때만 쓴다. OpenAI 호환이면 무엇이든 붙는다 —
        # NVIDIA NIM / 로컬 vLLM / Ollama. 키는 api_key_env 가 가리키는 환경변수에서 읽는다.
        ('base_url', 'https://integrate.api.nvidia.com/v1', 'OpenAI 호환 엔드포인트'),
        ('model', 'nvidia/nemotron-3-super-120b-a12b', '교사 모델 이름'),
        ('api_key_env', 'NVIDIA_API_KEY', 'API 키를 담은 환경변수 이름'),
        ('max_tokens', '2048', '응답 상한. 추론 모델은 예산을 추론에 먼저 쓰므로 넉넉히'),
        ('system_prompt', 'detailed thinking off',
         'Nemotron 추론 토글. 다른 제공자면 빈 문자열로'),
        ('llm_retries', '3', '스키마 위반 시 재시도 횟수. 다 실패하면 베이스라인으로 폴백'),
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
                'base_url': LaunchConfiguration('base_url'),
                'model': LaunchConfiguration('model'),
                'api_key_env': LaunchConfiguration('api_key_env'),
                'max_tokens': LaunchConfiguration('max_tokens'),
                'system_prompt': LaunchConfiguration('system_prompt'),
                'llm_retries': LaunchConfiguration('llm_retries'),
            }],
        )])
