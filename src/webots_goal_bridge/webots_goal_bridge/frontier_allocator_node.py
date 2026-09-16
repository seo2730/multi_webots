"""전역 지도에서 프론티어를 뽑아 **어느 로봇이 어디로 갈지** 배정한다 (explorer 컨테이너).

왜 중앙집중인가, 왜 별도 컨테이너인가
------------------------------------
다중 로봇 할당은 원리적으로 중앙집중이다. 다른 로봇이 어디로 가는지 모르면 겹침을
막을 수 없다. 그래서 로봇 컨테이너가 아니라 전역 지도(`/map_merged`)를 구독하는
한 곳에서 돈다.

다만 master 안이 아니라 **별도 컨테이너(explorer)** 다. master 는 맵 병합·RViz·시계라는
관제 기반이고, 탐사 할당은 그 위에 얹히는 정책이라 실험 중 자주 바꿔 낀다
(거리 기준 ↔ LLM). 같이 두면 전략 하나 바꾸려고 RViz 와 맵 병합까지 재시작해야 한다.

    /map_merged ─┐
    /{ns}/odom   ├─▶ 프론티어 검출 ─▶ **할당** ─▶ /{ns}/goal_pose
    /{ns}/map   ─┘                      ▲
                                        └── strategy 가 여기를 갈아 끼운다

`strategy=distance` 는 거리 합을 최소화하는 정확한 할당(베이스라인),
`strategy=llm` 은 같은 관측을 LLM 에게 주고 배정을 받는다. **llm 일 때도 매 주기
베이스라인을 같이 계산해 짝지어 기록한다** — 둘이 같은 답을 내는 주기는 증류할 신호가
없다는 뜻이고, 갈리는 주기가 곧 학습 대상이다.

편대를 어떻게 아는가
--------------------
`/robot_registry` 를 구독한다. 로봇마다 `robot_registrar` 가 1 Hz 로 자기 명함을 보내므로
**재시작 없이** 런타임 소환/제거를 따라간다. 종류(ugv/spot/drone)는 이름 접두사로 안다 —
소환기가 `robot_types.RobotType.id_prefix` 로 그렇게 채번한다.

실측으로 얻은 규칙 네 가지
--------------------------
1. **너무 가까운 후보는 뺀다.** `dist/sqrt(cells)` 정렬은 가까운 후보가 압도적이라,
   안 빼면 로봇이 0.5 m 앞으로만 반복해 보내지고 탐사가 2라운드 만에 멈춘다.
2. **목표를 유지한다.** 매 주기 새로 배정하면 목표가 흔들려(2→6→2→4→8…) 어디에도
   도달하지 못한다. 드론이 6주기 연속 이동 0.0 m 였다.
3. **포기 조건이 있어야 한다.** 유지만 하면 도달 못 하는 목표에 묶인다. 실제로 ugv1 이
   같은 목표를 반복 재시도하며 제자리에 머물렀다.
4. **창 밖 목표는 가장자리로 자른다.** 드론의 Nav2 static layer 는 30x30 m 롤링 창이라
   그 밖은 "off the global costmap" 으로 계획이 무조건 실패한다.
"""

import json
import math
import os
import re
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from webots_goal_bridge.frontier_allocator import (PROMPT_SHA, PROMPT_VERSION,
                                                   assign_by_distance,
                                                   assign_by_llm,
                                                   build_alloc_prompt,
                                                   build_observation)
from webots_goal_bridge.llm_goal_assigner import extract_frontiers

# 🚨 맵은 TRANSIENT_LOCAL 이다. 기본 QoS 로 구독하면 에러 없이 아무것도 안 온다.
MAP_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
REL_QOS = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE)

# 이름 접두사 -> 종류. 소환기의 id_prefix 와 맞춘다.
PREFIXES = (('drone', 'drone'), ('spot', 'spot'), ('ugv', 'ugv'))


def robot_type_of(robot_id):
    for prefix, kind in PREFIXES:
        if robot_id.startswith(prefix):
            return kind
    return 'ugv'


def opt_str(value):
    """선택 문자열 파라미터. '', 'none', 'null' 은 전부 "안 줌" 이다.

    🚨 compose 에서 빈 값을 넘길 수가 없다. `dataset_path:=` 처럼 값이 빈 launch 인자는
       ros2 launch 가 "malformed launch argument" 로 거부해 컨테이너가 재시작을 반복한다
       (실측). 그래서 compose 의 기본값은 none 이고, 여기서 빈 값과 같게 취급한다.
    """
    v = str(value if value is not None else '').strip()
    return '' if v.lower() in ('', 'none', 'null') else v


def parse_bounds(text):
    """'x0,y0,x1,y1' -> (x0, y0, x1, y1). 비었으면 None.

    ROS 파라미터를 문자열로 받는 이유: launch 치환을 거친 숫자 배열은 형 추론이
    엇갈리기 쉽다. 문자열 하나면 compose 환경변수에서 그대로 넘기기도 쉽다.
    """
    text = opt_str(text).strip('[]')
    if not text:
        return None
    v = [float(t) for t in text.split(',')]
    if len(v) != 4 or v[0] >= v[2] or v[1] >= v[3]:
        raise ValueError(f"explore_bounds 는 'x0,y0,x1,y1' (x0<x1, y0<y1) 이어야 한다: {text}")
    return tuple(v)


class FrontierAllocatorNode(Node):

    def __init__(self):
        super().__init__('frontier_allocator')

        self.declare_parameter('period', 30.0)
        self.declare_parameter('strategy', 'distance')   # distance | llm
        self.declare_parameter('min_goal_dist', 4.0)
        self.declare_parameter('min_frontier_cells', 8)
        self.declare_parameter('cluster_size', 2.0)
        self.declare_parameter('max_candidates', 12)
        # 배정된 후보끼리 최소 이만큼(m) 떨어져야 한다. 안 그러면 군집 버킷이 2 m 라
        # 1.5 m 떨어진 사실상 같은 지점에 두 로봇이 배정돼 분산 효과가 사라진다(실측).
        self.declare_parameter('min_separation', 10.0)
        # 목표를 유지할 최대 주기 수. 이 안에 못 가면 버리고 다시 배정한다.
        self.declare_parameter('giveup_rounds', 3)
        # 한 주기에 이만큼도 못 움직였으면 "진전 없음"으로 센다.
        self.declare_parameter('progress_min', 1.0)
        # 증류용 학습 데이터. 비우면 안 쓴다.
        self.declare_parameter('dataset_path', '')
        # 탐사 범위 'x0,y0,x1,y1' (월드 좌표, m). 비우면 지도 전체.
        # 이 사각형 밖의 프론티어는 후보에서 뺀다 — 예: oneroom 건물 내부만 정찰.
        self.declare_parameter('explore_bounds', '')
        # 🧪 증류 데이터. dataset_path 가 비어 있고 dataset_dir 가 있으면 **실행마다 파일 하나**를
        #    {run_id}.jsonl 로 만든다. 한 파일에 이어 쓰면 실행 경계가 사라져 train/val 을
        #    실행 단위로 나눌 수 없다 (연속 주기끼리는 거의 같은 관측이라 섞이면 검증이 샌다).
        self.declare_parameter('dataset_dir', '')
        self.declare_parameter('run_tag', '')
        # strategy=llm 일 때만 쓴다. OpenAI 호환이면 무엇이든 붙는다 —
        # NVIDIA NIM / 로컬 vLLM / Ollama. base_url 과 model 만 바꾸면 된다.
        self.declare_parameter('base_url', 'https://integrate.api.nvidia.com/v1')
        self.declare_parameter('model', 'nvidia/nemotron-3-super-120b-a12b')
        self.declare_parameter('api_key_env', 'NVIDIA_API_KEY')
        # 🚨 추론(reasoning) 모델은 예산을 추론에 먼저 쓴다. 작게 주면 본문이 빈 채로
        #    finish_reason=length 가 돌아온다 — 실측: nemotron-3-super 에 768 을 주면
        #    5/5 전부 빈 응답이었고, 3000 을 줘도 '{}' 였다. 넉넉히 준다.
        self.declare_parameter('max_tokens', 2048)
        # Nemotron 계열의 추론 토글. 'detailed thinking off' 면 추론이 짧아져
        # 같은 답을 **29초 -> 9초**로 낸다(실측). 다른 제공자면 빈 문자열로 둔다.
        self.declare_parameter('system_prompt', 'detailed thinking off')
        self.declare_parameter('llm_retries', 3)

        self.period = float(self.get_parameter('period').value)
        self.strategy = str(self.get_parameter('strategy').value)
        self.min_goal_dist = float(self.get_parameter('min_goal_dist').value)
        self.min_cells = int(self.get_parameter('min_frontier_cells').value)
        self.cluster = float(self.get_parameter('cluster_size').value)
        self.max_cand = int(self.get_parameter('max_candidates').value)
        self.min_sep = float(self.get_parameter('min_separation').value)
        self.giveup = int(self.get_parameter('giveup_rounds').value)
        self.progress_min = float(self.get_parameter('progress_min').value)
        self.dataset_path = opt_str(self.get_parameter('dataset_path').value)
        self.explore_bounds = parse_bounds(str(self.get_parameter('explore_bounds').value))
        self.dataset_dir = opt_str(self.get_parameter('dataset_dir').value)
        self.run_tag = re.sub(r'[^A-Za-z0-9_-]+', '-',
                              opt_str(self.get_parameter('run_tag').value))
        self.run_id = (time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + f'_{self.strategy}'
                       + (f'_{self.run_tag}' if self.run_tag else ''))
        if not self.dataset_path and self.dataset_dir:
            try:
                os.makedirs(self.dataset_dir, exist_ok=True)
                self.dataset_path = os.path.join(self.dataset_dir, f'{self.run_id}.jsonl')
            except OSError as exc:
                self.get_logger().error(f'증류 데이터 폴더를 못 만든다 — 기록 안 함: {exc}')
        self._meta_written = False
        self._calls = []          # 이번 주기 _chat 의 HTTP 호출 원문
        self._round_events = []   # 이번 주기 포기 등

        self.merged = None
        self.pose = {}          # ns -> Pose
        self.bounds = {}        # ns -> (x0, y0, x1, y1)  계획 가능 범위
        self.goal_pub = {}
        self.target = {}        # ns -> (x, y)   지속 목표
        self.stuck = {}         # ns -> 진전 없는 주기 수
        self.last_pos = {}
        # 🚨 분산 포기(폴백) 집계. 이게 곧 비교 지표다 — 베이스라인이 몇 번이나
        #    "분리 제약을 만족하는 조합이 없어" 물러났는지가 LLM 이 개선할 여지다.
        #
        # llm 전략일 때는 **매 주기 베이스라인도 같이 계산**해 짝지어 기록한다.
        # 완전탐색이 132가지뿐이라 공짜에 가깝고, 같은 관측에서 둘이 무엇을 다르게
        # 골랐는지가 곧 "증류할 것이 있는가" 에 대한 답이다.
        # new_targets 가 "탐사 생성 점" 이다 — 로봇에게 **새로** 내준 목표 수.
        # 같은 목표를 주기마다 다시 보내는 것은 세지 않는다.
        self.stats = {'rounds': 0, 'degraded': 0, 'sep_sum': 0.0, 'sep_n': 0,
                      'new_targets': 0, 'giveups': 0,
                      'llm_fail': 0, 'fell_back': 0,
                      'llm_calls': 0, 'llm_sec': 0.0, 'late': 0,
                      'agree': 0, 'llm_better': 0, 'base_better': 0}

        self.client = None
        if self.strategy == 'llm':
            self._make_client()

        self.create_subscription(OccupancyGrid, '/map_merged', self._on_map, MAP_QOS)
        self.create_subscription(String, '/robot_registry', self._on_registry, 10)
        # 🚨 주기 처리를 **구독과 다른 콜백 그룹**에 둔다. llm 전략은 한 번 물어보는 데
        #    12~65초가 걸리는데(실측, nemotron-3-super), 같은 그룹이면 그동안 odom/map
        #    구독이 통째로 멈춰 다음 주기가 수십 초 묵은 위치로 배정하게 된다.
        #    main() 의 MultiThreadedExecutor 와 짝이다 — 한쪽만 바꾸면 효과가 없다.
        self.create_timer(self.period, self.cycle,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(
            f'프론티어 할당기 시작 | 전략 {self.strategy} | 주기 {self.period}s | '
            f'최소 목표거리 {self.min_goal_dist} m | 최소 분리 {self.min_sep} m | '
            f'탐사 범위 {self.explore_bounds or "지도 전체"}')

    # ----------------------------------------------------------------- LLM
    def _make_client(self):
        """OpenAI 호환 클라이언트. 실패해도 노드는 뜨고 베이스라인으로 돈다.

        🚨 여기서 죽으면 안 된다. 키가 없거나 openai 패키지가 빠진 이미지에서
           할당기가 통째로 안 뜨면, 원인이 "탐사가 안 된다" 로만 보인다.
        """
        key_env = str(self.get_parameter('api_key_env').value)
        api_key = os.environ.get(key_env, '')
        if not api_key:
            self.get_logger().error(
                f'{key_env} 가 비어 있다 — LLM 전략을 켰지만 매 주기 베이스라인으로 '
                f'물러난다. compose 의 environment 에 키가 전달됐는지 확인할 것')
            return
        try:
            from openai import OpenAI       # 지연 임포트 — 키가 없어도 노드는 뜬다
            self.client = OpenAI(
                base_url=str(self.get_parameter('base_url').value), api_key=api_key)
        except Exception as exc:            # noqa: BLE001
            self.get_logger().error(f'LLM 클라이언트 생성 실패: {exc}')
            return
        self.get_logger().info(
            f'LLM 할당 준비 | {self.get_parameter("model").value} @ '
            f'{self.get_parameter("base_url").value}')

    def _chat(self, prompt):
        """assign_by_llm 에 넘길 이음매. 응답 본문 문자열만 돌려준다.

        🚨 추론 모델은 예산을 추론에 먼저 쓴다. 다 쓰면 본문이 **빈 문자열**로 오고
           finish_reason 이 'length' 가 된다. 그대로 두면 호출부가 JSONDecodeError
           만 보게 되어 원인이 "모델이 이상한 답을 한다" 로 오독된다. 여기서 잘림을
           구분해 말해 주고, 한 번은 예산을 두 배로 늘려 다시 물어본다.

           예산을 키우는 것이 만능은 아니다 — 실측(후보 12개, 3회씩):
             2048 → 3/3 성공, 평균 17.6초
             4096 → 3/3 성공, 평균 24.5초
             6144 → 2/3 성공, 평균 83.1초 ('length' 1회)
           추론 길이 자체가 실행마다 크게 흔들려서, 예산을 키우면 더 길게 헤매다
           같은 벽에 부딪히기도 한다. 그래서 기본값은 2048 로 두고 한 번만 늘린다.
        """
        sys_msg = str(self.get_parameter('system_prompt').value).strip()
        messages = ([{'role': 'system', 'content': sys_msg}] if sys_msg else []) \
            + [{'role': 'user', 'content': prompt}]
        budget = int(self.get_parameter('max_tokens').value)
        for factor in (1, 2):
            t_call = time.monotonic()
            resp = self.client.chat.completions.create(
                model=str(self.get_parameter('model').value),
                messages=messages,
                response_format={'type': 'json_object'},
                max_tokens=budget * factor,
                temperature=0.2)
            choice = resp.choices[0]
            body = (choice.message.content or '').strip()
            usage = getattr(resp, 'usage', None)
            # 증류용 원문. 추론 텍스트(reasoning_content)도 남긴다 — 학생에게 추론까지
            # 가르칠지는 내보낼 때 정한다. 잘린 호출도 버리지 않는다.
            self._calls.append({
                'max_tokens': budget * factor,
                'finish_reason': choice.finish_reason,
                'content': choice.message.content,
                'reasoning': getattr(choice.message, 'reasoning_content', None),
                'latency_s': round(time.monotonic() - t_call, 2),
                'prompt_tokens': getattr(usage, 'prompt_tokens', None),
                'completion_tokens': getattr(usage, 'completion_tokens', None),
            })
            # 🚨 finish_reason 을 본문보다 **먼저** 본다. 잘렸을 때 본문이 비어서만 오는
            #    게 아니다 — 추론 텍스트가 본문 자리에 들어온 채 잘리기도 한다(실측:
            #    max_tokens=256, finish=length, 본문 = 'Okay, the user is asking...').
            #    본문 유무로 판단하면 그 경우를 정상 응답으로 넘겨 JSON 파싱에서 죽는다.
            if choice.finish_reason != 'length':
                if body:
                    return body
                raise ValueError(
                    f'본문이 비었다 (finish_reason={choice.finish_reason})')
            self.get_logger().warn(
                f'추론이 예산 {budget * factor} 토큰을 다 써 본문이 잘렸다'
                + (' — 두 배로 다시 물어본다' if factor == 1 else ''))
        raise ValueError(f'예산 {budget * 2} 토큰으로도 본문이 안 나왔다')

    # ------------------------------------------------------------- 편대 파악
    def _on_registry(self, msg):
        """로봇이 스스로 보내는 명함. 런타임 소환을 재시작 없이 따라간다."""
        try:
            info = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        ns = info.get('robot_id')
        if not ns or ns in self.goal_pub:
            return
        kind = robot_type_of(ns)
        self.create_subscription(
            Odometry, f'/{ns}/odom',
            lambda m, k=ns: self.pose.__setitem__(k, m.pose.pose), REL_QOS)
        # 드론의 Nav2 static layer 는 map_active(현재 순항 고도 층)다.
        topic = f'/{ns}/map_active' if kind == 'drone' else f'/{ns}/map'
        self.create_subscription(
            OccupancyGrid, topic,
            lambda m, k=ns: self.bounds.__setitem__(k, (
                m.info.origin.position.x, m.info.origin.position.y,
                m.info.origin.position.x + m.info.width * m.info.resolution,
                m.info.origin.position.y + m.info.height * m.info.resolution)),
            MAP_QOS)
        # 🚨 드론도 goal_pose 를 쓴다. goal_pose_3d 는 altitude_selector 를 거치는데,
        #    그 노드는 고른 층에 기체가 도달해야 Nav2 로 넘긴다. 창이 새 영역으로 가서
        #    세 층이 모두 미탐색이면 통과하는 층이 없어 목표를 영영 안 넘긴다(실측).
        self.goal_pub[ns] = self.create_publisher(PoseStamped, f'/{ns}/goal_pose', 10)
        self.get_logger().info(f'편대에 합류: {ns} ({kind})')

    def _on_map(self, msg):
        self.merged = msg

    # ------------------------------------------------------------- 목표 전송
    def clamp(self, ns, x, y):
        """계획 가능 범위 밖이면 범위 안까지만 자른다(경유점).

        🚨 축별로 자른다. 예전에는 로봇→목표 광선 전체를 스칼라 s 로 줄였는데
           (s = 두 축의 교차 매개변수 중 min), **한 축의 미미한 여유가 다른 축의
           긴 전진을 통째로 죽였다.** 실측 사례:

             ugv1 (-48.11, -23.42) → 목표 (-49.6, 41.0)   dx=-1.49, dy=+64.4
             맵 좌측 끝 -50.11 이라 여백 2 m 를 빼면 x0 = -48.11 = 로봇의 x
             → x축 s 후보 = 0 → min 이 이걸 채택 → 경유점 = 로봇 자기 위치

           경유점이 제자리면 Nav2 는 즉시 "Reached the goal" 을 내고 로봇은 안
           움직인다. 안 움직이니 맵이 안 자라고, 맵이 그대로니 다음 주기도 똑같다
           — **영구 교착**이다. 커버리지가 20% 언저리에서 평평해진 원인이 이것이었다.

           축별로 자르면 위 사례는 (-48.11, 41.0) 이 되어 북쪽 64 m 가 살아난다.
        """
        b = self.bounds.get(ns)
        if not b or ns not in self.pose:
            return x, y
        m = 2.0                      # 가장자리에 딱 붙이면 코스트맵 경계에 걸린다
        # 맵이 여백 두 배보다 좁으면 상자가 뒤집힌다. 그때는 여백을 포기한다.
        mx = m if b[2] - b[0] > 2 * m else 0.0
        my = m if b[3] - b[1] > 2 * m else 0.0
        gx = min(max(x, b[0] + mx), b[2] - mx)
        gy = min(max(y, b[1] + my), b[3] - my)

        # 그래도 제자리에 가까우면(로봇이 맵 구석에 박혀 모든 축이 막힌 경우)
        # 목표 방향으로 최소 전진거리만큼은 밀어 준다. 코스트맵을 벗어나 Nav2 가
        # 거절할 수 있지만, 확실한 교착보다는 낫다.
        rx, ry = self.pose[ns].position.x, self.pose[ns].position.y
        if math.hypot(gx - rx, gy - ry) < self.min_goal_dist:
            dx, dy = x - rx, y - ry
            d = math.hypot(dx, dy)
            if d > 1e-6:
                step = min(d, self.min_goal_dist)
                gx, gy = rx + dx / d * step, ry + dy / d * step
        return gx, gy

    def send_goal(self, ns, x, y):
        gx, gy = self.clamp(ns, x, y)
        g = PoseStamped()
        # 🚨 'map' 이 아니라 '{ns}/map'. 틀리면 에러 없이 무시된다.
        g.header.frame_id = f'{ns}/map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = float(gx), float(gy)
        g.pose.orientation.w = 1.0
        self.goal_pub[ns].publish(g)
        return gx, gy

    # ------------------------------------------------------------- 주기 처리
    def cycle(self):
        if self.merged is None:
            self.get_logger().warn(
                '/map_merged 를 아직 못 받았다 (QoS 를 먼저 의심할 것 — TRANSIENT_LOCAL)',
                throttle_duration_sec=30.0)
            return
        active = [ns for ns in self.goal_pub if ns in self.pose]
        if not active:
            self.get_logger().info('배정할 로봇이 없다', throttle_duration_sec=30.0)
            return

        info = self.merged.info
        grid = np.asarray(self.merged.data, dtype=np.int16).reshape(
            info.height, info.width)
        robots = [{'id': ns, 'type': robot_type_of(ns),
                   'x': self.pose[ns].position.x,
                   'y': self.pose[ns].position.y} for ns in sorted(active)]

        fr = extract_frontiers(grid, info.origin.position.x, info.origin.position.y,
                               info.resolution, robots[0]['x'], robots[0]['y'],
                               min_cells=self.min_cells, bucket=self.cluster,
                               max_n=self.max_cand * 4, bounds=self.explore_bounds)
        # 어느 로봇에게도 너무 가깝지 않은 것만 (규칙 1)
        fr = [f for f in fr
              if min(math.hypot(f['x'] - r['x'], f['y'] - r['y']) for r in robots)
              >= self.min_goal_dist][:self.max_cand]
        for i, f in enumerate(fr):
            f['id'] = i
        if not fr:
            self.get_logger().info(
                '프론티어 후보가 없다 — '
                + ('탐사 범위 안을 다 봤다' if self.explore_bounds
                   else '탐사 완료이거나 맵이 비었다'),
                throttle_duration_sec=60.0)
            self._record_event('no_frontiers')
            return

        obs = build_observation(grid, info.origin.position.x, info.origin.position.y,
                                info.resolution, robots, fr)
        # 베이스라인은 **언제나** 계산한다. llm 전략일 때도 마찬가지다 — 완전탐색이
        # 132가지뿐이라 비용이 없고, 같은 관측에서 둘이 무엇을 다르게 골랐는지가
        # 곧 증류 가치의 증거다. 짝지어 데이터셋에 남긴다.
        baseline = assign_by_distance(obs, min_separation=self.min_sep)
        teacher = None
        if self.strategy == 'llm' and self.client is not None:
            t0 = time.monotonic()
            self._calls = []
            attempts = []
            result = assign_by_llm(
                obs, self._chat, min_separation=self.min_sep,
                retries=int(self.get_parameter('llm_retries').value),
                on_retry=lambda i, n, msg: self.get_logger().warn(
                    f'LLM 응답 불량 ({i}/{n}) — {msg}'),
                on_attempt=lambda i, raw, ok, err: attempts.append(
                    {'attempt': i, 'ok': ok, 'error': err, 'raw': raw}))
            took = time.monotonic() - t0
            self.stats['llm_calls'] += 1
            self.stats['llm_sec'] = round(self.stats['llm_sec'] + took, 1)
            teacher = {'latency_s': round(took, 2), 'calls': self._calls,
                       'attempts': attempts}
            if took > self.period:
                self.stats['late'] += 1
                # 벽시계 기준이다. 주기를 넘기면 배정이 묵은 위치로 나가기 시작한다.
                self.get_logger().warn(
                    f'LLM 응답에 {took:.0f}초 걸렸다 — 주기 {self.period}초를 넘겼다. '
                    f'period 를 늘리거나 더 작은 모델을 쓸 것')
        else:
            result = baseline

        self._compare(result, baseline)
        self.stats['rounds'] += 1
        sep = result.get('achieved_separation')
        if sep is not None:
            self.stats['sep_sum'] += sep
            self.stats['sep_n'] += 1
        if result.get('degraded'):
            self.stats['degraded'] += 1
            # 조용히 넘어가면 "분리 제약을 걸었다"고 믿는 채로 두 로봇이 3 m 옆에
            # 배정되는 일이 생긴다(실측). 반드시 남긴다.
            self.get_logger().warn(
                f"분리 완화 — {result.get('reason', '')} | 확보 {sep} m "
                f"(누적 {self.stats['degraded']}/{self.stats['rounds']}회)")
        fresh = {a['robot']: next(f for f in fr if f['id'] == a['frontier_id'])
                 for a in result['assignments']}

        self._round_events = []
        sent = {}
        for r in robots:
            ns = r['id']
            self._update_progress(ns, r)
            tgt = self._pick_target(ns, r, fr, fresh)
            if tgt is None:
                continue
            gx, gy = self.send_goal(ns, *tgt)
            sent[ns] = {'target': [round(tgt[0], 2), round(tgt[1], 2)],
                        'waypoint': [round(gx, 2), round(gy, 2)]}
            self.get_logger().info(
                f'[{ns}] 목표 ({tgt[0]:.1f}, {tgt[1]:.1f})'
                + (f' -> 경유점 ({gx:.1f}, {gy:.1f})'
                   if abs(gx - tgt[0]) + abs(gy - tgt[1]) > 0.1 else ''))
        self._record(obs, result, baseline, extra={
            'round': self.stats['rounds'],
            'prompt_version': PROMPT_VERSION, 'prompt_sha': PROMPT_SHA,
            'min_separation': self.min_sep,
            'prompt': build_alloc_prompt(obs, self.min_sep),
            'teacher': teacher,
            # 판단 **시점**의 탐색 셀 수. 다음 행과의 차이가 이 판단의 결과(보상)다.
            'explored': self._explored(grid, info),
            'sent': sent,
            'events': list(self._round_events),
        })
        if self.stats['rounds'] % 20 == 0:
            avg = (self.stats['sep_sum'] / self.stats['sep_n']
                   if self.stats['sep_n'] else 0.0)
            line = (f"집계: {self.stats['rounds']}주기 | 새 목표 "
                    f"{self.stats['new_targets']}개 · 포기 {self.stats['giveups']}회 | 분리 완화 "
                    f"{self.stats['degraded']}회 "
                    f"({100.0 * self.stats['degraded'] / self.stats['rounds']:.0f}%) | "
                    f"평균 확보 분리 {avg:.1f} m (요구 {self.min_sep} m)")
            if self.strategy == 'llm':
                line += (f" || LLM 폴백 {self.stats['fell_back']}회 "
                         f"(버린 응답 {self.stats['llm_fail']}건) | "
                         f"베이스라인과 동일 {self.stats['agree']}회 | "
                         f"분리 우위 LLM {self.stats['llm_better']} : "
                         f"{self.stats['base_better']} 베이스라인")
            self.get_logger().info(line)

    def _update_progress(self, ns, r):
        """진전이 없으면 센다. 규칙 3 — 포기 조건의 근거."""
        prev = self.last_pos.get(ns)
        now = (r['x'], r['y'])
        if prev is not None:
            moved = math.hypot(now[0] - prev[0], now[1] - prev[1])
            self.stuck[ns] = 0 if moved >= self.progress_min else self.stuck.get(ns, 0) + 1
        self.last_pos[ns] = now

    def _pick_target(self, ns, r, fr, fresh):
        """지속 목표를 유지하되, 도달했거나 진전이 없으면 새로 받는다."""
        tgt = self.target.get(ns)
        if tgt:
            far = math.hypot(tgt[0] - r['x'], tgt[1] - r['y']) > self.min_goal_dist
            still = min((math.hypot(tgt[0] - f['x'], tgt[1] - f['y']) for f in fr),
                        default=1e9) < 6.0
            if far and still and self.stuck.get(ns, 0) < self.giveup:
                return tgt
            if self.stuck.get(ns, 0) >= self.giveup:
                self.get_logger().warn(
                    f'[{ns}] {self.giveup}주기 동안 진전이 없어 목표를 버린다')
                self.stuck[ns] = 0
                self.stats['giveups'] += 1
                self._round_events.append({'robot': ns, 'event': 'giveup',
                                           'target': [round(tgt[0], 2), round(tgt[1], 2)]})
        f = fresh.get(ns)
        if f is None:
            return None
        if tgt is None or math.hypot(f['x'] - tgt[0], f['y'] - tgt[1]) > 0.5:
            self.stats['new_targets'] += 1
        self.target[ns] = (f['x'], f['y'])
        return self.target[ns]

    def _compare(self, result, baseline):
        """LLM 과 베이스라인이 같은 관측에서 무엇을 다르게 골랐는가.

        🚨 잣대는 거리 합이 아니라 **확보한 분리**를 먼저 본다. 거리 합만 보면
           베이스라인이 이기는 게 당연하다 — 베이스라인은 바로 그 값을 최소화하도록
           짜였으니 자기 잣대로 재는 셈이다. LLM 이 기여할 수 있는 자리는
           "거리를 조금 더 쓰더라도 두 대를 더 멀리 떼어 놓는" 선택이다.
        """
        if result is baseline:
            return
        if result.get('fell_back'):
            self.stats['fell_back'] += 1
        self.stats['llm_fail'] += int(result.get('llm_failures', 0))

        a = {(x['robot'], x['frontier_id']) for x in result['assignments']}
        b = {(x['robot'], x['frontier_id']) for x in baseline['assignments']}
        if a == b:
            self.stats['agree'] += 1
            return
        rs, bs = result.get('achieved_separation'), baseline.get('achieved_separation')
        if rs is None or bs is None:
            return
        if rs > bs:
            self.stats['llm_better'] += 1
        elif bs > rs:
            self.stats['base_better'] += 1
        self.get_logger().info(
            f'판단 갈림 | LLM 분리 {rs} m (거리합 {result.get("total_cost")}) vs '
            f'베이스라인 {bs} m (거리합 {baseline.get("total_cost")})'
            + (f' | LLM 근거: {result.get("reason", "")[:60]}'
               if result.get('reason') else ''))

    def _sim_now(self):
        return round(self.get_clock().now().nanoseconds / 1e9, 1)

    def _explored(self, grid, info):
        """탐사 범위 안(범위가 없으면 지도 전체)의 탐색된 셀 수."""
        b = self.explore_bounds
        if b is None:
            return {'known': int((grid >= 0).sum()), 'total': int(grid.size)}
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        c0 = max(0, int(math.floor((b[0] - ox) / res)))
        c1 = min(info.width, int(math.ceil((b[2] - ox) / res)))
        r0 = max(0, int(math.floor((b[1] - oy) / res)))
        r1 = min(info.height, int(math.ceil((b[3] - oy) / res)))
        win = grid[r0:r1, c0:c1]
        total = int(round((b[2] - b[0]) / res)) * int(round((b[3] - b[1]) / res))
        return {'known': int((win >= 0).sum()), 'total': total}

    def _meta(self):
        """실행 첫 줄. 이 파일의 데이터가 **어떤 조건에서** 나왔는지 — 섞어 학습할 때 거를 근거."""
        g = lambda k: self.get_parameter(k).value
        return {
            'type': 'meta', 'run_id': self.run_id, 'run_tag': self.run_tag,
            'strategy': self.strategy, 'started_utc': self.run_id.split('_')[0],
            'model': g('model'), 'base_url': g('base_url'),
            'system_prompt': g('system_prompt'), 'max_tokens': g('max_tokens'),
            'temperature': 0.2, 'llm_retries': g('llm_retries'),
            'period': self.period, 'min_separation': self.min_sep,
            'min_goal_dist': self.min_goal_dist, 'max_candidates': self.max_cand,
            'giveup_rounds': self.giveup,
            'explore_bounds': list(self.explore_bounds) if self.explore_bounds else None,
            'prompt_version': PROMPT_VERSION, 'prompt_sha': PROMPT_SHA,
        }

    def _append(self, row):
        if not self.dataset_path:
            return
        try:
            with open(self.dataset_path, 'a', encoding='utf-8') as fh:
                if not self._meta_written:
                    fh.write(json.dumps(self._meta(), ensure_ascii=False) + '\n')
                    self._meta_written = True
                fh.write(json.dumps(row, ensure_ascii=False) + '\n')
        except OSError as exc:
            self.get_logger().warn(f'데이터셋 기록 실패: {exc}', throttle_duration_sec=60.0)

    def _record_event(self, event):
        """배정이 없는 주기도 남긴다 — 탐사 완료 시각을 데이터셋에서 읽을 수 있게."""
        self._append({'type': 'event', 't_sim': self._sim_now(), 'event': event,
                      'stats': dict(self.stats)})

    def _record(self, obs, result, baseline=None, extra=None):
        """관측 + 두 전략의 답 + 교사 원문 + 판단 시점 탐색량을 한 줄에 남긴다.

        증류 학습은 이 파일을 먹는다. 교사(LLM)의 답만 남기면 "교사가 옳았나" 를 나중에
        되물을 수 없으므로, **같은 관측에 대한 베이스라인의 답도 함께** 남긴다. 둘이 같은
        주기는 학습 신호가 없다는 뜻이기도 하다. 내보내기는 scripts/export_distill.py.
        """
        row = {'type': 'round', 't_sim': self._sim_now(), 'observation': obs,
               'result': result, 'stats': dict(self.stats)}
        if baseline is not None and baseline is not result:
            row['baseline'] = baseline
        if extra:
            row.update(extra)
        self._append(row)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierAllocatorNode()
    # 스레드 2개면 충분하다 — 주기 처리 1개 + 구독 전부 1개.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
