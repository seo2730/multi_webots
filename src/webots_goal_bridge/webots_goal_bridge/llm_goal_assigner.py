"""LLM 에게 다음 탐사 목표를 고르게 하고 Nav2 로 보낸다.

OpenAI 호환 엔드포인트면 무엇이든 붙는다 — NVIDIA NIM(build.nvidia.com), 로컬 vLLM,
Ollama. `base_url` 과 `model` 파라미터만 바꾸면 된다. **지식 증류를 염두에 둔 설계다**:
티처(NIM 의 큰 모델)로 데이터를 모으고, 나중에 학습한 스튜던트를 로컬에 띄운 뒤
같은 노드를 그대로 재사용한다.

왜 지도를 이미지로 안 주는가
-----------------------------
예전 구현(gemini_goal_assigner)은 OccupancyGrid 를 PNG 로 그려 비전 모델에 넘겼다.
그 방식은 증류에 불리하다.

  1. **입력 분포가 갈린다.** 티처는 이미지를, 8 GB 로 돌릴 작은 스튜던트는 텍스트를
     보게 되면 배우는 대상이 달라진다. 티처·스튜던트가 같은 것을 봐야 한다.
  2. **프론티어 찾기는 기하 계산이지 추론이 아니다.** 격자에서 후보를 뽑는 일은
     결정적 알고리즘이 정확하고 싸다. LLM 이 픽셀을 세는 것보다 낫다.
  3. **출력 공간이 좁아진다.** "좌표를 지어내라"가 아니라 "후보 중 고르라"가 되면
     작은 모델도 배울 수 있고, 좌표를 환각할 수 없다.

그래서 후보 추출은 여기서 하고, LLM 에게는 **고르게만** 시킨다.

🚨 이 노드가 밟지 않도록 조심한 함정들 (전부 문서에 기록된 것)
    - `/{ns}/map` 은 TRANSIENT_LOCAL 이다. 기본 QoS 로 구독하면 **에러 없이** 안 온다
    - 목표의 frame_id 는 `{ns}/map` 이어야 한다. 'map' 으로 두면 조용히 무시된다
    - use_sim_time 을 켜야 한다. 안 켜면 목표 스탬프가 벽시계라 Nav2 가 헛돈다
"""

import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

UNKNOWN, FREE = -1, 0
OCCUPIED_MIN = 50          # 이 값 이상이면 장애물로 본다


def map_qos():
    """🚨 맵은 TRANSIENT_LOCAL 이다. 기본 QoS 로 구독하면 조용히 아무것도 안 온다."""
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def extract_frontiers(grid, origin_x, origin_y, res, rx, ry,
                      min_cells=5, bucket=1.0, max_n=8, bounds=None):
    """미탐색과 맞닿은 자유 공간(프론티어)을 덩어리로 묶어 후보 목록을 만든다.

    ROS 에 의존하지 않는 순수 함수다 — 격자와 숫자만 받는다. 그래야 시뮬을 띄우지
    않고 검증할 수 있고, 나중에 학습 데이터를 오프라인으로 다시 만들 때도 그대로 쓴다.

    고전적인 정의 그대로다: **자유 셀인데 4-이웃에 미탐색이 있는 셀.** 그런 셀이
    수백 개 나오므로 bucket(m) 격자로 묶어 대표점 하나씩으로 줄인다.
    (scipy 연결요소 대신 격자 버킷을 쓴다 — 의존성이 늘지 않고, 탐사 목표를 고르는
     해상도로는 충분하다.)

    bounds=(x0, y0, x1, y1) 를 주면 그 사각형 안의 프론티어 셀만 쓴다.
    🚨 **정렬·자르기 전에** 거른다. 뒤에서 거르면 가까운 외부 프론티어가 상위 max_n 을
       차지해 범위 안 후보가 잘려 나간다 — 건물 밖 마당이 로봇 곁에 있으면 실제로 그렇다.
    """
    h, w = grid.shape
    if w == 0 or h == 0:
        return []
    free = (grid == FREE)
    unknown = (grid == UNKNOWN)
    nbr_unknown = np.zeros_like(unknown)
    nbr_unknown[1:, :] |= unknown[:-1, :]
    nbr_unknown[:-1, :] |= unknown[1:, :]
    nbr_unknown[:, 1:] |= unknown[:, :-1]
    nbr_unknown[:, :-1] |= unknown[:, 1:]
    fr = free & nbr_unknown

    js, is_ = np.nonzero(fr)                 # (행=y, 열=x)
    if js.size == 0:
        return []
    wx = origin_x + (is_ + 0.5) * res
    wy = origin_y + (js + 0.5) * res
    if bounds is not None:
        x0, y0, x1, y1 = bounds
        keep = (wx >= x0) & (wx <= x1) & (wy >= y0) & (wy <= y1)
        wx, wy = wx[keep], wy[keep]
        if wx.size == 0:
            return []

    keys = (np.floor(wx / bucket).astype(np.int64) << 32) ^ \
           np.floor(wy / bucket).astype(np.int64)
    out = []
    for k in np.unique(keys):
        sel = keys == k
        n = int(sel.sum())
        if n < min_cells:
            continue
        cx, cy = float(wx[sel].mean()), float(wy[sel].mean())
        out.append({'x': round(cx, 2), 'y': round(cy, 2), 'cells': n,
                    'dist': round(float(np.hypot(cx - rx, cy - ry)), 2)})

    # 가까우면서 큰 것을 위로. LLM 에게 넘길 개수를 줄이는 것이 목적이다.
    out.sort(key=lambda c: (c['dist'] / max(1.0, c['cells'] ** 0.5)))
    out = out[:max_n]
    for i, c in enumerate(out):
        c['id'] = i
    return out


class LlmGoalAssigner(Node):

    def __init__(self):
        super().__init__('llm_goal_assigner')

        self.declare_parameter('namespace', 'ugv1')
        # OpenAI 호환 엔드포인트. 제공자를 갈아 끼우는 지점이다 — 코드는 그대로 둔다.
        #
        #   Google AI Studio  https://generativelanguage.googleapis.com/v1beta/openai/
        #                     키: GEMINI_API_KEY,  모델 예: gemini-2.0-flash
        #   NVIDIA NIM        https://integrate.api.nvidia.com/v1
        #                     키: NVIDIA_API_KEY,  모델: nvidia/nemotron-3-super-120b-a12b
        #   로컬 vLLM/Ollama  http://호스트:8000/v1  ← 증류한 스튜던트를 여기에 띄운다
        #
        # 티처를 상용 API 로 두면 **가중치를 얻을 수 없어 로짓 단위(화이트박스) 증류는
        # 불가능하다.** 출력만 모으는 블랙박스 증류로 한정된다는 뜻이고, 약관상 출력으로
        # 경쟁 모델을 학습하는 것을 제한하는 경우가 있으니 실제로 쓰기 전에 확인할 것.
        self.declare_parameter('base_url', 'https://integrate.api.nvidia.com/v1')
        # ⚠️ 카탈로그에 있어도 안 되는 모델이 많다 (실측): llama-3.1-nemotron-70b 404,
        #    deepseek-v4-pro / kimi-k3 90초 타임아웃. 아래는 실제로 통과한 값이다
        #    (5.2s, response_format 지원).
        self.declare_parameter('model', 'nvidia/nemotron-3-super-120b-a12b')
        # 키를 파라미터로 받지 않는다 — ros2 param 덤프나 로그에 그대로 찍히기 때문이다.
        self.declare_parameter('api_key_env', 'NVIDIA_API_KEY')
        self.declare_parameter('period', 30.0)
        self.declare_parameter('request_timeout', 30.0)
        self.declare_parameter('max_candidates', 8)
        # 이보다 작은 프론티어 덩어리는 잡음으로 보고 버린다.
        self.declare_parameter('min_frontier_cells', 5)
        # 후보를 묶는 격자 크기(m). 인접한 경계 셀 수백 개를 한 후보로 만든다.
        self.declare_parameter('cluster_size', 1.0)
        # 증류용 학습 데이터를 남길 경로. 비우면 안 쓴다.
        self.declare_parameter('dataset_path', '')
        # 🚨 너무 작으면 응답이 잘려 JSON 파싱이 깨진다. 실측: 250 에서 잘림 발생.
        self.declare_parameter('max_tokens', 512)
        # 스키마를 어기는 응답이 드물지 않다 (실측 6회 중 3회 빈 JSON). 재시도한다.
        self.declare_parameter('retries', 3)

        self.ns = self.get_parameter('namespace').value
        self.model = self.get_parameter('model').value
        self.timeout = float(self.get_parameter('request_timeout').value)
        self.max_candidates = int(self.get_parameter('max_candidates').value)
        self.min_cells = int(self.get_parameter('min_frontier_cells').value)
        self.cluster_size = float(self.get_parameter('cluster_size').value)
        self.dataset_path = str(self.get_parameter('dataset_path').value).strip()
        self.max_tokens = int(self.get_parameter('max_tokens').value)
        self.retries = int(self.get_parameter('retries').value)

        if not self.get_parameter('use_sim_time').value:
            # 🚨 목표 스탬프가 벽시계가 되면 Nav2 가 수백 초 과거로 TF 를 조회한다.
            self.get_logger().warn(
                'use_sim_time 이 false 다. 목표 스탬프가 벽시계 시각이 되어 Nav2 가 '
                '헛돈다. 런치에서 use_sim_time:=true 로 띄울 것')

        key_env = self.get_parameter('api_key_env').value
        api_key = os.environ.get(key_env, '')
        if not api_key:
            self.get_logger().error(
                f'환경 변수 {key_env} 가 비어 있다. build.nvidia.com 에서 받은 키를 '
                f'compose 의 environment 에 넣어라 (값을 코드에 적지 말 것)')

        from openai import OpenAI          # 지연 임포트 — 키가 없어도 노드는 뜬다
        self.client = OpenAI(base_url=self.get_parameter('base_url').value,
                             api_key=api_key or 'missing',
                             timeout=self.timeout)

        self.map_msg = None
        self.pose = None
        self.busy = False

        self.create_subscription(
            OccupancyGrid, f'/{self.ns}/map', self._on_map, map_qos())
        self.create_subscription(
            Odometry, f'/{self.ns}/odom', self._on_odom, 10)
        self.nav = ActionClient(
            self, NavigateToPose, f'/{self.ns}/navigate_to_pose')

        self.create_timer(float(self.get_parameter('period').value), self._cycle)
        self.get_logger().info(
            f'[{self.ns}] LLM 목표 할당기 시작 | 모델 {self.model} | '
            f'{self.get_parameter("base_url").value}'
            + (f' | 데이터셋 {self.dataset_path}' if self.dataset_path else ''))

    # ------------------------------------------------------------------ 입력
    def _on_map(self, msg):
        self.map_msg = msg

    def _on_odom(self, msg):
        self.pose = msg.pose.pose

    # ------------------------------------------------------- 프론티어 후보 추출
    def frontiers(self):
        info = self.map_msg.info
        grid = np.asarray(self.map_msg.data, dtype=np.int16).reshape(
            info.height, info.width)
        return extract_frontiers(
            grid, info.origin.position.x, info.origin.position.y, info.resolution,
            self.pose.position.x, self.pose.position.y,
            min_cells=self.min_cells, bucket=self.cluster_size,
            max_n=self.max_candidates)

    def observation(self, cands):
        """LLM 과 학습 데이터가 **공유하는** 상태 표현. 이미지가 아니라 텍스트다."""
        info = self.map_msg.info
        grid = np.asarray(self.map_msg.data, dtype=np.int16)
        total = max(1, grid.size)
        return {
            'robot': {'x': round(self.pose.position.x, 2),
                      'y': round(self.pose.position.y, 2)},
            'map': {'width': info.width, 'height': info.height,
                    'resolution': round(info.resolution, 3),
                    'explored_pct': round(100.0 * (grid >= 0).sum() / total, 1),
                    'occupied_pct': round(100.0 * (grid >= OCCUPIED_MIN).sum() / total, 1)},
            'candidates': cands,
        }

    # ------------------------------------------------------------------ LLM
    def ask(self, obs):
        """후보 중 하나를 고르게 한다. 좌표를 생성시키지 않으므로 환각이 불가능하다.

        🚨 스키마를 어기는 응답이 드물지 않다. 실측(nemotron-3-super, 6회):
           성공 1 / 빈 JSON 3 / HTTP 503 1 / 응답 잘림 1. response_format 을 줘도
           빈 객체 `{}` 가 오는 경우가 있어서 **재시도와 검증이 필수**다.
        """
        prompt = (
            '너는 실내 탐사 로봇의 경로 계획기다. 아래는 로봇의 현재 상태와, 이미 계산된 '
            '탐사 후보(프론티어) 목록이다.\n\n'
            f'{json.dumps(obs, ensure_ascii=False, indent=2)}\n\n'
            'cells 는 그 후보에 속한 경계 셀 수(클수록 넓은 미탐색 영역), dist 는 '
            '로봇으로부터의 직선 거리(m)다.\n'
            '가장 탐사 효율이 좋은 후보를 하나 골라라. 좌표를 새로 만들지 말고 '
            '반드시 목록의 id 중에서 고른다.\n'
            '아래 JSON 형식으로만 답한다: {"choice_id": <정수>, "reason": "<한 문장>"}')
        valid = {c['id'] for c in obs['candidates']}
        last = None
        for attempt in range(max(1, self.retries)):
            try:
                resp = self.client.chat.completions.create(
                    model=self.model,
                    messages=[{'role': 'user', 'content': prompt}],
                    response_format={'type': 'json_object'},
                    max_tokens=self.max_tokens,
                    temperature=0.2)
                ans = json.loads(resp.choices[0].message.content)
                if ans.get('choice_id') in valid:
                    return ans
                last = f'스키마 위반 또는 범위 밖: {str(ans)[:80]}'
            except Exception as exc:                  # noqa: BLE001
                last = f'{type(exc).__name__}: {exc}'
            self.get_logger().warn(
                f'LLM 응답 불량 ({attempt + 1}/{self.retries}) — {last}')
        return None

    # ------------------------------------------------------------------ 출력
    def send_goal(self, x, y):
        if not self.nav.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f'/{self.ns}/navigate_to_pose 에 연결 실패')
            return False
        goal = NavigateToPose.Goal()
        # 🚨 'map' 이 아니라 '{ns}/map'. 로봇마다 좌표계 이름이 다르고,
        #    틀리면 에러 없이 무시된다 (01_INTERFACES.md).
        goal.pose.header.frame_id = f'{self.ns}/map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        self.nav.send_goal_async(goal)
        return True

    def record(self, obs, choice_id, reason, chosen):
        """증류용 학습 데이터. 상태와 선택을 그대로 남긴다.

        나중에 시뮬로 실행해 성공한 것만 남기는 필터(rejection sampling)를 거치면
        티처의 실수를 학생이 따라 배우는 것을 막을 수 있다.
        """
        if not self.dataset_path:
            return
        rec = {'ts': time.time(), 'ns': self.ns, 'model': self.model,
               'observation': obs, 'choice_id': choice_id, 'reason': reason,
               'chosen': chosen}
        try:
            os.makedirs(os.path.dirname(self.dataset_path) or '.', exist_ok=True)
            with open(self.dataset_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(rec, ensure_ascii=False) + '\n')
        except OSError as exc:
            self.get_logger().warn(f'데이터셋 기록 실패: {exc}')

    # ------------------------------------------------------------------ 루프
    def _cycle(self):
        if self.map_msg is None or self.pose is None:
            self.get_logger().warn(
                '지도나 위치를 아직 못 받았다 '
                '(맵이 안 오면 QoS 를 먼저 의심할 것 — TRANSIENT_LOCAL)',
                throttle_duration_sec=30.0)
            return
        if self.busy:
            # 무료 티어는 분당 요청 수 제한이 있다. 겹쳐 부르지 않는다.
            return

        cands = self.frontiers()
        if not cands:
            self.get_logger().info('프론티어 후보가 없다 — 탐사가 끝났거나 맵이 비었다')
            return

        obs = self.observation(cands)
        self.busy = True
        try:
            ans = self.ask(obs)
        finally:
            self.busy = False

        if ans is None:
            # 재시도까지 다 실패. 휴리스틱 1순위로 간다 — 탐사가 멈추는 것보다 낫다.
            self.get_logger().warn('LLM 응답을 못 얻어 휴리스틱 1순위로 대체')
            chosen, cid, reason = cands[0], cands[0]['id'], '(휴리스틱 대체)'
        else:
            cid = ans['choice_id']
            chosen = next(c for c in cands if c['id'] == cid)
            reason = str(ans.get('reason', ''))[:200]
        self.get_logger().info(
            f"목표 ({chosen['x']}, {chosen['y']}) "
            f"[후보 {len(cands)}개 중 id={cid}] — {reason}")
        if self.send_goal(chosen['x'], chosen['y']):
            self.record(obs, cid, reason, chosen)


def main(args=None):
    rclpy.init(args=args)
    node = LlmGoalAssigner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
