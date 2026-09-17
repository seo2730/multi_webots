#!/usr/bin/env python3
"""탐사 할당·증류 기록·내보내기 검증 — ROS 없이, 시뮬 없이 돈다.

    python3 src/webots_goal_bridge/test/test_allocator_offline.py

노드(`frontier_allocator_node`)는 import 하면 rclpy 를 끌어오므로, 검사할 메서드만
**소스에서 떼어내** 껍데기 클래스에 붙여 돌린다. 덕분에 호스트에서도 검증된다.

🚨 이 파일은 저장소에 둔다. 예전에는 임시 폴더에 뒀다가 세 번 날아갔다.
"""
import ast
import json
import math
import os
import sys
import tempfile
import textwrap
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)                      # src/webots_goal_bridge
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, 'scripts'))
SRC = os.path.join(PKG, 'webots_goal_bridge')

FAILS = []


def check(name, ok, detail=''):
    print(f'{"OK  " if ok else "FAIL"}  {name}  {detail}')
    if not ok:
        FAILS.append(name)


def grab(path, name):
    src = open(path, encoding='utf-8').read()
    fn = next(n for n in ast.walk(ast.parse(src))
              if isinstance(n, ast.FunctionDef) and n.name == name)
    return ast.get_source_segment(src, fn)


def shell(names, path, ns=None):
    """지정한 함수들을 담은 껍데기 클래스를 만든다."""
    body = '\n'.join(textwrap.indent(grab(path, n), '    ') for n in names)
    env = dict(ns or {})
    exec('class N:\n' + body, env)
    return env['N']


class Log:
    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass

    def info(self, *a, **k):
        pass


# ---------------------------------------------------------------- 프론티어 범위
def test_extract_frontiers():
    ns = {'np': np, 'FREE': 0, 'UNKNOWN': -1}
    exec(grab(os.path.join(SRC, 'llm_goal_assigner.py'), 'extract_frontiers'), ns)
    ef = ns['extract_frontiers']
    g = np.full((100, 100), -1, dtype=np.int16)      # 원점 -50, 셀 1 m
    g[20:80, 20:80] = 0                              # 내부 [-30,30]²
    g[45:55, 5:15] = 0                               # 로봇 곁 마당 조각
    rx, ry, IN = -40.0, 0.0, (-30.0, -30.0, 30.0, 30.0)
    inside = lambda c: IN[0] <= c['x'] <= IN[2] and IN[1] <= c['y'] <= IN[3]
    kw = dict(min_cells=2, bucket=4.0, max_n=4)
    no_b = ef(g, -50.0, -50.0, 1.0, rx, ry, **kw)
    with_b = ef(g, -50.0, -50.0, 1.0, rx, ry, bounds=IN, **kw)
    check('범위 없으면 가까운 마당 후보가 상위를 차지', any(not inside(c) for c in no_b))
    check('범위를 주면 전부 내부', with_b and all(inside(c) for c in with_b))
    check('자르기 **전에** 걸러 max_n 을 채운다', len(with_b) == 4, f'{len(with_b)}/4')
    check('범위 안에 없으면 빈 목록',
          ef(g, -50.0, -50.0, 1.0, rx, ry, bounds=(40., 40., 45., 45.), **kw) == [])
    check('bounds 기본값은 기존 동작 그대로', ef(g, -50.0, -50.0, 1.0, rx, ry, **kw) == no_b)


# ------------------------------------------------------------------ 선택 파라미터
def test_opt_and_bounds():
    # 🚨 둘은 **모듈 수준** 함수다. 클래스에 메서드로 붙이면 parse_bounds 안에서
    #    opt_str 이름이 안 보인다 (클래스 본문은 스코프를 만들지 않는다).
    ns = {}
    node = os.path.join(SRC, 'frontier_allocator_node.py')
    exec(grab(node, 'opt_str'), ns)
    exec(grab(node, 'parse_bounds'), ns)
    opt, pb = ns['opt_str'], ns['parse_bounds']
    for v, want in (('', ''), ('none', ''), (' None ', ''), ('null', ''), (None, ''),
                    ('/data/x', '/data/x')):
        check(f'opt_str({v!r})', opt(v) == want)
    check("parse_bounds('')", pb('') is None)
    check("parse_bounds('none') — compose 기본값", pb('none') is None)
    check("parse_bounds('-37,-37,37,37')", pb('-37,-37,37,37') == (-37., -37., 37., 37.))
    check("parse_bounds('[1,2,3,4]')", pb('[1,2,3,4]') == (1., 2., 3., 4.))
    for bad in ('5,0,1,1', '1,2,3'):
        try:
            pb(bad)
            check(f'parse_bounds({bad!r}) 거절', False)
        except ValueError:
            check(f'parse_bounds({bad!r}) 거절', True)


# ------------------------------------------------------------- 경유점 / 목표 집계
def test_clamp_and_targets():
    node = os.path.join(SRC, 'frontier_allocator_node.py')
    N = shell(['clamp', '_pick_target', '_explored', '_stale_seconds', '_update_progress'],
              node, {'math': math, 'np': np})

    class P:
        def __init__(self, x, y):
            self.position = types.SimpleNamespace(x=x, y=y)

    n = N()
    n.bounds = {'u': (-50.11, -60.0, -3.0, 45.0)}
    n.pose = {'u': P(-48.11, -23.42)}
    n.min_goal_dist = 4.0
    gx, gy = n.clamp('u', -49.6, 41.0)          # 실측 교착 사례
    d = math.hypot(gx + 48.11, gy + 23.42)
    check('교착 사례에서 축별로 잘라 전진한다', d >= 4.0 and gy > 30.0, f'{d:.1f} m')
    n.bounds = {'u': (-50., -50., 50., 50.)}
    n.pose = {'u': P(0.0, 0.0)}
    check('범위 안 목표는 그대로', n.clamp('u', 10.0, 20.0) == (10.0, 20.0))
    check('범위 밖은 여백만큼 안쪽으로', n.clamp('u', 80.0, 10.0) == (48.0, 10.0))
    n.bounds = {'u': (-50., -50., -46., -46.)}
    n.pose = {'u': P(-48.0, -48.0)}
    gx, gy = n.clamp('u', 40.0, 40.0)
    check('구석에 박혀도 최소 전진거리 확보',
          round(math.hypot(gx + 48.0, gy + 48.0), 2) >= 3.99)

    n = N()
    n.target, n.stuck, n.giveup, n.min_goal_dist = {}, {}, 3, 4.0
    n.giveup_sec, n.no_prog_since = 180.0, {}
    n.stats = {'new_targets': 0, 'giveups': 0}
    n._round_events = []
    n.get_logger = lambda: Log()
    clock = {'t': 0.0}
    n._sim_now = lambda: clock['t']
    A, B = {'x': 20.0, 'y': 0.0}, {'x': 0.0, 'y': 20.0}
    r0 = {'x': 0.0, 'y': 0.0}
    n._pick_target('u', r0, [A, B], {'u': A})
    check('첫 배정은 새 목표 1', n.stats['new_targets'] == 1)
    n._pick_target('u', r0, [A, B], {'u': B})
    check('멀고 아직 프론티어면 유지', n.stats['new_targets'] == 1 and n.target['u'] == (20.0, 0.0))
    n.stuck['u'] = 3
    n._pick_target('u', r0, [A, B], {'u': B})
    check('주기 수로 포기 — giveups+1 · 새 목표+1 · 이벤트에 by=rounds',
          n.stats == {'new_targets': 2, 'giveups': 1}
          and n._round_events[0]['by'] == 'rounds' and n._round_events[0]['robot'] == 'u')
    n._pick_target('u', {'x': 0.0, 'y': 19.0}, [A, B], {'u': B})
    check('도달 후 같은 좌표는 세지 않는다', n.stats['new_targets'] == 2)

    # --- 시간 기준 포기: 주기가 느려도 같은 실제 시간에 포기한다 ---
    m = N()
    m.target, m.stuck, m.giveup, m.min_goal_dist = {'u': (20.0, 0.0)}, {}, 3, 4.0
    m.giveup_sec, m.no_prog_since = 180.0, {}
    m.stats = {'new_targets': 0, 'giveups': 0}
    m._round_events, m.last_pos, m.progress_min = [], {}, 1.0
    m.get_logger = lambda: Log()
    clk = {'t': 0.0}
    m._sim_now = lambda: clk['t']
    m._update_progress('u', {'x': 0.0, 'y': 0.0})            # 기준점
    clk['t'] = 150.0
    m._update_progress('u', {'x': 0.2, 'y': 0.0})            # 거의 안 움직임
    m._round_events = []
    m._pick_target('u', {'x': 0.0, 'y': 0.0}, [A], {'u': A})
    check('150초 정체(주기 2회)는 아직 유지', m.stats['giveups'] == 0 and m._round_events == [])
    clk['t'] = 400.0
    m._update_progress('u', {'x': 0.3, 'y': 0.0})
    m._pick_target('u', {'x': 0.0, 'y': 0.0}, [A], {'u': A})
    check('180초를 넘기면 주기 수와 무관하게 포기 (by=time)',
          m.stats['giveups'] == 1 and m._round_events[-1]['by'] == 'time'
          and m._round_events[-1]['stale_s'] >= 180.0, str(m._round_events[-1]))
    check('포기 뒤 정체 시계가 초기화된다', m._stale_seconds('u') == 0.0)

    n = N()
    info = types.SimpleNamespace(resolution=1.0, width=100, height=100,
                                 origin=types.SimpleNamespace(
                                     position=types.SimpleNamespace(x=-50.0, y=-50.0)))
    grid = np.full((100, 100), -1, dtype=np.int16)
    grid[40:60, 40:60] = 0          # 월드 [-10,10]²
    grid[0:5, 0:5] = 0              # 범위 밖
    n.explore_bounds = (-20., -20., 20., 20.)
    check('탐색 셀은 범위 안만 센다', n._explored(grid, info) == {'known': 400, 'total': 1600})
    n.explore_bounds = None
    check('범위 없으면 지도 전체', n._explored(grid, info) == {'known': 425, 'total': 10000})


# --------------------------------------------------------------------- 교사 호출
def test_chat_truncation():
    node = os.path.join(SRC, 'frontier_allocator_node.py')
    import time as _t
    N = shell(['_chat'], node, {'time': _t})

    def resp(content, finish, reasoning=None):
        msg = types.SimpleNamespace(content=content, reasoning_content=reasoning)
        return types.SimpleNamespace(
            choices=[types.SimpleNamespace(message=msg, finish_reason=finish)],
            usage=types.SimpleNamespace(prompt_tokens=900, completion_tokens=1024))

    sent_kwargs = []

    def node_with(seq, thinking=False):
        n = N()
        n._calls, budgets = [], []
        n.llm_thinking = thinking

        def create(**kw):
            budgets.append(kw['max_tokens'])
            sent_kwargs.append(kw)
            return seq.pop(0)

        n.client = types.SimpleNamespace(
            chat=types.SimpleNamespace(completions=types.SimpleNamespace(create=create)))
        n.get_logger = lambda: Log()
        vals = {'system_prompt': 'detailed thinking off', 'max_tokens': 2048, 'model': 'm'}
        n.get_parameter = lambda k: types.SimpleNamespace(value=vals[k])
        return n, budgets

    n, b = node_with([resp('{"a": 1}', 'stop')])
    check('정상 응답은 그대로', n._chat('p') == '{"a": 1}' and b == [2048])
    n, b = node_with([resp('Okay, the user is asking...', 'length', '추론'),
                      resp('{"a": 1}', 'stop', '추론2')])
    check('추론 텍스트가 본문에 든 채 잘려도 두 배로 재시도', n._chat('p') == '{"a": 1}' and b == [2048, 4096])
    check('잘린 호출도 원문·추론·토큰을 남긴다',
          [c['finish_reason'] for c in n._calls] == ['length', 'stop']
          and n._calls[1]['reasoning'] == '추론2' and n._calls[0]['prompt_tokens'] == 900)
    n, _ = node_with([resp('', 'length'), resp('', 'length')])
    try:
        n._chat('p')
        check('두 번 다 잘리면 예외', False)
    except ValueError as e:
        check('두 번 다 잘리면 예외', '4096' in str(e))
    n, b = node_with([resp('', 'stop')])
    try:
        n._chat('p')
        check('잘림이 아닌 빈 본문은 즉시 예외', False)
    except ValueError:
        check('잘림이 아닌 빈 본문은 즉시 예외', b == [2048])

    # 사고 끄기가 실제로 요청에 실리는가 — 실측에서 17.9초/잘림 vs 2.5초/정상을 가른 지점
    sent_kwargs.clear()
    n, _ = node_with([resp('{"a": 1}', 'stop')], thinking=False)
    n._chat('p')
    check('사고 끔 → chat_template_kwargs 로 진짜 끈다',
          sent_kwargs[-1].get('extra_body') == {'chat_template_kwargs': {'thinking': False}},
          str(sent_kwargs[-1].get('extra_body')))
    sent_kwargs.clear()
    n, _ = node_with([resp('{"a": 1}', 'stop')], thinking=True)
    n._chat('p')
    check('사고 켬 → 추가 인자 없음 (지원 안 하는 제공자용)',
          sent_kwargs[-1].get('extra_body') is None)


# ------------------------------------------------------------------ 기록 / 내보내기
def _obs():
    from webots_goal_bridge.frontier_allocator import build_observation
    g = np.zeros((40, 40), dtype=np.int16)
    robots = [{'id': 'ugv1', 'type': 'ugv', 'x': 0.0, 'y': 0.0},
              {'id': 'drone1', 'type': 'drone', 'x': 2.0, 'y': 0.0}]
    fr = [{'id': 0, 'x': 8.0, 'y': 8.0, 'cells': 40},
          {'id': 1, 'x': -8.0, 'y': -8.0, 'cells': 30}]
    return build_observation(g, -10.0, -10.0, 0.5, robots, fr)


def test_assign_by_llm():
    from webots_goal_bridge.frontier_allocator import (PROMPT_SHA, PROMPT_VERSION,
                                                       assign_by_llm, build_alloc_prompt)
    obs = _obs()
    good = json.dumps({'assignments': [{'robot': 'ugv1', 'frontier_id': 0},
                                       {'robot': 'drone1', 'frontier_id': 1}], 'reason': 'r'})
    seq, seen, att = [RuntimeError('503'), 'not json', good], [], []

    def chat(p):
        seen.append(p)
        x = seq.pop(0)
        if isinstance(x, Exception):
            raise x
        return x

    res = assign_by_llm(obs, chat, min_separation=10.0, retries=3,
                        on_attempt=lambda i, raw, ok, err: att.append((i, raw, ok)))
    check('기록용 프롬프트 == 실제 보낸 프롬프트',
          all(p == build_alloc_prompt(obs, 10.0) for p in seen))
    check('시도 콜백: 예외는 raw=None, 실패 원문 보존, 성공도 보고',
          att[0][1] is None and att[1][1] == 'not json' and att[2][2] is True)
    check('유효 응답은 폴백 아님', not res['fell_back'] and res['method'] == 'llm')
    check('프롬프트 판이 박혀 있다', PROMPT_VERSION.startswith('alloc-v') and len(PROMPT_SHA) == 12,
          f'{PROMPT_VERSION} {PROMPT_SHA}')
    bad = assign_by_llm(obs, lambda p: '{}', min_separation=10.0, retries=2)
    check('스키마 위반이면 베이스라인으로 폴백', bad['fell_back'] and bad['assignments'])
    far = assign_by_llm(obs, lambda p: json.dumps({'assignments': [
        {'robot': 'drone1', 'frontier_id': 0}]}), min_separation=0.0, retries=1)
    check('도달 가능 배정은 통과', not far['fell_back'])


def test_record_and_export():
    import export_distill as ex
    from webots_goal_bridge.frontier_allocator import build_alloc_prompt
    node = os.path.join(SRC, 'frontier_allocator_node.py')
    N = shell(['_append'], node, {'json': json})
    tmp = tempfile.mkdtemp()
    n = N()
    n.dataset_path = os.path.join(tmp, 'run.jsonl')
    n._meta_written = False
    n._meta = lambda: {'type': 'meta', 'run_id': 'r1'}
    n.get_logger = lambda: Log()
    n._append({'type': 'round', 'k': 1})
    n._append({'type': 'round', 'k': 2})
    kinds = [json.loads(l)['type'] for l in open(n.dataset_path)]
    check('meta 는 첫 줄에 한 번만', kinds == ['meta', 'round', 'round'])

    obs = _obs()
    llm = {'assignments': [{'robot': 'ugv1', 'frontier_id': 0},
                           {'robot': 'drone1', 'frontier_id': 1}],
           'method': 'llm', 'fell_back': False, 'reason': '반대편',
           'achieved_separation': 22.6, 'total_cost': 30.0}
    base = {'assignments': [{'robot': 'ugv1', 'frontier_id': 1},
                            {'robot': 'drone1', 'frontier_id': 0}],
            'method': 'distance', 'achieved_separation': 22.6, 'total_cost': 20.0}
    raw = os.path.join(tmp, 'raw')
    os.makedirs(raw)
    with open(os.path.join(raw, 'new.jsonl'), 'w', encoding='utf-8') as f:
        f.write(json.dumps({'type': 'meta', 'run_id': 'R', 'strategy': 'llm',
                            'min_separation': 15.0}) + '\n')
        for i, known in enumerate((100, 400), start=1):
            # 🚨 주기마다 관측이 달라야 한다. 같은 프롬프트·같은 답이면 내보내기가
            #    (제대로) 중복으로 지운다 — 그러면 이 검사가 그 동작에 걸려 넘어진다.
            o = json.loads(json.dumps(obs))
            o['robots'][0]['x'] = float(i)
            f.write(json.dumps({
                'type': 'round', 't_sim': 60.0 * i, 'round': i, 'observation': o,
                'result': llm, 'baseline': base, 'min_separation': 15.0,
                'prompt': build_alloc_prompt(o, 15.0),
                'teacher': {'latency_s': 12.0, 'calls': [
                    {'finish_reason': 'length', 'content': 'Okay...', 'reasoning': '잘림'},
                    {'finish_reason': 'stop', 'content': '{}', 'reasoning': '진짜 추론'}]},
                'explored': {'known': known, 'total': 1000}, 'events': [], 'stats': {}}) + '\n')
        f.write(json.dumps({'type': 'round', 't_sim': 180.0, 'observation': obs,
                            'result': {**base, 'method': 'llm(폴백:distance)', 'fell_back': True},
                            'stats': {}}) + '\n')
    out = os.path.join(tmp, 'out')
    s = ex.export([raw], out, val_pct=0, include_reasoning=True)
    rows = [json.loads(l) for l in open(os.path.join(out, 'sft_train.jsonl'))]
    check('폴백은 빼고 유효 답만 내보낸다', len(rows) == 2, str(len(rows)))
    check('결과 = 다음 판단까지 분당 탐색 증가',
          rows[0]['meta']['outcome']['gain_per_min'] == 300.0)
    check('마지막 판단은 결과 없음', rows[-1]['meta']['outcome']['last_round'] is True)
    check('추론은 잘리지 않은 호출에서 <think> 로',
          rows[0]['messages'][-1]['content'].startswith('<think>\n진짜 추론\n</think>\n'))
    dis = [json.loads(l) for l in open(os.path.join(out, 'disagreements.jsonl'))]
    check('갈림에는 선호 라벨을 붙이지 않는다',
          len(dis) == 2 and all('chosen' not in d for d in dis))
    check('폴백 집계', s['runs'][0]['fell_back'] == 1)
    s2 = ex.export([raw, raw], out + '2', val_pct=0)
    check('같은 프롬프트·답은 중복 제거', s2['totals']['dup'] >= 2, str(s2['totals']))
    s3 = ex.export([raw], out + '3', val_pct=100)
    check('val 100% 면 전부 val (실행 단위 분할)',
          s3['totals']['sft_train'] == 0 and s3['totals']['sft_val'] == 2)


def test_async_llm():
    """비동기 계획: 준비되기 전엔 None(기존 목표 유지), 준비되면 그 관측째로 돌려준다."""
    from concurrent.futures import Future
    node = os.path.join(SRC, 'frontier_allocator_node.py')
    N = shell(['_async_llm'], node)
    n = N()
    n.get_logger = lambda: Log()
    n._pending = None
    futures = []

    class Pool:
        def submit(self, fn, *a):
            f = Future()
            futures.append((f, a))
            return f

    n._pool = Pool()
    n._llm_call = lambda o, f: None      # 제출 시점에 속성으로 잡힌다 (여기선 안 불린다)
    obs, fr = {'tag': 'obs1'}, [{'id': 0}]
    check('첫 주기: 제출하고 None (로봇은 기존 목표 유지)',
          n._async_llm(obs, fr) is None and len(futures) == 1)
    check('생각 중에는 겹쳐 묻지 않는다',
          n._async_llm({'tag': 'obs2'}, fr) is None and len(futures) == 1)
    f, args = futures[0]
    check('제출된 것은 그때의 관측', args == (obs, fr))
    f.set_result({'obs': obs, 'fr': fr, 'result': {'method': 'llm'}, 'teacher': {}})
    snap = n._async_llm({'tag': 'obs3'}, fr)
    check('준비되면 **제출 당시 관측**과 함께 돌려준다', snap is not None and snap['obs'] is obs)
    check('받아간 뒤 다음 주기에 새로 제출', n._async_llm({'tag': 'obs4'}, fr) is None
          and len(futures) == 2)
    futures[1][0].set_exception(RuntimeError('boom'))
    check('작업이 예외로 끝나면 None (탐사는 계속)', n._async_llm(obs, fr) is None)


def test_resend_while_pending():
    """교사를 기다리는 주기: 새 배정은 없지만 기존 목표는 다시 나가고 진행도 추적된다."""
    node = os.path.join(SRC, 'frontier_allocator_node.py')
    N = shell(['_resend_targets', '_update_progress'], node, {'math': math})
    n = N()
    n.target = {'ugv1': (10.0, 0.0)}          # drone1 은 아직 목표가 없다
    n.last_pos, n.stuck, n.progress_min = {}, {}, 1.0
    n.no_prog_since = {}
    clock = {'t': 0.0}
    n._sim_now = lambda: clock['t']
    out = []
    n.send_goal = lambda ns, x, y: (out.append((ns, x, y)) or (x, y))
    robots = [{'id': 'ugv1', 'x': 0.0, 'y': 0.0}, {'id': 'drone1', 'x': 5.0, 'y': 5.0}]
    sent = n._resend_targets(robots)
    check('목표가 있는 로봇에게만 다시 보낸다', list(sent) == ['ugv1'] and out == [('ugv1', 10.0, 0.0)])
    n._resend_targets([{'id': 'ugv1', 'x': 0.1, 'y': 0.0}])
    check('기다리는 동안에도 진전 없음이 쌓인다 (포기 조건이 멈추지 않는다)',
          n.stuck['ugv1'] == 1, str(n.stuck))
    n._resend_targets([{'id': 'ugv1', 'x': 9.0, 'y': 0.0}])
    check('움직였으면 초기화', n.stuck['ugv1'] == 0)


def test_local_retarget():
    """답 사이 재조준: 교사가 나눈 구역은 지키고, 그 안에서 가까운 목표로 갱신한다."""
    node = os.path.join(SRC, 'frontier_allocator_node.py')
    N = shell(['_local_retarget', '_owner_of', '_region_fresh'], node, {'math': math})
    n = N()
    n.anchor = {'ugv1': (20.0, 0.0), 'drone1': (-20.0, 0.0)}
    n.target = {'ugv1': (20.0, 0.0), 'drone1': (-20.0, 0.0)}
    n.min_goal_dist = 4.0
    n.stats = {'new_targets': 0, 'retargets': 0}
    n.no_prog_since = {}
    n._sim_now = lambda: 100.0
    fr = [{'x': 12.0, 'y': 0.0}, {'x': 18.0, 'y': 0.0},      # ugv1 구역
          {'x': -12.0, 'y': 0.0}, {'x': -25.0, 'y': 0.0},    # drone1 구역
          {'x': 1.0, 'y': 0.0}]                              # ugv1 곁 — 너무 가깝다
    robots = [{'id': 'ugv1', 'x': 0.0, 'y': 0.0}, {'id': 'drone1', 'x': -30.0, 'y': 0.0}]
    changed = n._local_retarget(robots, fr)
    check('두 로봇 모두 재조준', sorted(changed) == ['drone1', 'ugv1'], str(changed))
    check('ugv1 은 자기 구역에서 가장 가까운 12.0 (1.0 은 min_goal_dist 미만이라 제외)',
          n.target['ugv1'] == (12.0, 0.0), str(n.target['ugv1']))
    check('drone1 은 자기 구역에서 가장 가까운 -25.0', n.target['drone1'] == (-25.0, 0.0),
          str(n.target['drone1']))
    check('남의 구역은 안 가져간다 (-12 는 drone1 구역이라 ugv1 이 못 잡는다)',
          n.target['ugv1'][0] > 0)
    check('갱신은 새 탐사점으로 센다', n.stats == {'new_targets': 2, 'retargets': 2})
    check('정체 시계도 새로 시작', n.no_prog_since == {'ugv1': 100.0, 'drone1': 100.0})

    same = n._local_retarget(robots, fr)
    check('같은 목표면 다시 세지 않는다', same == [] and n.stats['new_targets'] == 2)

    n2 = N()
    n2.anchor, n2.target, n2.min_goal_dist = {}, {}, 4.0
    n2.stats, n2.no_prog_since = {'new_targets': 0, 'retargets': 0}, {}
    n2._sim_now = lambda: 0.0
    check('앵커가 없으면(첫 답 전) 아무것도 안 한다', n2._local_retarget(robots, fr) == [])
    n2.anchor = {'ugv1': (20.0, 0.0)}
    check('구역에 후보가 없으면 그대로',
          n2._local_retarget([{'id': 'ugv1', 'x': 19.0, 'y': 0.0}],
                             [{'x': 20.0, 'y': 0.0}]) == [])


def main():
    for fn in (test_extract_frontiers, test_opt_and_bounds, test_clamp_and_targets,
               test_chat_truncation, test_assign_by_llm, test_async_llm,
               test_resend_while_pending, test_local_retarget,
               test_record_and_export):
        print(f'\n--- {fn.__name__} ---')
        fn()
    print('\n전부 통과' if not FAILS else f'\n실패 {len(FAILS)}건: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
