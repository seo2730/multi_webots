"""다중 로봇 프론티어 할당 — 관측 스키마와 거리 기준 베이스라인.

이 모듈은 **ROS 에 의존하지 않는다.** 격자와 숫자만 다루므로 시뮬을 띄우지 않고
검증할 수 있고, 학습 데이터를 오프라인으로 다시 만들 때도 그대로 쓴다.

왜 베이스라인을 먼저 만드는가
------------------------------
단일 로봇 실험에서 배운 것이 있다. 후보를 `dist / sqrt(cells)` 로 정렬해 주고
LLM 에게 고르라고 했더니 **항상 1순위를 골랐고 근거도 그 정렬 기준을 그대로 말했다.**
순서를 섞어도 같은 후보를 골랐으니 순서에 끌린 것은 아니지만, 결국 **정렬 한 줄이
하는 일을 느리고 불안정하게 재현**한 셈이다. 그걸 증류하면 정렬 함수를 배운 작은
모델이 나온다 — 아무 의미가 없다.

그래서 **거리 기준 최적 할당을 먼저 구현한다.** LLM 이 이걸 못 이기면 증류할 것이
없다는 뜻이고, 이기면 그 차이가 곧 증류 대상이다. 비교 없이 LLM 을 붙이면 같은
헛수고를 반복하게 된다.

무엇이 거리로 환원되지 않는가
------------------------------
로봇 2대가 후보 N개를 나눠 갖는 문제는, 비용이 거리뿐이면 **최적해가 즉시 나온다.**
LLM 이 기여할 여지가 없다. 기여하려면 거리에 담기지 않는 정보가 있어야 한다.

이 프로젝트에는 그게 실재한다 — **기체마다 잘하는 지형이 다르다.**

    드론  : 층별 매퍼가 순항 고도의 **수평 단면**을 그린다. 트인 공간을 빠르게
            훑지만 그 높이에 없는 낮은 장애물은 못 본다
    UGV   : 지상 라이다로 바닥 높이를 조밀하게 본다. 좁고 복잡한 곳에 강하다

그래서 관측에 `openness`(후보 주변이 얼마나 트였는가)를 넣는다. 거리와 독립인
축이고, 여기서 LLM 과 베이스라인의 판단이 갈릴 수 있다.
"""

import itertools
import json
import math

import numpy as np

UNKNOWN, FREE = -1, 0
OCCUPIED_MIN = 50

# 로봇 종류별 성격. 프롬프트에 그대로 실어 LLM 이 읽게 한다.
# 🚨 여기 적힌 것은 **문서화된 사실**이어야 한다. 근거 없는 수치를 넣으면 LLM 이
#    그걸 그대로 믿고 계획을 세운다.
ROBOT_PROFILE = {
    'ugv':   {'sensor': '지상 라이다(Velodyne VLP-16) — 바닥 높이를 조밀하게 본다',
              'strength': '좁고 복잡한 실내, 낮은 장애물 탐지'},
    'spot':  {'sensor': '뎁스카메라 5개 병합 스캔 — 360도, 유효거리 약 10 m',
              'strength': '거친 지형, 좁은 통로. 다만 운용 속도가 0.15 m/s 로 느리다'},
    'drone': {'sensor': '층별 매퍼 — 순항 고도의 수평 단면만 본다',
              'strength': '트인 공간을 빠르게 훑기. 그 고도에 없는 낮은 장애물은 못 본다'},
}


def openness(grid, origin_x, origin_y, res, x, y, radius_m=3.0):
    """후보 주변이 얼마나 트였는가 — 0.0(막힘) ~ 1.0(완전히 트임).

    반경 안에서 **장애물이 아닌 것으로 확인된 셀의 비율**이다. 미탐색은 분모에만
    넣는다 — "아직 모르는 곳"을 트였다고 보면 드론을 미탐색 구역으로 계속 보내게 된다.
    """
    h, w = grid.shape
    r = max(1, int(radius_m / res))
    cx = int((x - origin_x) / res)
    cy = int((y - origin_y) / res)
    x0, x1 = max(0, cx - r), min(w, cx + r + 1)
    y0, y1 = max(0, cy - r), min(h, cy + r + 1)
    if x0 >= x1 or y0 >= y1:
        return 0.0
    patch = grid[y0:y1, x0:x1]
    total = patch.size
    if total == 0:
        return 0.0
    return round(float((patch == FREE).sum()) / total, 3)


def build_observation(grid, origin_x, origin_y, res, robots, frontiers,
                      openness_radius=3.0):
    """LLM 과 베이스라인이 **똑같이** 받는 상태 표현.

    티처와 스튜던트, 그리고 베이스라인이 모두 같은 입력을 봐야 비교가 성립하고
    증류도 성립한다. 이미지가 아니라 텍스트인 이유도 그것이다.

    robots: [{'id','type','x','y'}, ... , 선택적으로 'nav_bounds': (x0,y0,x1,y1)]
    frontiers: extract_frontiers() 결과 [{'id','x','y','cells'}, ...]

    🚨 nav_bounds 는 **그 로봇이 실제로 경로를 만들 수 있는 범위**다. 드론은 층 매퍼가
       기체를 따라다니는 30x30 m 롤링 창만 갖고 있어서, 그 밖의 목표는 Nav2 가
       "The goal sent to the planner is off the global costmap" 으로 무조건 실패한다
       (실측). UGV 는 slam_toolbox 맵이 계속 자라므로 사실상 전역이다.
       로봇마다 도달 가능 영역이 다르다는 것은 할당에서 반드시 고려해야 하는 제약이다.
    """
    fr = []
    for f in frontiers:
        d = {r['id']: round(float(np.hypot(f['x'] - r['x'], f['y'] - r['y'])), 2)
             for r in robots}
        fr.append({'id': f['id'], 'x': f['x'], 'y': f['y'], 'cells': f['cells'],
                   'openness': openness(grid, origin_x, origin_y, res,
                                        f['x'], f['y'], openness_radius),
                   'dist': d})
    known = int((grid >= 0).sum())
    def _rob(r):
        d = {**{k: r[k] for k in ('id', 'type', 'x', 'y')},
             **ROBOT_PROFILE.get(r['type'], {})}
        if r.get('nav_bounds'):
            x0, y0, x1, y1 = r['nav_bounds']
            d['nav_bounds'] = [round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)]
        return d

    return {
        'robots': [_rob(r) for r in robots],
        'map': {'explored_pct': round(100.0 * known / max(1, grid.size), 1),
                'resolution': round(res, 3)},
        'frontiers': fr,
    }


def reachable(robot, frontier):
    """그 로봇이 이 후보까지 경로를 만들 수 있는가.

    nav_bounds 가 없으면 제한 없음으로 본다(UGV/Spot 처럼 맵이 계속 자라는 경우).
    """
    b = robot.get('nav_bounds')
    if not b:
        return True
    x0, y0, x1, y1 = b
    return x0 <= frontier['x'] <= x1 and y0 <= frontier['y'] <= y1


def _best_combo(robots, fronts, min_separation):
    """주어진 분리 거리에서 거리 합이 최소인 조합. 없으면 None."""
    k = min(len(robots), len(fronts))
    best, best_cost = None, float('inf')
    for combo in itertools.permutations(range(len(fronts)), k):
        # 🚨 도달 불가능한 배정은 제외한다. 안 그러면 거리 합은 작아 보여도
        #    그 로봇의 Nav2 가 계획 자체를 못 만들어 주기가 통째로 낭비된다.
        if any(not reachable(robots[ri], fronts[fi]) for ri, fi in enumerate(combo)):
            continue
        if min_separation > 0.0 and len(combo) > 1:
            # 배정된 후보끼리 너무 붙어 있으면 두 로봇이 같은 곳을 훑게 된다
            if any(math.hypot(fronts[a]['x'] - fronts[b]['x'],
                              fronts[a]['y'] - fronts[b]['y']) < min_separation
                   for a, b in itertools.combinations(combo, 2)):
                continue
        cost = sum(fronts[fi]['dist'][robots[ri]['id']]
                   for ri, fi in enumerate(combo))
        if cost < best_cost:
            best, best_cost = combo, cost
    return (best, best_cost) if best is not None else (None, None)


def assign_by_distance(obs, min_separation=0.0, relax_steps=4):
    """거리 합을 최소화하는 **정확한** 할당. 이것이 베이스라인이다.

    로봇 k대, 후보 n개면 조합이 n!/(n-k)! 뿐이라(2대·후보 12개면 132가지) 완전탐색이
    정확하고 즉시 끝난다. scipy 의 헝가리안을 쓸 수도 있지만, 그건 이미지에 우연히
    깔려 있는 패키지라 의존을 만들지 않는다.

    한 후보에 두 로봇이 가지 않도록 **서로 다른 후보**를 배정한다.

    🚨 `min_separation` 은 그것만으로는 부족해서 넣었다. 군집 버킷이 2 m 라 인접한
       두 버킷이 별개 후보가 되는데, "서로 다른 id" 만 보장하면 **1.5 m 떨어진 사실상
       같은 지점**에 두 로봇이 배정된다(실측: ugv1 (-31.4,38.4) / drone1 (-32.8,38.9)).
       분산 효과가 사라진다.

    🚨 **못 맞추면 단계적으로 낮춘다.** 예전에는 조건을 만족하는 조합이 없으면 제약을
       통째로 버리고 "각자 가장 가까운 후보"로 무너졌다. 그러면 두 로봇이 같은 구역에서
       부산하게 움직이며 1대만큼만 일한다 — 다중 로봇을 쓰는 이유가 사라진다.
       지금은 15 → 11 → 7 → 4 → 0 처럼 낮춰가며 **가능한 최대 분리**를 취한다.

       `achieved_separation` 에 실제로 확보한 값을, `degraded` 에 요구치에 못 미쳤는지를
       담는다. 이 둘이 LLM 과의 비교 지표다 — LLM 이 같은 상황에서 더 큰 분리를
       찾아내면 그 차이가 곧 증류할 가치다.
    """
    robots = obs['robots']
    fronts = obs['frontiers']
    if not robots or not fronts:
        return {'assignments': [], 'total_cost': 0.0, 'method': 'distance',
                'degraded': False, 'achieved_separation': None}

    # 요구치에서 0 까지 균등하게 낮춰 가며 처음 성공하는 값을 쓴다
    steps = [min_separation * (1.0 - i / max(1, relax_steps))
             for i in range(relax_steps)] + [0.0] if min_separation > 0.0 else [0.0]
    for sep in steps:
        best, cost = _best_combo(robots, fronts, sep)
        if best is None:
            continue
        out = [{'robot': robots[ri]['id'], 'frontier_id': fronts[fi]['id']}
               for ri, fi in enumerate(best)]
        actual = min((math.hypot(fronts[a]['x'] - fronts[b]['x'],
                                 fronts[a]['y'] - fronts[b]['y'])
                      for a, b in itertools.combinations(best, 2)), default=None)
        degraded = min_separation > 0.0 and sep < min_separation
        return {
            'assignments': out,
            'total_cost': round(cost, 2),
            'method': 'distance' if not degraded else f'distance(분리 {sep:.1f}m)',
            'degraded': degraded,
            'achieved_separation': round(actual, 1) if actual is not None else None,
            'reason': (f'요구 {min_separation} m 를 못 맞춰 {sep:.1f} m 로 낮춤'
                       if degraded else ''),
        }

    # 도달 가능한 조합이 하나도 없다 — 각자 갈 수 있는 곳으로 따로 준다.
    used, out = set(), []
    for r in robots:
        cand = [(f['dist'][r['id']], i) for i, f in enumerate(fronts)
                if i not in used and reachable(r, f)]
        if not cand:
            continue
        _, i = min(cand)
        used.add(i)
        out.append({'robot': r['id'], 'frontier_id': fronts[i]['id']})
    return {'assignments': out, 'total_cost': assignment_cost(obs, out) or 0.0,
            'method': 'distance(부분)', 'degraded': True,
            'achieved_separation': None,
            'reason': '도달 가능한 조합이 없어 각자 가장 가까운 후보로'}


def assignment_cost(obs, assignments):
    """어떤 할당이든 같은 잣대로 재는 비용(거리 합). LLM 결과를 베이스라인과 비교할 때 쓴다."""
    by_id = {f['id']: f for f in obs['frontiers']}
    total = 0.0
    for a in assignments:
        f = by_id.get(a['frontier_id'])
        if f is None:
            return None                     # 존재하지 않는 후보를 고른 할당
        total += f['dist'][a['robot']]
    return round(total, 2)


def validate(obs, assignments):
    """LLM 이 낸 할당이 실행 가능한가. 실패 사유를 문자열로 돌려준다(정상이면 None).

    🚨 모델을 믿되 검증한다. 실측에서 빈 JSON·범위 밖 id 가 드물지 않았다.
    """
    rid = {r['id'] for r in obs['robots']}
    fid = {f['id'] for f in obs['frontiers']}
    seen_r, seen_f = set(), set()
    for a in assignments:
        if not isinstance(a, dict) or 'robot' not in a or 'frontier_id' not in a:
            return f'형식 오류: {a}'
        if a['robot'] not in rid:
            return f"모르는 로봇: {a['robot']}"
        if a['frontier_id'] not in fid:
            return f"모르는 후보 id: {a['frontier_id']}"
        if a['robot'] in seen_r:
            return f"로봇 중복 배정: {a['robot']}"
        if a['frontier_id'] in seen_f:
            return f"후보 중복 배정: {a['frontier_id']}"
        seen_r.add(a['robot']); seen_f.add(a['frontier_id'])
    return None


# --------------------------------------------------------------------- LLM 전략

ALLOC_PROMPT = """\
너는 실내를 탐사하는 다중 로봇 편대의 **임무 계획기**다. 전역 지도에서 뽑은 탐사
후보(프론티어)를 로봇에게 나눠 배정하는 것이 네 일이다.

{obs}

읽는 법
- robots[].sensor / strength: 그 기체가 무엇을 잘 보는가. **거리만으로 정하지 마라.**
- robots[].nav_bounds: 그 로봇이 실제로 경로를 만들 수 있는 범위 [x0,y0,x1,y1].
  이 밖의 후보를 주면 그 로봇은 한 주기를 통째로 낭비한다. 없으면 제한이 없다는 뜻.
- frontiers[].cells: 그 후보에 속한 경계 셀 수. 클수록 넓은 미탐색 영역이다.
- frontiers[].openness: 후보 주변이 얼마나 트였는가(0~1). 높으면 트인 공간이다.
- frontiers[].dist: 로봇별 직선 거리(m).

규칙
1. 한 로봇에 후보 하나. 두 로봇이 같은 후보로 가지 않는다.
2. 배정된 후보끼리 **{min_sep} m 이상 떨어뜨려라.** 가까우면 두 대가 같은 구역을
   중복 탐사해 1대만큼만 일한다. 도저히 못 맞추면 가능한 한 멀리 떨어뜨린다.
3. 좌표를 새로 만들지 마라. 반드시 frontiers 목록의 id 중에서 고른다.
4. nav_bounds 밖의 후보는 그 로봇에게 배정하지 마라.

아래 JSON 형식으로만 답한다:
{{"assignments": [{{"robot": "<id>", "frontier_id": <정수>}}, ...], "reason": "<한 문장>"}}
"""


def _separation_of(obs, assignments):
    """배정된 후보들 사이의 최소 거리. 1대 이하면 None."""
    by_id = {f['id']: f for f in obs['frontiers']}
    pts = [by_id[a['frontier_id']] for a in assignments if a['frontier_id'] in by_id]
    if len(pts) < 2:
        return None
    return round(min(math.hypot(a['x'] - b['x'], a['y'] - b['y'])
                     for a, b in itertools.combinations(pts, 2)), 1)


def unreachable_in(obs, assignments):
    """nav_bounds 를 벗어난 배정이 있으면 그 사유를 돌려준다(없으면 None).

    validate() 는 스키마만 본다. 도달 가능성은 별도다 — 범위 밖 목표는 Nav2 가
    "goal is off the global costmap" 으로 거절해 그 주기가 통째로 낭비된다.
    """
    by_r = {r['id']: r for r in obs['robots']}
    by_f = {f['id']: f for f in obs['frontiers']}
    for a in assignments:
        r, f = by_r.get(a['robot']), by_f.get(a['frontier_id'])
        if r is None or f is None:
            continue
        if not reachable(r, f):
            return (f"{a['robot']} 는 후보 {a['frontier_id']}"
                    f"({f['x']:.1f},{f['y']:.1f}) 에 경로를 못 만든다")
    return None


def assign_by_llm(obs, chat, min_separation=0.0, retries=3, on_retry=None):
    """LLM 에게 배정을 맡긴다. **베이스라인과 같은 관측·같은 반환 스키마**를 쓴다.

    `chat` 은 `prompt -> 응답 문자열` 콜러블이다. 이 모듈을 ROS 와 openai 양쪽에서
    떼어 놓기 위한 이음매다 — 덕분에 가짜 chat 으로 단위 검증이 된다.

    🚨 **실패하면 거리 베이스라인으로 물러난다.** 실측에서 스키마 위반이 드물지
       않았다(빈 JSON, 범위 밖 id, 응답 잘림). 탐사가 통째로 멈추는 것보다
       베이스라인으로라도 도는 편이 낫고, 무엇보다 **폴백 횟수 자체가 비교
       지표**다 — LLM 이 얼마나 자주 쓸 수 없는 답을 내는지가 증류의 난이도다.

    반환에 `llm_failures`(이번 주기에 버린 응답 수)와 `fell_back` 을 담는다.
    """
    prompt = ALLOC_PROMPT.format(
        obs=json.dumps(obs, ensure_ascii=False, indent=2),
        min_sep=min_separation)

    failures = []
    for attempt in range(max(1, retries)):
        try:
            ans = json.loads(chat(prompt))
            got = ans.get('assignments')
            if not isinstance(got, list) or not got:
                raise ValueError(f'assignments 가 비었다: {str(ans)[:80]}')
            bad = validate(obs, got) or unreachable_in(obs, got)
            if bad:
                raise ValueError(bad)
        except Exception as exc:                      # noqa: BLE001
            failures.append(f'{type(exc).__name__}: {exc}')
            if on_retry is not None:
                on_retry(attempt + 1, retries, failures[-1])
            continue

        sep = _separation_of(obs, got)
        return {
            'assignments': [{'robot': a['robot'], 'frontier_id': a['frontier_id']}
                            for a in got],
            'total_cost': assignment_cost(obs, got),
            'method': 'llm',
            'degraded': bool(min_separation > 0.0 and sep is not None
                             and sep < min_separation),
            'achieved_separation': sep,
            'reason': str(ans.get('reason', ''))[:200],
            'llm_failures': len(failures),
            'fell_back': False,
        }

    out = assign_by_distance(obs, min_separation=min_separation)
    out['method'] = 'llm(폴백:distance)'
    out['llm_failures'] = len(failures)
    out['fell_back'] = True
    out['reason'] = f'LLM {retries}회 실패 — {failures[-1] if failures else "?"}'
    return out
