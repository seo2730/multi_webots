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


def assign_by_distance(obs):
    """거리 합을 최소화하는 **정확한** 할당. 이것이 베이스라인이다.

    로봇 k대, 후보 n개면 조합이 n!/(n-k)! 뿐이라(2대·후보 12개면 132가지) 완전탐색이
    정확하고 즉시 끝난다. scipy 의 헝가리안을 쓸 수도 있지만, 그건 이미지에 우연히
    깔려 있는 패키지라 의존을 만들지 않는다.

    한 후보에 두 로봇이 가지 않도록 **서로 다른 후보**를 배정한다 — 겹치면 탐사가
    낭비되는데, 이건 거리만 봐도 알 수 있는 제약이라 베이스라인에도 넣는다.
    """
    robots = obs['robots']
    fronts = obs['frontiers']
    if not robots or not fronts:
        return {'assignments': [], 'total_cost': 0.0, 'method': 'distance'}

    k = min(len(robots), len(fronts))
    best, best_cost = None, float('inf')
    for combo in itertools.permutations(range(len(fronts)), k):
        # 🚨 도달 불가능한 배정은 후보에서 제외한다. 안 그러면 거리 합은 작아 보여도
        #    그 로봇의 Nav2 가 계획 자체를 못 만들어 라운드가 통째로 낭비된다.
        if any(not reachable(robots[ri], fronts[fi]) for ri, fi in enumerate(combo)):
            continue
        cost = sum(fronts[fi]['dist'][robots[ri]['id']]
                   for ri, fi in enumerate(combo))
        if cost < best_cost:
            best, best_cost = combo, cost

    if best is None:
        # 모든 조합이 막혔다. 각자 도달 가능한 것 중 가장 가까운 것으로 따로 준다.
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
                'method': 'distance(부분)'}

    return {
        'assignments': [{'robot': robots[ri]['id'], 'frontier_id': fronts[fi]['id']}
                        for ri, fi in enumerate(best)],
        'total_cost': round(best_cost, 2),
        'method': 'distance',
    }


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
