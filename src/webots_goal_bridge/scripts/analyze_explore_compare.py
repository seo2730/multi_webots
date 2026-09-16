#!/usr/bin/env python3
"""탐사 전략 비교 분석 — 커버리지 곡선(cov) + 할당기 원시 로그(raw)를 run_tag 로 묶는다.

    python3 src/webots_goal_bridge/scripts/analyze_explore_compare.py
    python3 src/webots_goal_bridge/scripts/analyze_explore_compare.py --filter oneroom-in2

찾는 곳 (data/distill 기준):
  cov/{run_tag}.json            coverage_logger.py 결과
  raw/*.jsonl                   첫 줄 meta.run_tag 로 짝을 찾는다
  raw/legacy/*_cmp_{tag}_cov.json + *_cmp_{tag}.jsonl   기록 형식이 바뀌기 전 결과

🚨 도달 시각은 내부 BASE%(10%) 를 처음 넘은 순간부터 잰다. 측정기의 0초는 "지도와 시계를
   처음 받은 시각" 이라 시행마다 다르다 (실측 6.55% vs 1.22% 에서 시작). 스텝 사이는 선형 보간.
"""
import argparse, glob, json, os, re, statistics as st

THR = (25, 50, 75, 90)
BASE = 10.0


def t_at(cov, pct):
    prev = None
    for r in cov:
        if r['pct'] >= pct:
            if prev is None or r['pct'] == prev['pct']:
                return r['t']
            return prev['t'] + (pct - prev['pct']) / (r['pct'] - prev['pct']) * (r['t'] - prev['t'])
        prev = r
    return None


def dt(cov, pct):
    a, b = t_at(cov, BASE), t_at(cov, pct)
    return None if a is None or b is None else b - a


def read_jsonl(p):
    return [json.loads(l) for l in open(p, encoding='utf-8') if l.strip()]


def strategy_of(tag, meta):
    if meta.get('strategy'):
        return meta['strategy']
    for tok in re.split(r'[_-]', tag):
        if tok in ('distance', 'llm'):
            return tok
    return '?'


def trials(root):
    out = []
    raw_by_tag = {}
    for p in glob.glob(os.path.join(root, 'raw', '*.jsonl')):
        with open(p, encoding='utf-8') as fh:
            first = fh.readline()
        try:
            m = json.loads(first)
        except ValueError:
            continue
        if m.get('type') == 'meta' and m.get('run_tag'):
            raw_by_tag[m['run_tag']] = p
    for p in sorted(glob.glob(os.path.join(root, 'cov', '*.json'))):
        tag = os.path.splitext(os.path.basename(p))[0]
        out.append((tag, p, raw_by_tag.get(tag)))
    for p in sorted(glob.glob(os.path.join(root, 'raw', 'legacy', '*_cmp_*_cov.json'))):
        tag = re.sub(r'^.*_cmp_|_cov\.json$', '', os.path.basename(p))
        raw = p.replace('_cov.json', '.jsonl')
        out.append((f'legacy-{tag}', p, raw if os.path.exists(raw) else None))
    return out


def summarize(tag, cov_p, raw_p):
    cov = json.load(open(cov_p, encoding='utf-8'))
    rows = read_jsonl(raw_p) if raw_p else []
    meta = next((r for r in rows if r.get('type') == 'meta'), {})
    body = [r for r in rows if r.get('type') != 'meta']
    stats = body[-1]['stats'] if body else {}
    rounds = [r for r in body if 'result' in r]
    diff = better = 0
    for r in rounds:
        b = r.get('baseline')
        if not b or r['result'].get('fell_back'):
            continue
        A = {(a['robot'], a['frontier_id']) for a in r['result']['assignments']}
        B = {(a['robot'], a['frontier_id']) for a in b['assignments']}
        if A != B:
            diff += 1
            rs, bs = r['result'].get('achieved_separation'), b.get('achieved_separation')
            better += (rs is not None and bs is not None and rs > bs)
    calls = [c for r in rounds for c in ((r.get('teacher') or {}).get('calls') or [])]
    last = cov[-1] if cov else {}
    return {
        'tag': tag, 'strategy': strategy_of(tag, meta), 'model': meta.get('model'),
        'start_pct': cov[0]['pct'] if cov else None, 'final_pct': last.get('pct'),
        'final_t': last.get('t'), 'stop': last.get('stop_reason'),
        **{f't{p}': dt(cov, p) for p in THR},
        'new_targets': stats.get('new_targets'), 'giveups': stats.get('giveups'),
        'rounds': stats.get('rounds'), 'dist': last.get('dist', {}),
        'llm_calls': stats.get('llm_calls'), 'llm_sec': stats.get('llm_sec'),
        'late': stats.get('late'), 'fell_back': stats.get('fell_back'),
        'diff': diff, 'sep_better': better,
        'http_calls': len(calls), 'truncated': sum(c.get('finish_reason') == 'length' for c in calls),
    }


def label(tag):
    """표에 쓸 짧고 **구별되는** 이름. 긴 run_tag 를 앞에서 자르면 같은 묶음의 행이 전부 똑같아진다.

    {실험}_{묶음}_{커밋}_{전략}_{쌍} -> {묶음}:{전략}_{쌍}
    """
    parts = tag.split('_')
    if len(parts) >= 5:
        return f'{parts[1]}:{parts[-2]}_{parts[-1]}'
    return tag


def fmt(v, f='{:.0f}'):
    return '—' if v is None else f.format(v)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default='data/distill')
    ap.add_argument('--filter', default='', help='run_tag 에 이 문자열이 든 시행만')
    a = ap.parse_args(argv)
    res = [summarize(*t) for t in trials(a.root) if a.filter in t[0]]
    if not res:
        print('시행 없음 —', a.root); return
    print(f'도달 시각 = 내부 {BASE:.0f}% 통과 순간부터 잰 초 (선형 보간)\n')
    print(f"{'시행':28s} {'시작%':>5s} {'최종%':>5s} " + ' '.join(f"{'t'+str(p):>5s}" for p in THR)
          + f" {'새목표':>5s} {'포기':>4s} {'주기':>4s} {'LLM s/회':>8s} {'늦음':>4s} {'폴백':>4s} {'잘림':>6s}  종료")
    for r in res:
        avg = r['llm_sec'] / r['llm_calls'] if r['llm_calls'] else None
        trunc = f"{r['truncated']}/{r['http_calls']}" if r['http_calls'] else '—'
        print(f"{label(r['tag'])[:28]:28s} {fmt(r['start_pct'],'{:.1f}'):>5s} {fmt(r['final_pct'],'{:.1f}'):>5s} "
              + ' '.join(f"{fmt(r['t'+str(p)]):>5s}" for p in THR)
              + f" {fmt(r['new_targets']):>5s} {fmt(r['giveups']):>4s} {fmt(r['rounds']):>4s}"
              + f" {fmt(avg,'{:.1f}'):>8s} {fmt(r['late']):>4s} {fmt(r['fell_back']):>4s} {trunc:>6s}  {r['stop'] or ''}")
    print('\n전략별 (평균 ± 표준편차, n = 해당 지표가 있는 시행 수)')
    for s in sorted({r['strategy'] for r in res}):
        g = [r for r in res if r['strategy'] == s]
        def ms(k, f='{:.0f}'):
            v = [r[k] for r in g if r[k] is not None]
            if not v:
                return '—'
            return (f.format(st.mean(v)) + (' ± ' + f.format(st.stdev(v)) if len(v) > 1 else '')
                    + f' (n={len(v)})')
        print(f"  {s}: 시행 {len(g)}")
        print(f"    최종 {ms('final_pct','{:.1f}')}% | " + ' | '.join(f"t{p} {ms('t'+str(p))}s" for p in THR))
        print(f"    새 목표 {ms('new_targets','{:.1f}')} | 포기 {ms('giveups','{:.1f}')} | 주기 {ms('rounds','{:.1f}')}")
        if s == 'llm':
            print(f"    판단 갈림 {sum(r['diff'] for r in g)} (분리 우위 {sum(r['sep_better'] for r in g)}) | "
                  f"늦은 주기 {sum(r['late'] or 0 for r in g)} | 폴백 {sum(r['fell_back'] or 0 for r in g)} | "
                  f"잘린 호출 {sum(r['truncated'] for r in g)}/{sum(r['http_calls'] for r in g)}")


if __name__ == '__main__':
    main()
