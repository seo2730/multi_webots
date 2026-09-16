#!/usr/bin/env python3
"""탐사 할당 원시 로그 -> 지식 증류용 데이터셋.

ROS 가 필요 없다. 호스트든 컨테이너든 파이썬 3 + numpy 만 있으면 된다
(numpy 는 프롬프트를 다시 만들 때 할당 모듈이 끌어온다).

    python3 src/webots_goal_bridge/scripts/export_distill.py
    python3 src/webots_goal_bridge/scripts/export_distill.py data/distill/raw \\
        --out data/distill/export --val-pct 20 --include-reasoning

입력: data/distill/raw 아래 *.jsonl (하위 폴더 포함). 한 파일 = 한 실행.
  - 새 형식: 첫 줄 {"type": "meta"}, 이후 {"type": "round"} / {"type": "event"}
  - 옛 형식(meta·prompt 없음)도 읽는다. 프롬프트는 관측으로 **다시 만들고** prompt_rebuilt 로 표시한다

출력 (--out):
  sft_train.jsonl / sft_val.jsonl   {"messages": [...], "meta": {...}}  — 교사의 **유효한** 답만
  disagreements.jsonl               교사와 베이스라인이 다르게 고른 주기 (두 답 + 실행된 쪽의 결과)
  summary.json                      실행별 집계

🚨 train/val 은 **실행 단위**로 나눈다. 같은 실행의 연속 주기는 관측이 거의 같아서 행 단위로
   섞으면 검증 점수가 샌다. run_id 해시로 정해서 다시 돌려도 같은 분할이 나온다.

🚨 disagreements 에 chosen/rejected 를 붙이지 않는다. 한 주기에 실제로 실행된 것은 한쪽뿐이라
   다른 쪽의 결과는 관측되지 않았다 — 반사실(counterfactual)이다. 선호 데이터로 쓰려면
   같은 관측에서 양쪽을 각각 실행해 보는 별도 실험이 필요하다.
"""

import argparse
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))       # webots_goal_bridge 패키지 (rclpy 없이 import)


def _sha(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def load_run(path):
    rows = []
    with open(path, encoding='utf-8') as fh:
        for ln, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                print(f'  ⚠️ {path}:{ln} JSON 깨짐 — 건너뜀', file=sys.stderr)
    meta = next((r for r in rows if r.get('type') == 'meta'), None)
    rounds = [r for r in rows if r.get('type') == 'round' or ('result' in r and 'type' not in r)]
    events = [r for r in rows if r.get('type') == 'event' or ('event' in r and 'type' not in r)]
    rounds.sort(key=lambda r: r.get('t_sim', 0.0))
    run_id = (meta or {}).get('run_id') or os.path.splitext(os.path.basename(path))[0]
    return {'path': path, 'run_id': run_id, 'meta': meta or {}, 'rounds': rounds, 'events': events}


def answer_of(res):
    """학습 목표로 쓸 정규화된 답. 교사가 실제로 낸 필드 중 스키마에 있는 것만."""
    out = {'assignments': [{'robot': a['robot'], 'frontier_id': a['frontier_id']}
                           for a in res['assignments']]}
    if res.get('reason'):
        out['reason'] = res['reason']
    return json.dumps(out, ensure_ascii=False)


def outcome(rounds, i):
    """i 번째 판단의 결과 — 다음 판단까지 범위 내 탐색 셀 증가. 기록이 없으면 None."""
    cur = rounds[i].get('explored')
    if not cur:
        return None
    nxt = next((r for r in rounds[i + 1:] if r.get('explored')), None)
    if nxt is None:
        return {'gain_cells': None, 'dt_s': None, 'gain_per_min': None,
                'known': cur['known'], 'total': cur['total'], 'last_round': True}
    dt = nxt['t_sim'] - rounds[i]['t_sim']
    gain = nxt['explored']['known'] - cur['known']
    return {'gain_cells': gain, 'dt_s': round(dt, 1),
            'gain_per_min': round(gain / dt * 60.0, 1) if dt > 0 else None,
            'known': cur['known'], 'total': cur['total'], 'last_round': False,
            'giveups_next': len(nxt.get('events') or [])}


def reasoning_of(row):
    """성공한 답을 만든 호출의 추론 텍스트 (잘리지 않은 마지막 호출)."""
    calls = ((row.get('teacher') or {}).get('calls')) or []
    ok = [c for c in calls if c.get('finish_reason') != 'length' and c.get('content')]
    return (ok[-1].get('reasoning') or None) if ok else None


def in_val(run_id, val_pct):
    return int(_sha(run_id)[:8], 16) % 100 < val_pct


def export(paths, out, val_pct=20, include_reasoning=False, system=None,
           min_gain=None, default_min_sep=15.0):
    from webots_goal_bridge.frontier_allocator import (PROMPT_SHA, PROMPT_VERSION,
                                                       build_alloc_prompt)
    files = []
    for p in paths:
        if os.path.isdir(p):
            for root, _, names in os.walk(p):
                files += [os.path.join(root, n) for n in sorted(names) if n.endswith('.jsonl')]
        elif p.endswith('.jsonl'):
            files.append(p)
    runs = [load_run(f) for f in sorted(files)]

    os.makedirs(out, exist_ok=True)
    fh = {k: open(os.path.join(out, f'{k}.jsonl'), 'w', encoding='utf-8')
          for k in ('sft_train', 'sft_val', 'disagreements')}
    seen = set()
    summary = {'prompt_version_now': PROMPT_VERSION, 'prompt_sha_now': PROMPT_SHA, 'runs': []}
    totals = {'sft_train': 0, 'sft_val': 0, 'disagreements': 0, 'dup': 0, 'filtered_gain': 0}

    for run in runs:
        meta, rounds = run['meta'], run['rounds']
        split = 'sft_val' if in_val(run['run_id'], val_pct) else 'sft_train'
        rs = {'run_id': run['run_id'], 'file': os.path.relpath(run['path']), 'split': split,
              'strategy': meta.get('strategy') or (
                  'llm' if any(str(r['result'].get('method', '')).startswith('llm')
                               for r in rounds) else 'distance'),
              'model': meta.get('model'), 'run_tag': meta.get('run_tag'),
              'rounds': len(rounds), 'teacher_valid': 0, 'fell_back': 0,
              'disagree': 0, 'exported': 0, 'latency_s': []}
        for i, row in enumerate(rounds):
            res = row['result']
            base = row.get('baseline') or res
            is_llm = str(res.get('method', '')).startswith('llm')
            if not is_llm:
                continue
            if res.get('fell_back'):
                rs['fell_back'] += 1
                continue
            rs['teacher_valid'] += 1
            if (row.get('teacher') or {}).get('latency_s') is not None:
                rs['latency_s'].append(row['teacher']['latency_s'])

            min_sep = row.get('min_separation', meta.get('min_separation', default_min_sep))
            prompt = row.get('prompt')
            rebuilt = prompt is None
            if rebuilt:
                prompt = build_alloc_prompt(row['observation'], min_sep)
            ans = answer_of(res)
            oc = outcome(rounds, i)

            A = {(a['robot'], a['frontier_id']) for a in res['assignments']}
            B = {(a['robot'], a['frontier_id']) for a in base['assignments']}
            if A != B:
                rs['disagree'] += 1
                fh['disagreements'].write(json.dumps({
                    'run_id': run['run_id'], 'round': row.get('round', i + 1),
                    'prompt': prompt, 'teacher': json.loads(ans),
                    'baseline': json.loads(answer_of(base)),
                    'teacher_separation': res.get('achieved_separation'),
                    'baseline_separation': base.get('achieved_separation'),
                    'teacher_cost': res.get('total_cost'), 'baseline_cost': base.get('total_cost'),
                    'executed': 'teacher', 'outcome_of_executed': oc,
                }, ensure_ascii=False) + '\n')
                totals['disagreements'] += 1

            if min_gain is not None and (oc is None or oc.get('gain_per_min') is None
                                         or oc['gain_per_min'] < min_gain):
                totals['filtered_gain'] += 1
                continue

            target = ans
            reasoning = reasoning_of(row) if include_reasoning else None
            if reasoning:
                target = f'<think>\n{reasoning.strip()}\n</think>\n{ans}'
            key = _sha(prompt) + _sha(target)
            if key in seen:
                totals['dup'] += 1
                continue
            seen.add(key)

            msgs = ([{'role': 'system', 'content': system}] if system else []) + [
                {'role': 'user', 'content': prompt},
                {'role': 'assistant', 'content': target}]
            fh[split].write(json.dumps({'messages': msgs, 'meta': {
                'run_id': run['run_id'], 'round': row.get('round', i + 1),
                'model': meta.get('model'), 'prompt_version': row.get('prompt_version'),
                'prompt_sha': row.get('prompt_sha'), 'prompt_rebuilt': rebuilt,
                'agrees_with_baseline': A == B, 'with_reasoning': bool(reasoning),
                'outcome': oc}}, ensure_ascii=False) + '\n')
            totals[split] += 1
            rs['exported'] += 1
        lat = rs.pop('latency_s')
        rs['latency_mean_s'] = round(sum(lat) / len(lat), 1) if lat else None
        summary['runs'].append(rs)

    for f in fh.values():
        f.close()
    summary['totals'] = totals
    with open(os.path.join(out, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('inputs', nargs='*', default=['data/distill/raw'])
    ap.add_argument('--out', default='data/distill/export')
    ap.add_argument('--val-pct', type=int, default=20, help='검증으로 보낼 실행 비율(%%)')
    ap.add_argument('--include-reasoning', action='store_true',
                    help='교사의 추론 텍스트를 <think> 로 감싸 답 앞에 붙인다')
    ap.add_argument('--system', default=None, help='학생용 시스템 프롬프트 (기본: 없음)')
    ap.add_argument('--min-gain', type=float, default=None,
                    help='다음 판단까지 분당 탐색 셀 증가가 이 값 미만인 샘플은 뺀다')
    a = ap.parse_args(argv)
    s = export(a.inputs, a.out, a.val_pct, a.include_reasoning, a.system, a.min_gain)
    t = s['totals']
    print(f"{'run_id':42s} {'분할':9s} {'전략':8s} {'주기':>4s} {'유효':>4s} {'폴백':>4s} {'갈림':>4s} {'내보냄':>5s} {'지연s':>6s}")
    for r in s['runs']:
        print(f"{r['run_id'][:42]:42s} {r['split']:9s} {r['strategy']:8s} {r['rounds']:4d} "
              f"{r['teacher_valid']:4d} {r['fell_back']:4d} {r['disagree']:4d} {r['exported']:5d} "
              f"{r['latency_mean_s'] if r['latency_mean_s'] is not None else '—':>6}")
    print(f"\ntrain {t['sft_train']} · val {t['sft_val']} · 갈림 {t['disagreements']} · "
          f"중복 제외 {t['dup']} · 결과 기준 제외 {t['filtered_gain']}  →  {a.out}")


if __name__ == '__main__':
    main()
