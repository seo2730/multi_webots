"""탐사 범위 안 커버리지 + 로봇별 이동거리를 시뮬 시각 기준으로 기록한다.

master 컨테이너 안에서 돈다 (rclpy 필요). 비교 실험 스크립트가 복사해 넣는다.

    python3 coverage_logger.py <벽시계 상한 초> <간격 시뮬 초> <출력 JSON> [x0,y0,x1,y1] [로봇,로봇]
    python3 coverage_logger.py 2000 60 /tmp/cov.json -37.5,-37.5,37.5,37.5 ugv1,drone1

범위는 **벽 안쪽 면**을 준다 (할당기의 탐사 범위는 거기서 0.5 m 들인다 — 분모는 방 전체여야 한다).
행마다 t(측정 시작부터), t_abs(절대 시뮬 시각 — 할당기 데이터셋의 t_sim 과 같은 시계),
pct, 로봇별 누적 이동거리를 남기고, 종료 사유를 마지막 행에 적는다.
"""

import json, math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

DUR = float(sys.argv[1]); STEP = float(sys.argv[2]); OUT = sys.argv[3]
B = tuple(float(v) for v in (sys.argv[4] if len(sys.argv) > 4
                             else '-37.5,-37.5,37.5,37.5').split(','))
ROBOTS = tuple((sys.argv[5] if len(sys.argv) > 5 else 'ugv1,drone1').split(','))

MQ = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
CQ = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.BEST_EFFORT)

rclpy.init()
n = Node('covlog_in')
st = {'dist': {r: 0.0 for r in ROBOTS}, 'last': {}}

def on_odom(r):
    def cb(m):
        p = m.pose.pose.position
        prev = st['last'].get(r)
        if prev is not None:
            d = math.hypot(p.x - prev[0], p.y - prev[1])
            if d < 2.0:                 # 재소환 등 순간이동은 이동거리가 아니다
                st['dist'][r] += d
        st['last'][r] = (p.x, p.y)
    return cb

n.create_subscription(OccupancyGrid, '/map_merged', lambda m: st.__setitem__('map', m), MQ)
n.create_subscription(Clock, '/clock',
                      lambda m: st.__setitem__('t', m.clock.sec + m.clock.nanosec * 1e-9), CQ)
for r in ROBOTS:
    n.create_subscription(Odometry, f'/{r}/odom', on_odom(r), 10)

def interior(m):
    i = m.info
    g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
    ox, oy, res = i.origin.position.x, i.origin.position.y, i.resolution
    # 🚨 창의 양 끝을 같은 방식(round)으로 잡는다. floor/ceil 을 섞으면 창이 분모보다
    #    한 칸 넓어져 커버리지가 100% 를 넘는다 (실측 100.25%).
    c0 = max(0, int(round((B[0] - ox) / res)))
    c1 = min(i.width, int(round((B[2] - ox) / res)))
    r0 = max(0, int(round((B[1] - oy) / res)))
    r1 = min(i.height, int(round((B[3] - oy) / res)))
    win = g[r0:r1, c0:c1]
    total = int(round((B[2] - B[0]) / res)) * int(round((B[3] - B[1]) / res))
    return int((win >= 0).sum()), total

# 종료 조건 — 두 전략에 똑같이 적용한다.
#   빈 방은 10분 남짓이면 다 보여서, 고정 시간으로 재면 나머지를 헛돌린다
#   (실측: 660초에 99.87% 도달 후 2000초까지 변화 없음).
#   ① 99.5% 이상이면 끝
#   ② 95% 이상에서 5스텝(5분) 동안 0.05%p 도 안 늘면 끝 — 벽 속 셀처럼 원래 못 보는 잔여
#   95% 조건을 거는 이유: 느린 전략이 중간에 잠깐 정체한 것을 완료로 오인하지 않게.
DONE_PCT, PLATEAU_MIN, PLATEAU_N, PLATEAU_EPS = 99.5, 95.0, 5, 0.05
stop_reason = f'벽시계 {DUR:.0f}초'

rows, t0, nxt = [], None, 0.0
start = time.time()
while time.time() - start < DUR:
    rclpy.spin_once(n, timeout_sec=0.2)
    m, t = st.get('map'), st.get('t')
    if m is None or t is None:
        continue
    if t0 is None:
        t0 = t
    if t - t0 < nxt:
        continue
    nxt += STEP
    known, total = interior(m)
    # t_abs = 절대 시뮬 시각. 할당기 데이터셋의 t_sim 과 같은 시계라 둘을 맞춰 볼 수 있다.
    row = {'t': round(t - t0, 1), 't_abs': round(t, 1), 'known': known, 'total': total,
           'pct': round(100.0 * known / max(1, total), 2),
           'dist': {r: round(v, 1) for r, v in st['dist'].items()}}
    rows.append(row)
    print(f"  {row['t']:7.0f}s  내부 {row['pct']:6.2f}%  이동 "
          + ' · '.join(f"{r} {row['dist'][r]:6.1f} m" for r in ROBOTS), flush=True)
    json.dump(rows, open(OUT, 'w'))
    if row['pct'] >= DONE_PCT:
        stop_reason = f'{DONE_PCT}% 도달'
        break
    if (row['pct'] >= PLATEAU_MIN and len(rows) > PLATEAU_N
            and row['pct'] - rows[-1 - PLATEAU_N]['pct'] < PLATEAU_EPS):
        stop_reason = f'{PLATEAU_MIN}% 이상에서 {PLATEAU_N}분 정체'
        break

if rows:
    rows[-1]['stop_reason'] = stop_reason
json.dump(rows, open(OUT, 'w'))
print(f'측정 종료: {stop_reason}')
print(f"최종 내부 {rows[-1]['pct']:.2f}% @ {rows[-1]['t']:.0f}s" if rows
      else '기록 없음 — /map_merged 또는 /clock 을 못 받았다')
