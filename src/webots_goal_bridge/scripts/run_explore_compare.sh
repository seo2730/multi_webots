#!/bin/bash
# 탐사 할당 전략 비교 — 같은 실험을 전략을 번갈아(B L B L ...) 반복하고, 증류 데이터를 영구 폴더에 쌓는다.
#
#   src/webots_goal_bridge/scripts/run_explore_compare.sh <os> <쌍 수> ["distance llm"] [시행당 벽시계 상한 초]
#   src/webots_goal_bridge/scripts/run_explore_compare.sh mac 4
#   DRY=1 src/webots_goal_bridge/scripts/run_explore_compare.sh mac 2        # 계획만 출력
#
# 전제: Webots 가 실험 월드(기본 oneroom)를 열고 ▶ 상태, explorer 이미지가 최신.
#
# 결과 (저장소 data/distill/, git 에 안 올라감):
#   raw/{run_id}.jsonl          할당기 원시 로그 — 첫 줄 meta.run_tag 로 시행을 식별
#   cov/{run_tag}.json          내부 커버리지·이동거리 곡선
#   logs/{run_tag}_explorer.log 할당기 로그
# 분석:  python3 src/webots_goal_bridge/scripts/analyze_explore_compare.py --filter <묶음>
# 증류:  python3 src/webots_goal_bridge/scripts/export_distill.py
#
# 🚨 번갈아 도는 이유 — 한 전략을 몰아서 돌리면 Webots·API 상태의 시간 변화가 전략 차이로 섞인다.
# 🚨 시행마다 /remove_robot {all, force} 를 먼저 한다. Webots 가 호스트에서 돌면 compose down 이
#    월드를 지우지 않아 지난 시행의 몸을 물려받는다 (03장 8절). timeout 은 Webots 가 죽어 있을 때
#    소환기가 응답하지 않아 스크립트 전체가 멈추는 것을 막는다.
set -u
OS=${1:?사용: $0 <mac|ubuntu|windows> <쌍 수> [순서] [상한 초]}
PAIRS=${2:-2}
ORDER=${3:-"distance llm"}
DUR=${4:-2000}
EXP=${EXP:-oneroom_in2}                      # experiments/{EXP}.override.yml
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
OVR="$REPO/src/webots_goal_bridge/scripts/experiments/${EXP}.override.yml"
C="docker compose -f $REPO/docker-configs/$OS/docker-compose.yml -f $OVR"
BATCH=${BATCH:-b$(date -u +%Y%m%dT%H%M%SZ)}
GIT=$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo nogit)
# 커밋 안 된 변경으로 돌면 이름표의 커밋이 코드를 속인다 — 표시해 둔다
# --ignore-submodules=all: 서브모듈의 Webots GUI 상태(.wbproj)까지 dirty 로 잡히면
# 소스가 깨끗해도 이름표가 늘 -dirty 가 된다
git -C "$REPO" diff --quiet --ignore-submodules=all HEAD -- src docker-configs 2>/dev/null \
  || GIT="${GIT}-dirty"
DATA="$REPO/data/distill"

export EXPLORE_PERIOD=${EXPLORE_PERIOD:-60.0}
export EXPLORE_MIN_SEP=${EXPLORE_MIN_SEP:-15.0}
export EXPLORE_BOUNDS=${EXPLORE_BOUNDS:--37.0,-37.0,37.0,37.0}   # 할당기: 벽 안쪽 면에서 0.5 m 들임
COV_BOUNDS=${COV_BOUNDS:--37.5,-37.5,37.5,37.5}                 # 커버리지 분모: 벽 안쪽 면
ROBOTS=${ROBOTS:-ugv1,drone1}
unset EXPLORE_DATASET                        # 파일 하나가 아니라 dataset_dir(영구 폴더)로 쌓는다

if [ -z "${NVIDIA_API_KEY:-}" ]; then
  eval "$(grep -h '^export NVIDIA_API_KEY=\|^NVIDIA_API_KEY=' "$HOME/.zshrc" "$HOME/.bashrc" 2>/dev/null | tail -1)" || true
fi
export NVIDIA_API_KEY=${NVIDIA_API_KEY:-}

[ -f "$OVR" ] || { echo "오버라이드 없음: $OVR"; exit 1; }
echo "실험 $EXP | 묶음 $BATCH | 커밋 $GIT | OS $OS | 쌍 $PAIRS | 순서 $ORDER | 상한 ${DUR}s"
echo "주기 $EXPLORE_PERIOD s | 분리 $EXPLORE_MIN_SEP m | 탐사 범위 $EXPLORE_BOUNDS | NIM 키 $([ -n "$NVIDIA_API_KEY" ] && echo 있음 || echo 없음)"
if [ -n "${DRY:-}" ]; then
  for p in $(seq 1 "$PAIRS"); do for s in $ORDER; do echo "  시행 ${EXP//_/-}_${BATCH}_${GIT}_${s}_${p}"; done; done
  echo "  compose: $C"; exit 0
fi
mkdir -p "$DATA/raw" "$DATA/cov" "$DATA/logs"

for p in $(seq 1 "$PAIRS"); do
  for strat in $ORDER; do
    TAG="${EXP//_/-}_${BATCH}_${GIT}_${strat}_${p}"
    export EXPLORE_STRATEGY=$strat EXPLORE_TAG=$TAG
    echo "===== $TAG 시작 $(date '+%H:%M:%S') ====="

    if docker ps --format '{{.Names}}' | grep -q "^fleet_spawner_${OS}\$"; then
      docker exec "fleet_spawner_${OS}" bash -lc \
        "source /ros2_ws/install/setup.bash && timeout 60 ros2 service call /remove_robot \
         webots_spawner_msgs/srv/RemoveRobot '{all: true, force: true}'" 2>&1 \
        | grep -oE "removed=\[[^]]*\]" | head -1 || echo "  ⚠️ 월드 비우기 실패 — Webots 확인"
    fi
    $C --profile explore down --remove-orphans --timeout 15 >/dev/null 2>&1
    $C --profile explore up -d master fleet ugv1 drone1 explorer >/dev/null 2>&1

    for _ in $(seq 1 60); do
      docker logs "fleet_spawner_${OS}" 2>&1 | grep -q "편대 소환 완료" && break; sleep 5
    done
    SPAWN=$(docker logs "fleet_spawner_${OS}" 2>&1 | grep -oE "이미 있는 로봇 [0-9]+대|새로 [0-9]+대, 뇌만 붙임 [0-9]+대" | tr '\n' ' ')
    echo "  소환: $SPAWN"
    echo "$SPAWN" | grep -qE "이미 있는 로봇 [1-9]|뇌만 붙임 [1-9]" && echo "  ⚠️ 잔여 몸을 물려받았다 — 이 시행은 비교에서 뺄 것"

    for _ in $(seq 1 30); do
      docker logs "explorer_${OS}" 2>&1 | grep -q "할당기 시작" && break; sleep 5
    done
    START=$(docker logs "explorer_${OS}" 2>&1 | grep -oE "전략 [a-z]+ \| 주기 [0-9.]+s|탐사 범위 .*" | tr '\n' ' ')
    echo "  할당기: $START"
    echo "$START" | grep -q "전략 $strat " || echo "  ⚠️ 전략이 $strat 로 뜨지 않았다 — compose 명령 확인"

    for _ in $(seq 1 30); do
      docker exec "rviz_master_${OS}" bash -lc \
        'source /ros2_ws/install/setup.bash && ros2 topic info /clock 2>/dev/null | grep -q "Publisher count: 1"' \
        2>/dev/null && break
      sleep 10
    done
    docker cp "$REPO/src/webots_goal_bridge/scripts/coverage_logger.py" "rviz_master_${OS}:/tmp/coverage_logger.py" >/dev/null
    docker exec "rviz_master_${OS}" bash -lc \
      "source /ros2_ws/install/setup.bash && python3 -u /tmp/coverage_logger.py $DUR 60 /tmp/cov.json $COV_BOUNDS $ROBOTS" \
      | grep -E "내부|측정 종료|기록 없음"

    docker cp "rviz_master_${OS}:/tmp/cov.json" "$DATA/cov/${TAG}.json" >/dev/null 2>&1 || echo "  ⚠️ 커버리지 회수 실패"
    docker logs "explorer_${OS}" > "$DATA/logs/${TAG}_explorer.log" 2>&1
    RAW=$(grep -l "\"run_tag\": \"$TAG\"" "$DATA"/raw/*.jsonl 2>/dev/null | head -1)
    if [ -n "$RAW" ]; then
      echo "  저장: raw/$(basename "$RAW") ($(wc -l < "$RAW" | tr -d ' ')줄) · cov/${TAG}.json"
    else
      echo "  ⚠️ raw 로그를 못 찾음 — explorer 에 data/distill 이 마운트됐는지 확인"
    fi
  done
done
echo "===== 묶음 $BATCH 완료 $(date '+%H:%M:%S') ====="
python3 "$REPO/src/webots_goal_bridge/scripts/analyze_explore_compare.py" --root "$DATA" --filter "$BATCH" || true
