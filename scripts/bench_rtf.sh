#!/usr/bin/env bash
# RTF 벤치마크 — 어떤 요소가 실시간 배율을 잡아먹는지 분리 측정한다.
#
#   bash scripts/bench_rtf.sh <world.sdf> [측정라벨]
#
# 헤드리스 서버로만 돌린다(GUI 렌더는 별도 비용이라 섞이면 해석이 흐려진다).
# 워밍업 후 정상 상태 RTF 를 표본으로 잡는다.
set -uo pipefail
WORLD="${1:?사용법: bash scripts/bench_rtf.sh <world.sdf> [라벨]}"
LABEL="${2:-$(basename "$WORLD" .sdf)}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

set +u                      # ROS setup.bash 가 미정의 변수를 건드린다
source /opt/ros/jazzy/setup.bash
set -u
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH="$ROOT/sim/models:${GZ_SIM_RESOURCE_PATH:-}"

WORLD_NAME=$(grep -oP '(?<=<world name=")[^"]+' "$WORLD" | head -1)

gz sim -v1 -s -r "$WORLD" >/tmp/bench_$$.log 2>&1 &
SIM=$!
trap "kill -9 $SIM 2>/dev/null; wait $SIM 2>/dev/null" EXIT

# 로드 + 워밍업 대기 (엔티티 수에 따라 길어진다)
until gz topic -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/stats"; do
  kill -0 $SIM 2>/dev/null || { echo "[bench] 서버가 죽음 — /tmp/bench_$$.log 확인"; exit 1; }
  sleep 2
done
sleep 12          # 정상 상태 진입

SAMPLES=$(timeout 20 gz topic -e -t "/world/${WORLD_NAME}/stats" -n 12 2>/dev/null \
          | grep -oP '(?<=real_time_factor: )[0-9.]+' | tail -8)

if [ -z "$SAMPLES" ]; then
  echo "[bench] $LABEL : 표본 없음"
  exit 1
fi

echo "$SAMPLES" | awk -v L="$LABEL" '
  {s+=$1; n++; if(min==""||$1<min)min=$1; if($1>max)max=$1}
  END{printf "[bench] %-28s RTF 평균 %.3f  (범위 %.3f~%.3f, n=%d)\n", L, s/n, min, max, n}'
