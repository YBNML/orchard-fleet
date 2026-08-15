#!/usr/bin/env bash
# 편대 기동 — 시뮬레이터 하나 + 로봇 스택 2대(관제 8080/8081)
#
#   scripts/run_fleet.sh [terraced|real] [로봇수]
#   WORLD=real ROBOTS=2 LOG_DIR=~/logs scripts/run_fleet.sh
#
# 월드 SDF·gz 월드 이름·런치 world 인자를 한 곳에서 고른다. **기본은 terraced**
# (실사 월드로의 기본 전환은 스펙 ④ T7 게이트 이후다).
#   terraced  sim/worlds/orchard_nav.sdf   · gz 월드 orchard_10x41
#   real      sim/worlds/orchard_real.sdf  · gz 월드 orchard_real
#             기하(rows·row_spacing·통로 중심·열 구간)를 maps/orchard_real/farm.json
#             에서 읽어 control_agent 파라미터로 넘긴다 (robomw 무관, 데이터 주도)
#
# /clock 브리지는 월드당 하나뿐이라 1호기만 clock:=true 다.
# 종료: scripts/stop_all.sh 또는 Ctrl+C.
set -o pipefail   # set -u 금지 — ROS setup.bash 가 미정의 변수를 참조한다
cd "$(dirname "$0")/.."
ROOT=$(pwd)

WORLD_KIND=${1:-${WORLD:-terraced}}
N=${2:-${ROBOTS:-2}}
case "$WORLD_KIND" in
  terraced) WORLD_NAME=orchard_10x41; WORLD_SDF=sim/worlds/orchard_nav.sdf ;;
  real)     WORLD_NAME=orchard_real;  WORLD_SDF=sim/worlds/orchard_real.sdf ;;
  *) echo "!! 월드는 terraced | real 입니다: $WORLD_KIND"; exit 2 ;;
esac
[[ -f "$ROOT/$WORLD_SDF" ]] || {
  echo "!! 월드 SDF 가 없습니다: $WORLD_SDF"
  echo "   실사 월드는 scripts/gen_heightmap.py --flat-gentle --farm … 로 지형을 만든 뒤"
  echo "   scripts/gen_world.py --farm maps/orchard_real/farm.json … 로 생성합니다."
  exit 2; }
LOG=${LOG_DIR:-${TMPDIR:-/tmp}/orchard_logs}
mkdir -p "$LOG"

source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$ROOT/sim/models:${GZ_SIM_RESOURCE_PATH:-}"

PIDS=()
cleanup() {
  echo; echo "[정리]"
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 4
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  return 0
}
trap cleanup EXIT INT TERM

echo "[1/2] Gazebo 기동 (헤드리스, 월드=$WORLD_KIND → $WORLD_SDF)"
gz sim -s -r -v2 "$ROOT/$WORLD_SDF" > "$LOG/gz_${WORLD_KIND}.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 180); do
  gz topic -l 2>/dev/null | grep -q "/world/$WORLD_NAME/stats" && { echo "      준비됨 (${i}s)"; READY=1; break; }
  sleep 1
done
[[ -z "${READY:-}" ]] && { echo "!! 월드 미기동"; tail -25 "$LOG/gz_${WORLD_KIND}.log"; exit 1; }

echo "[2/2] 로봇 스택 ${N}대"
for i in $(seq 1 "$N"); do
  RID=$(printf "scout%02d" "$i")
  PORT=$((8079 + i))
  CLK=$([[ $i -eq 1 ]] && echo true || echo false)
  ros2 launch orchard_sim control.launch.py \
    world:="$WORLD_KIND" world_name:="$WORLD_NAME" \
    robot_id:="$RID" ns:="$RID" port:="$PORT" clock:="$CLK" \
    > "$LOG/ctl_${RID}.log" 2>&1 &
  PIDS+=($!)
  echo "      $RID → http://<이 PC>:$PORT/   (로그 $LOG/ctl_${RID}.log)"
  sleep 6
done

echo -n "      관제 서버 대기"
for i in $(seq 1 90); do
  n=$(grep -l "관제 서버 시작" "$LOG"/ctl_scout*.log 2>/dev/null | wc -l)
  [[ "$n" -ge "$N" ]] && { echo " — OK (${i}s, ${n}대)"; COK=1; break; }
  echo -n "."; sleep 1
done
[[ -z "${COK:-}" ]] && { echo; echo "!! 일부 관제 서버 미기동:"; tail -20 "$LOG"/ctl_scout*.log; }

echo
echo "════════════════════════════════════════════════════════════"
echo "  월드 $WORLD_KIND ($WORLD_NAME) · 로봇 ${N}대 운용 중"
echo "  종료: Ctrl+C"
echo "════════════════════════════════════════════════════════════"
while true; do sleep 5; done
