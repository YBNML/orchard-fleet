#!/usr/bin/env bash
# 편대 기동 — 시뮬레이터 하나 + 로봇 스택 2대(관제 8080/8081)
#
#   scripts/run_fleet.sh [real|terraced] [로봇수]
#   WORLD=real ROBOTS=2 LOG_DIR=~/logs scripts/run_fleet.sh
#
# 월드 SDF·gz 월드 이름·런치 world 인자를 한 곳에서 고른다. **기본은 real**
# (스펙 ④ T7 게이트 통과, 2026-08-15 — docs/findings/2026-08-15-photoreal-world.md).
# 주의: 기본 월드(real)에서 10통로급 장주행 임무는 열 끝 감시 오탐(수리 예정)으로 아직 완주가 보장되지 않는다 — docs/findings/2026-08-15-photoreal-world.md §9
#   real      sim/worlds/orchard_real.sdf  · gz 월드 orchard_real
#             기하(rows·row_spacing·통로 중심·열 구간)를 maps/orchard_real/farm.json
#             에서 읽어 control_agent 파라미터로 넘긴다 (robomw 무관, 데이터 주도)
#   terraced  sim/worlds/orchard_nav.sdf   · gz 월드 orchard_10x41
#             계단식 연구 트랙 — 병존한다. 쓰려면 명시 인자로 고른다
#
# /clock 브리지는 월드당 하나뿐이라 1호기만 clock:=true 다.
# 종료: scripts/stop_all.sh 또는 Ctrl+C.
set -o pipefail   # set -u 금지 — ROS setup.bash 가 미정의 변수를 참조한다
cd "$(dirname "$0")/.."
ROOT=$(pwd)

WORLD_KIND=${1:-${WORLD:-real}}
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

# ── 계단식 월드 전용 스폰 보정 ──────────────────────────────────────────────
# 계단식 월드의 월드 스폰 지점 (-14, -33) 은 **헤드랜드 램프 위**다. 거기서는
# 횡경사 11.8% 때문에 직진만 해도 옆으로 흘러 밭 밖으로 나가고 결국 전복한다
# (2026-07-26 실측, scripts/15_drive_probe.py). 통로 안 평탄 테라스(-14, -28)로
# 옮기면 횡경사 0% 다. run_control.sh 가 1대에 대해 하던 것과 **같은 보정**이다.
#
# scout02 (14, -33) 은 보정하지 않는다 — 그 자리는 선회 평지 패드 7S
# (x 8.9~15.6, |y|≥32.5 평탄, gen_heightmap --turn-pads)의 한복판이라 이미
# 평지다. 실사(평탄) 월드는 램프 자체가 없고 스폰 좌표를 gen_world 가
# farm.json 에서 계산해 박아 두므로 전 로봇 보정 불요.
fix_spawn() {   # $1=로봇 이름
  local rid=$1 req=""
  case "$WORLD_KIND:$rid" in
    terraced:scout01) req='position: {x: -14.0, y: -28.0, z: 0.80}, orientation: {x: 0, y: 0, z: 0.7071068, w: 0.7071068}' ;;
    *) return 0 ;;
  esac
  gz service -s "/world/$WORLD_NAME/set_pose" --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"$rid\", $req" >/dev/null && echo "      $rid 스폰 보정 (램프 밖 평탄 테라스로)"
  sleep 2
}

echo "[2/2] 로봇 스택 ${N}대"
for i in $(seq 1 "$N"); do
  RID=$(printf "scout%02d" "$i")
  fix_spawn "$RID"
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
