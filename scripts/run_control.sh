#!/usr/bin/env bash
# 통합관제 기동 — 시뮬레이터 + 로봇 스택 + 웹 관제 (1대)
#   scripts/run_control.sh [groundtruth|fastlio] [포트] [로봇] [real|terraced]
#   SLAM=… PORT=… ROBOT=… WORLD=… LOG_DIR=… scripts/run_control.sh   (환경변수도 가능)
#
# 로봇 이름(기본 scout01)은 월드의 <include><name> 이자 토픽·TF 접두다.
# 4번째 인자로 월드를 고른다 — **기본은 real**(지면이 정사영상인 평탄 월드,
# 스펙 ④ T7 게이트 통과로 전환). 계단식 연구 트랙은 terraced 로 명시한다.
# 2대 편대는 scripts/run_fleet.sh 를 쓴다.
#
# 뜨고 나면 브라우저로 안내된 주소를 열면 된다. 관제 PC 에는 아무것도 안 깔아도 된다.
set -o pipefail   # set -u 금지 — ROS setup.bash 가 미정의 변수를 참조한다
cd "$(dirname "$0")/.."
ROOT=$(pwd)
SLAM=${1:-${SLAM:-groundtruth}}
PORT=${2:-${PORT:-8080}}
ROBOT=${3:-${ROBOT:-scout01}}
WORLD_KIND=${4:-${WORLD:-real}}
case "$WORLD_KIND" in
  terraced) WORLD_NAME=orchard_10x41; WORLD_SDF=sim/worlds/orchard_nav.sdf ;;
  real)     WORLD_NAME=orchard_real;  WORLD_SDF=sim/worlds/orchard_real.sdf ;;
  *) echo "!! 월드는 terraced | real 입니다: $WORLD_KIND"; exit 2 ;;
esac
WORLD=$WORLD_NAME
# 로그는 저장소·세션과 무관한 곳에 둔다 (LOG_DIR 로 덮어쓸 수 있다)
LOG=${LOG_DIR:-${TMPDIR:-/tmp}/orchard_logs}
mkdir -p "$LOG"

source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$ROOT/sim/models:${GZ_SIM_RESOURCE_PATH:-}"

cleanup() {
  echo
  echo "[정리]"
  [[ -n "${LAUNCH_PID:-}" ]] && kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 3
  [[ -n "${GZ_PID:-}" ]] && kill "$GZ_PID" 2>/dev/null
  sleep 2
  for p in "gz sim" parameter_bridge "orchard_sim/control_agent" fastlio_mapping \
           "orchard_sim/livox_sim_bridge" "orchard_sim/gt_localizer" \
           "orchard_sim/sdf_static_tf"; do
    for pid in $(pgrep -f "$p"); do kill "$pid" 2>/dev/null; done
  done
  return 0
}
trap cleanup EXIT INT TERM

echo "[1/3] Gazebo 기동 (헤드리스, 월드=$WORLD_KIND → $WORLD_SDF)"
gz sim -s -r -v2 "$ROOT/$WORLD_SDF" > "$LOG/gz_ctl.log" 2>&1 &
GZ_PID=$!
for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q "/world/$WORLD/stats" && { echo "      준비됨 (${i}s)"; READY=1; break; }
  sleep 1
done
[[ -z "${READY:-}" ]] && { echo "!! 월드 미기동"; tail -20 "$LOG/gz_ctl.log"; exit 1; }

# 계단식 월드의 스폰 지점은 헤드랜드 램프 위라 횡경사에 흘러내린다 (2026-07-26 실측).
# 통로 안 평탄 테라스로 옮기고 시작한다. 실사(평탄) 월드에는 램프가 없고 스폰
# 좌표를 gen_world 가 farm.json 에서 계산해 박아 두므로 옮기지 않는다.
if [[ "$WORLD_KIND" == "terraced" ]]; then
  gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 3000 --req "name: \"$ROBOT\", position: {x: -14.0, y: -28.0, z: 0.80}, orientation: {x: 0, y: 0, z: 0.7071068, w: 0.7071068}" >/dev/null
  sleep 2
fi

echo "[2/3] 로봇 스택 + 관제 에이전트 (로봇=$ROBOT, SLAM=$SLAM, 월드=$WORLD_KIND)"
ros2 launch orchard_sim control.launch.py world:=$WORLD_KIND world_name:=$WORLD slam:=$SLAM \
  port:=$PORT robot_id:=$ROBOT ns:=$ROBOT > "$LOG/control.log" 2>&1 &
LAUNCH_PID=$!

echo -n "      관제 서버 대기"
for i in $(seq 1 60); do
  if grep -q "관제 서버 시작" "$LOG/control.log" 2>/dev/null; then echo " — OK (${i}s)"; COK=1; break; fi
  echo -n "."; sleep 1
done
if [[ -z "${COK:-}" ]]; then
  echo; echo "!! 관제 서버 미기동:"; tail -30 "$LOG/control.log"; exit 1
fi

echo
echo "════════════════════════════════════════════════════════════"
grep -A2 "관제 서버 시작" "$LOG/control.log" | head -3 | sed 's/^\[INFO\].*control_agent\]: /  /'
echo "════════════════════════════════════════════════════════════"
echo "  브라우저에서 위 주소를 여세요. 관제 PC 에 ROS 설치 불필요."
echo "  종료: Ctrl+C"
echo

echo "[3/3] 운영 중 — 로그: $LOG/control.log"
while kill -0 $LAUNCH_PID 2>/dev/null; do sleep 2; done
