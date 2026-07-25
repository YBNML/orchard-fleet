#!/usr/bin/env bash
# 로봇 확인 세션 — Gazebo GUI + Stage-0 스택 + RViz 를 한 번에 띄운다
#
#   bash scripts/run_robot_check.sh          # 주행용 월드 (RTF 높음)
#   bash scripts/run_robot_check.sh gt       # 정답 라벨용 월드 (과실 인스턴스 포함)
#   bash scripts/run_robot_check.sh nav rviz-off
#
# 띄운 뒤 조작은 별도 터미널에서:
#   source /opt/ros/jazzy/setup.bash && source ~/YBNML/ros2_ws/install/setup.bash
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WHICH="${1:-nav}"
RVIZ="${2:-rviz-on}"

case "$WHICH" in
  nav) WORLD="$ROOT/sim/worlds/orchard_nav.sdf" ;;
  gt)  WORLD="$ROOT/sim/worlds/orchard_gt.sdf" ;;
  *)   echo "사용법: $0 [nav|gt] [rviz-on|rviz-off]"; exit 2 ;;
esac
[ -f "$WORLD" ] || { echo "월드가 없습니다: $WORLD"; echo "  python3 scripts/gen_world.py ... 로 먼저 생성하세요"; exit 2; }

WORLD_NAME=$(grep -oP '(?<=<world name=")[^"]+' "$WORLD" | head -1)

set +u; source /opt/ros/jazzy/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH="$ROOT/sim/models:${GZ_SIM_RESOURCE_PATH:-}"
export DISPLAY="${DISPLAY:-:1}"

echo "▶ 기존 세션 정리"
for p in $(ps -eo pid,args | grep -E 'gz sim|parameter_bridge|livox_sim_bridge|gt_localizer|sdf_static_tf|rviz2|ros2 launch orchard_sim' \
           | grep -v 'grep\|bash -c\|run_robot_check' | awk '{print $1}'); do
  kill "$p" 2>/dev/null || true
done
sleep 3

echo "▶ Gazebo GUI 기동 — $WHICH ($WORLD_NAME)"
setsid nohup gz sim -v2 -r "$WORLD" > /tmp/rc_gz.log 2>&1 < /dev/null &
disown

# 준비 판정은 **토픽**으로 한다. 로그 문자열은 -v 레벨과 GUI/헤드리스 모드에 따라
# 나오기도 안 나오기도 한다 (-v2 에서는 "[Msg] Heightmap loaded" 가 아예 안 찍힌다).
echo "  씬 로드 대기 (엔티티가 많아 30~60초 걸립니다)"
DEADLINE=$(( $(date +%s) + 180 ))
until gz topic -l 2>/dev/null | grep -q "^/world/${WORLD_NAME}/stats$"; do
  sleep 3
  # 프로세스 이름은 ruby 라서 comm 이 아니라 전체 명령줄로 확인해야 한다
  ps -eo args | grep -q "[g]z sim" || {
    echo "  ✗ gz sim 이 죽었습니다 — tail /tmp/rc_gz.log"; exit 1; }
  [ "$(date +%s)" -lt "$DEADLINE" ] || {
    echo "  ✗ 180초 안에 월드 토픽이 안 올라왔습니다 — tail /tmp/rc_gz.log"; exit 1; }
done
echo "  ✔ 씬 로드 완료"

echo "▶ Stage-0 스택 기동 (브리지 + 참값 로컬라이저 + Livox 계약)"
setsid nohup ros2 launch orchard_sim stage0.launch.py world_name:="$WORLD_NAME" \
  > /tmp/rc_stage0.log 2>&1 < /dev/null &
disown

echo "  참값 로컬라이저 대기"
DEADLINE=$(( $(date +%s) + 90 ))
until ros2 node list 2>/dev/null | grep -q '^/gt_localizer$'; do
  sleep 3
  [ "$(date +%s)" -lt "$DEADLINE" ] || {
    echo "  ✗ Stage-0 스택이 안 올라옵니다 — tail /tmp/rc_stage0.log"; exit 1; }
done
echo "  ✔ Stage-0 스택 준비"

if [ "$RVIZ" = "rviz-on" ]; then
  echo "▶ RViz 기동"
  setsid nohup rviz2 -d "$ROOT/ros2_ws/src/orchard_sim/config/robot_check.rviz" \
    --ros-args -p use_sim_time:=true > /tmp/rc_rviz.log 2>&1 < /dev/null &
  disown
  DEADLINE=$(( $(date +%s) + 60 ))
  until ps -eo comm | grep -qx rviz2; do
    sleep 2
    [ "$(date +%s)" -lt "$DEADLINE" ] || {
      echo "  ✗ RViz 가 안 뜹니다 — tail /tmp/rc_rviz.log"; break; }
  done
  ps -eo comm | grep -qx rviz2 && echo "  ✔ RViz 기동"
fi

echo
echo "════════════════════════════════════════════════════════════"
echo " 준비 완료"
echo "════════════════════════════════════════════════════════════"
echo
echo " 키보드 주행 — 새 터미널에서:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   source $ROOT/ros2_ws/install/setup.bash"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "     i 전진 / , 후진 / j 좌회전 / l 우회전 / k 정지"
echo "     q,z 속도 조절.  통로 촬영 속도는 0.6 m/s 입니다"
echo
echo " 상태 확인:"
echo "   ros2 topic hz /livox/lidar          점군 발행률"
echo "   ros2 run tf2_tools view_frames      TF 트리 PDF 저장"
echo "   gz topic -e -t /world/$WORLD_NAME/stats -n 3 | grep real_time"
echo
echo " 로그: /tmp/rc_gz.log  /tmp/rc_stage0.log  /tmp/rc_rviz.log"
echo " 종료: bash scripts/stop_all.sh"
echo
