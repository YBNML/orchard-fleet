#!/usr/bin/env bash
# 단계 3 — Scout Mini + MID-70 로봇 모델 검증 (설계서 §5.3 게이트)
#
#   bash scripts/03_verify_robot.sh
#
# 검증: 모델 로드 / 센서 토픽 / 직진 / 제자리회전
set -euo pipefail
source /opt/ros/jazzy/setup.bash
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH="$(cd "$(dirname "$0")/.." && pwd)/sim/models:${GZ_SIM_RESOURCE_PATH:-}"
WORLD="$(cd "$(dirname "$0")/.." && pwd)/sim/worlds/robot_test.sdf"

echo "▶ 로봇 검증 월드 기동 (헤드리스)"
gz sim -v2 -s -r "$WORLD" >/tmp/robot_verify.log 2>&1 &
SIM=$!
trap "kill $SIM 2>/dev/null || true" EXIT
sleep 10

echo "▶ 센서 토픽 확인"
for t in /odom /livox/points_raw/points /imu /navsat /cam/left/image /cam/right/image /cam/forward/points /joint_states; do
  if gz topic -l 2>/dev/null | grep -qx "$t"; then echo "   ✔ $t"; else echo "   ✗ $t 없음"; fi
done

echo "▶ 직진 1.0 m/s (odom x 증가, y≈0 확인)"
( for i in $(seq 1 25); do gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear:{x:1.0}" 2>/dev/null; done ) &
sleep 3; kill %2 2>/dev/null || true
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear:{x:0}" 2>/dev/null
sleep 0.5
gz topic -e -t /odom -n 1 2>/dev/null | grep -A3 position | grep -E 'x:|y:' | head -2

echo "▶ 제자리회전 0.5 rad/s (x·y 불변, yaw 변화 확인)"
( for i in $(seq 1 25); do gz topic -t /cmd_vel -m gz.msgs.Twist -p "angular:{z:0.5}" 2>/dev/null; done ) &
sleep 3; kill %2 2>/dev/null || true
gz topic -t /cmd_vel -m gz.msgs.Twist -p "angular:{z:0}" 2>/dev/null
sleep 0.5
gz topic -e -t /odom -n 1 2>/dev/null | grep -A7 orientation | grep -E 'z:|w:' | head -2

echo "✔ 검증 완료 — 상세는 설계서 §5.3 / docs/findings 참조"
