#!/usr/bin/env bash
# 시뮬레이션 관련 프로세스 일괄 종료
# (pkill -f 는 자기 자신도 매칭해 셸을 죽이므로 args 필터 후 PID 로 종료한다)
PIDS=$(ps -eo pid,args \
  | grep -E 'gz sim|gz-sim|parameter_bridge|livox_sim_bridge|gt_localizer|sdf_static_tf|rviz2|ros2 launch orchard_sim|teleop_twist' \
  | grep -v 'grep\|bash -c\|stop_all' | awk '{print $1}')
if [ -z "$PIDS" ]; then echo "종료할 프로세스 없음"; exit 0; fi
echo "$PIDS" | xargs -r kill 2>/dev/null
sleep 2
LEFT=$(ps -eo pid,args | grep -E 'gz sim|parameter_bridge|rviz2' | grep -v 'grep\|stop_all' | awk '{print $1}')
[ -n "$LEFT" ] && echo "$LEFT" | xargs -r kill -9 2>/dev/null
echo "정리 완료"
