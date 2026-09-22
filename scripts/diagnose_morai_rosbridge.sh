#!/usr/bin/env bash
# MORAI rosbridge 연동 상태를 한 번에 훑는다. 증상 보고할 때 이 출력을 붙인다.
set -uo pipefail

source /opt/ros/noetic/setup.bash
[ -f /root/ws/devel/setup.bash ] && source /root/ws/devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
PORT="${MORAI_BRIDGE_PORT:-9090}"

line() { printf '%s\n' "------------------------------------------------------------"; }

line; echo "1. ROS master"
if rostopic list >/dev/null 2>&1; then
  echo "   OK  $ROS_MASTER_URI"
else
  echo "   FAIL  roscore 없음 -> ./scripts/rosbridge.sh"
  exit 1
fi

line; echo "2. rosbridge listen (${PORT})"
ss -ltn 2>/dev/null | grep -q ":${PORT} " \
  && echo "   OK  ${PORT} listen 중" \
  || echo "   FAIL  ${PORT} 를 아무도 안 듣는다 -> ./scripts/rosbridge.sh"

line; echo "3. morai_msgs 브랜치 (26.R1 이어야 한다)"
if rosmsg show morai_msgs/EgoVehicleStatus >/dev/null 2>&1; then
  if rosmsg show morai_msgs/EgoVehicleStatus | grep -q 'angular_velocity'; then
    echo "   OK  26.R1 (angular_velocity 있음)"
  else
    echo "   FAIL  beta_drive 계열이다. git submodule 을 26.R1 로 맞춰라"
  fi
  rosmsg show morai_msgs/CtrlCmd | grep -E 'front_steer|steering' | sed 's/^/        /'
else
  echo "   FAIL  morai_msgs 가 빌드되지 않았다 -> ./scripts/build_ws.sh"
fi

line; echo "4. 토픽"
for t in /Ego_topic /Object_topic /CollisionData /tf \
         /cam_front/image_jpeg/compressed /cam_left/image_jpeg/compressed \
         /cam_right/image_jpeg/compressed /cam_rear/image_jpeg/compressed \
         /velodyne_points /gps /imu; do
  if rostopic list 2>/dev/null | grep -qx "$t"; then
    printf '   %-40s advertised\n' "$t"
  else
    printf '   %-40s MISSING\n' "$t"
  fi
done

line; echo "5. 제어 채널"
rostopic list 2>/dev/null | grep -qx /ctrl_cmd \
  && echo "   OK  /ctrl_cmd advertised" \
  || echo "   MISSING  /ctrl_cmd — MORAI Network Settings 의 Ego Ctrl Cmd 를 ROS 로 켜야 한다 (docs/simulator.md §4)"

line; echo "6. 서비스"
for s in /Service_MoraiEventCmd /Service_MoraiMapSpec; do
  rosservice list 2>/dev/null | grep -qx "$s" \
    && printf '   %-30s OK\n' "$s" \
    || printf '   %-30s MISSING\n' "$s"
done

line; echo "7. 실제 수신율은 다음으로 본다"
echo "   python3 scripts/verify_morai_topics.py --duration 30"
line
