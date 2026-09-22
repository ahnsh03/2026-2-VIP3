#!/usr/bin/env bash
# roscore + rosbridge_server(WebSocket, 기본 9090) 기동.
# VIP3 의 MORAI 전송 경로는 이것 하나뿐이다. UDP 브리지는 없다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=vip3_env.sh
source "$SCRIPT_DIR/vip3_env.sh"

cd "$VIP3_WS_ROOT"
source /opt/ros/noetic/setup.bash
source devel/setup.bash 2>/dev/null || true

[ -f "$MORAI_ROSBRIDGE_ENV" ] && source "$MORAI_ROSBRIDGE_ENV"

BRIDGE_PORT="${MORAI_BRIDGE_PORT:-9090}"
BRIDGE_BIND="${MORAI_BRIDGE_BIND:-0.0.0.0}"

export ROS_MASTER_URI="http://127.0.0.1:11311"
export ROS_IP="${ROS_IP:-127.0.0.1}"

# morai_msgs 26.R1 판별.
#   26.R1      : EgoVehicleStatus.angular_velocity / front_steer_angle / rear_steer_angle,
#                CtrlCmd.front_steer / rear_steer
#   beta_drive : EgoVehicleStatus.wheel_angle, CtrlCmd.steering  (VIP3 에서는 오류)
_msgs_is_26r1() {
  local ego ctrl
  ego="$(rosmsg show morai_msgs/EgoVehicleStatus 2>/dev/null || true)"
  ctrl="$(rosmsg show morai_msgs/CtrlCmd 2>/dev/null || true)"
  grep -q 'angular_velocity' <<<"$ego" \
    && grep -q 'front_steer_angle' <<<"$ego" \
    && grep -q 'rear_steer_angle' <<<"$ego" \
    && grep -q 'front_steer' <<<"$ctrl" \
    && ! grep -q 'wheel_angle' <<<"$ego" \
    && ! grep -qE '(^|[[:space:]])steering([[:space:]]|$)' <<<"$ctrl"
}

_ensure_morai_msgs() {
  if rospack find morai_msgs >/dev/null 2>&1 && _msgs_is_26r1; then
    echo "[rosbridge] morai_msgs 26.R1 확인"
    return 0
  fi
  echo "[rosbridge] morai_msgs 없음 또는 버전 불일치 — 빌드 시도"
  source /opt/ros/noetic/setup.bash
  catkin_make --pkg morai_msgs
  source devel/setup.bash
  if ! rospack find morai_msgs >/dev/null 2>&1; then
    echo "[rosbridge] ERROR: morai_msgs 빌드 실패 → ./scripts/build_ws.sh"
    exit 1
  fi
  if ! _msgs_is_26r1; then
    echo "[rosbridge] ERROR: morai_msgs 가 26.R1 이 아니다."
    echo "  기대: EgoVehicleStatus.angular_velocity / front_steer_angle / rear_steer_angle"
    echo "        CtrlCmd.front_steer / rear_steer  (beta_drive 의 wheel_angle·steering 아님)"
    echo "  확인: rosmsg show morai_msgs/EgoVehicleStatus | grep -E 'angular_velocity|steer'"
    echo "  수정: git -C src/morai_msgs fetch origin 26.R1"
    echo "        git -C src/morai_msgs checkout 4c9be6f"
    echo "        ./scripts/build_ws.sh"
    exit 1
  fi
  echo "[rosbridge] morai_msgs 26.R1 (4c9be6f) 준비 완료"
}

_ensure_rosbridge() {
  if rospack find rosbridge_server >/dev/null 2>&1; then
    return 0
  fi
  echo "[rosbridge] rosbridge_server 없음 — 설치 시도 (이미지에 이미 있어야 정상)"
  apt-get update -qq
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-rosbridge-server \
    ros-noetic-rosbridge-suite \
    >/dev/null
  source /opt/ros/noetic/setup.bash
  if ! rospack find rosbridge_server >/dev/null 2>&1; then
    echo "[rosbridge] ERROR: rosbridge 설치 실패."
    exit 1
  fi
  echo "[rosbridge] rosbridge 설치 완료."
}

echo "=========================================="
echo "  MORAI rosbridge (WebSocket)  — VIP3"
echo "=========================================="
echo "  VIP3_WS_ROOT   : $VIP3_WS_ROOT"
echo "  ROS_MASTER_URI : $ROS_MASTER_URI"
echo "  Listening      : ${BRIDGE_BIND}:${BRIDGE_PORT}"
echo "  morai_msgs     : 26.R1 (4c9be6f)"
echo ""
echo "  MORAI(Windows) 설정:"
echo "    Network Setup  : ROS,  ws://127.0.0.1:${BRIDGE_PORT}  → Connect"
echo "    Sensor  Setup  : VIP3_sensor_set_v1 (전 센서 ROS, 토픽 명시)"
echo "    Ego Ctrl       : /ctrl_cmd  (morai_msgs/CtrlCmd, front_steer 사용)"
echo "    Map/Scenario   : R_KR_PG_KATRI"
echo ""
echo "  검증: ./scripts/verify_morai_topics.py --duration 8"
echo "=========================================="
echo ""

_ensure_rosbridge
_ensure_morai_msgs

if ! rostopic list >/dev/null 2>&1; then
  echo "[rosbridge] roscore 기동 중..."
  roscore &
  ROSCORE_PID=$!
  for _ in $(seq 1 15); do
    rostopic list >/dev/null 2>&1 && break
    sleep 1
  done
  if ! rostopic list >/dev/null 2>&1; then
    echo "[rosbridge] ERROR: roscore 기동 실패."
    kill "$ROSCORE_PID" 2>/dev/null || true
    exit 1
  fi
  echo "[rosbridge] roscore 준비 완료."
fi

echo "rosbridge 기동 (port ${BRIDGE_PORT})..."
exec roslaunch rosbridge_server rosbridge_websocket.launch \
  port:="${BRIDGE_PORT}" \
  address:="${BRIDGE_BIND}" \
  max_message_size:="${MORAI_ROSBRIDGE_MAX_MESSAGE_SIZE:-100000000}" \
  fragment_timeout:="${MORAI_ROSBRIDGE_FRAGMENT_TIMEOUT:-60}" \
  unregister_timeout:="${MORAI_ROSBRIDGE_UNREGISTER_TIMEOUT:-10}"
