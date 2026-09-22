#!/usr/bin/env bash
# 토픽이 실제로 메시지를 뿌릴 때까지 기다린다. rostopic list 에 이름만 떠도
# MORAI 가 Play 상태가 아니면 메시지가 안 온다 — 그래서 list 가 아니라 echo 로 본다.
#   ./scripts/wait_morai_topic.sh /Ego_topic 120
set -euo pipefail

TOPIC="${1:-/Ego_topic}"
TIMEOUT="${2:-120}"

source /opt/ros/noetic/setup.bash
[ -f /root/ws/devel/setup.bash ] && source /root/ws/devel/setup.bash

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

if ! rostopic list >/dev/null 2>&1; then
  echo "[FAIL] roscore 에 붙지 못했다. ./scripts/rosbridge.sh 를 먼저 띄운다." >&2
  exit 1
fi

echo "[WAIT] $TOPIC (최대 ${TIMEOUT}s)"
deadline=$(( SECONDS + TIMEOUT ))
while (( SECONDS < deadline )); do
  if timeout 2 rostopic echo -n1 "$TOPIC" >/dev/null 2>&1; then
    echo "[OK] $TOPIC 수신"
    rostopic hz "$TOPIC" -w 20 &
    HZ_PID=$!
    sleep 5
    kill "$HZ_PID" 2>/dev/null || true
    exit 0
  fi
  sleep 1
done

echo "[FAIL] ${TIMEOUT}s 안에 $TOPIC 를 못 받았다." >&2
echo "  1) rosbridge 가 떠 있는가         ss -ltn | grep 9090" >&2
echo "  2) MORAI Network 가 Connect 됐는가" >&2
echo "  3) MORAI 가 Play 상태인가" >&2
echo "  4) 센서 토픽이면 센서셋이 ROS(commType 3)로 Import 됐는가" >&2
echo "  자세한 것은 docs/simulator.md §6" >&2
exit 1
