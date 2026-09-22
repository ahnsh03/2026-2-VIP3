#!/usr/bin/env bash
# ROS1 catkin workspace 빌드.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."

if [[ ! -d src/morai_msgs && ! -L src/morai_msgs ]]; then
  echo "[build_ws] ERROR: src/morai_msgs 가 없습니다."
  echo "  git submodule update --init --recursive   (docs/setup.md §2 참고)"
  exit 1
fi

# morai_msgs 는 26.R1 브랜치의 이 커밋에 핀되어 있다. beta_drive(45c6baf)와는
# EgoVehicleStatus / CtrlCmd 스키마가 다르다 — 섞이면 조용히 잘못된 값이 나온다.
_pin="4c9be6f"
_head="$(git -C src/morai_msgs rev-parse --short=7 HEAD 2>/dev/null || echo unknown)"
if [[ "$_head" != "$_pin" ]]; then
  echo "[build_ws] WARN: morai_msgs HEAD=$_head, 기대 26.R1 pin=$_pin"
  echo "  git -C src/morai_msgs fetch origin 26.R1"
  echo "  git -C src/morai_msgs checkout $_pin"
fi

# bash -u: noetic setup 을 source 할 때 ROS_MASTER_URI 가 비어 있을 수 있다
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
catkin_make "$@"
echo "[build_ws] done. source devel/setup.bash"
