#!/usr/bin/env bash
# VIP3 workspace root (호스트 clone 경로 또는 Docker 안 /root/ws).
# 다른 스크립트에서: source "$(dirname "$0")/vip3_env.sh"
if [ -n "${VIP3:-}" ] && [ -d "${VIP3}/src" ]; then
  export VIP3_WS_ROOT="$(cd "${VIP3}" && pwd)"
elif [ -d /root/ws/src ]; then
  export VIP3_WS_ROOT=/root/ws
else
  export VIP3_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

# 저장소 밖 데이터 루트 (기본: 저장소와 나란한 ../data)
export VIP3_DATA="${VIP3_DATA:-$(cd "$(dirname "$VIP3_WS_ROOT")" && pwd)/data}"

# MORAI 전송은 rosbridge(WebSocket) 하나뿐이다. UDP 브리지는 쓰지 않는다.
# 이름에 'BRIDGE' 만 쓰면 UDP 브리지로 오해하는 사고가 ASMC 에서 실제로 있었다.
export MORAI_ROSBRIDGE_ENV="${VIP3_WS_ROOT}/config/morai_rosbridge.env"
# 포트·URL 을 한 곳에서만 정한다. 런치는 $(optenv MORAI_BRIDGE_URL ...) 로 읽으므로
# 여기서 실제로 읽어 들여야 config 변경이 roslaunch 까지 전달된다.
# shellcheck disable=SC1090
[ -f "$MORAI_ROSBRIDGE_ENV" ] && . "$MORAI_ROSBRIDGE_ENV"
