#!/usr/bin/env bash
# ROS Noetic 개발 컨테이너 빌드·기동·점검.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="$REPO_ROOT/docker/ros-noetic/docker-compose.yaml"
HOSTNET_FILE="$REPO_ROOT/docker/ros-noetic/docker-compose.hostnet.yaml"
RTX50_FILE="$REPO_ROOT/docker/ros-noetic/docker-compose.rtx50.yaml"
CONTAINER="vip3-ros-noetic"
BRIDGE_PORT="${MORAI_BRIDGE_PORT:-9090}"

export VIP3="${VIP3:-$REPO_ROOT}"
export VIP3_DATA="${VIP3_DATA:-$(dirname "$REPO_ROOT")/data}"
export TWINLITE_SOURCE="${TWINLITE_SOURCE:-$(dirname "$REPO_ROOT")/external/baselines/TwinLiteNetPlus}"
mkdir -p "$VIP3_DATA"

COMPOSE=(docker compose -f "$COMPOSE_FILE")
COMPOSE_HOSTNET=(docker compose -f "$COMPOSE_FILE" -f "$HOSTNET_FILE")
COMPOSE_RTX50=(docker compose -f "$COMPOSE_FILE" -f "$RTX50_FILE")

die() { echo "[FAIL] $*" >&2; exit 1; }

require_running() {
  docker inspect "$CONTAINER" >/dev/null 2>&1 \
    || die "$CONTAINER 가 없다. 먼저: $0 up"
  [ "$(docker inspect "$CONTAINER" --format '{{.State.Running}}')" = "true" ] \
    || die "$CONTAINER 가 멈춰 있다. 먼저: $0 up"
}

show_status() {
  require_running
  docker inspect "$CONTAINER" --format \
    'mode={{.HostConfig.NetworkMode}} published={{json .NetworkSettings.Ports}}'
}

# MORAI 는 Windows 에서 WebSocket 클라이언트로 붙는다. 따라서 확인할 것은
# "컨테이너의 9090 이 호스트에서 보이는가" 하나다.
check_bridge_port() {
  require_running
  local mode
  mode="$(docker inspect "$CONTAINER" --format '{{.HostConfig.NetworkMode}}')"
  if [ "$mode" = "host" ]; then
    echo "[OK] host network mode — 컨테이너 포트가 곧 호스트 포트다"
  else
    docker inspect "$CONTAINER" --format '{{json .NetworkSettings.Ports}}' \
      | grep -q "\"${BRIDGE_PORT}/tcp\"" \
      || die "${BRIDGE_PORT}/tcp 가 퍼블리시되지 않았다. $0 up 을 다시 실행"
    echo "[OK] ${BRIDGE_PORT}/tcp published"
  fi
  if docker exec "$CONTAINER" bash -lc "ss -ltn 2>/dev/null | grep -q ':${BRIDGE_PORT} '"; then
    echo "[OK] 컨테이너 안에서 ${BRIDGE_PORT} listen 중 (rosbridge 기동됨)"
  else
    echo "[INFO] 아직 ${BRIDGE_PORT} 를 listen 하지 않는다 — 컨테이너에서 ./scripts/rosbridge.sh 실행"
  fi
  if command -v powershell.exe >/dev/null 2>&1; then
    powershell.exe -NoProfile -Command \
      "try { \$c=New-Object Net.Sockets.TcpClient; \$c.Connect('127.0.0.1',$BRIDGE_PORT); \$c.Close(); exit 0 } catch { exit 3 }" \
      >/dev/null 2>&1 \
      && echo "[OK] Windows 127.0.0.1:${BRIDGE_PORT} 도달 — MORAI 가 붙을 수 있다" \
      || echo "[INFO] Windows 에서 아직 도달 안 됨 — rosbridge 를 먼저 띄운 뒤 다시 확인"
  fi
}

verify_topics() {
  require_running
  docker exec "$CONTAINER" bash -lc \
    "cd /root/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash 2>/dev/null; \
     python3 scripts/verify_morai_topics.py --duration ${1:-15}"
}

next_steps() {
  cat <<'EOF'

[다음] 컨테이너 터미널에서 rosbridge 를 띄우고 MORAI 를 연결한다.
  docker exec -it vip3-ros-noetic bash
  cd /root/ws && ./scripts/build_ws.sh && source devel/setup.bash
  ./scripts/rosbridge.sh                 # 포그라운드 유지

MORAI (Windows):
  Map            R_KR_PG_KATRI
  Network        config/VIP3_network_v1.json  Import -> Connect
  Sensor         config/VIP3_sensor_set_v1_ros.json  Import -> Connect
  Ego Ctrl Cmd   ROS / /ctrl_cmd      <-- 아직 프리셋에 없다. docs/simulator.md §4
  Play

확인:  ./scripts/docker_ros_up.sh verify 15
EOF
}

cmd="${1:-up}"
echo "[DATA]     $VIP3_DATA -> /data (rw)"
echo "[TWINLITE] $TWINLITE_SOURCE -> /opt/baselines/TwinLiteNetPlus (ro)"
case "$cmd" in
  config)   "${COMPOSE[@]}" config ;;
  build)    "${COMPOSE[@]}" build ;;
  up)       "${COMPOSE[@]}" up -d --force-recreate; show_status; next_steps ;;
  up-hostnet)
            "${COMPOSE_HOSTNET[@]}" up -d --force-recreate; show_status; next_steps ;;
  rebuild)  "${COMPOSE[@]}" build --no-cache
            "${COMPOSE[@]}" up -d --force-recreate; show_status ;;
  build-rtx50)
            "${COMPOSE[@]}" build
            "${COMPOSE_RTX50[@]}" build ;;
  up-rtx50) "${COMPOSE_RTX50[@]}" up -d --force-recreate; show_status; next_steps ;;
  status)   show_status ;;
  check)    check_bridge_port ;;
  verify)   verify_topics "${2:-15}" ;;
  shell)    require_running; docker exec -it "$CONTAINER" bash ;;
  down)     "${COMPOSE[@]}" down ;;
  *)
    echo "사용법: $0 {build|up|up-hostnet|rebuild|build-rtx50|up-rtx50|status|check|verify [초]|shell|down|config}"
    exit 2
    ;;
esac
