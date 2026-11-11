#!/usr/bin/env bash
# ROS Noetic 개발 컨테이너 빌드·기동·점검.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="$REPO_ROOT/docker/ros-noetic/docker-compose.yaml"
HOSTNET_FILE="$REPO_ROOT/docker/ros-noetic/docker-compose.hostnet.yaml"
RTX50_FILE="$REPO_ROOT/docker/ros-noetic/docker-compose.rtx50.yaml"
export VIP3_CONTAINER_NAME="${VIP3_CONTAINER_NAME:-vip3-parking-ros}"
export VIP3_IMAGE="${VIP3_IMAGE:-vip3/parking-ros:noetic}"
CONTAINER="$VIP3_CONTAINER_NAME"
BRIDGE_PORT="${MORAI_BRIDGE_PORT:-9090}"

export VIP3="${VIP3:-$REPO_ROOT}"
export VIP3_DATA="${VIP3_DATA:-$(dirname "$REPO_ROOT")/data}"
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
  local mode actual_port
  actual_port="$(docker exec "$CONTAINER" printenv MORAI_BRIDGE_PORT 2>/dev/null || true)"
  BRIDGE_PORT="${actual_port:-$BRIDGE_PORT}"
  mode="$(docker inspect "$CONTAINER" --format '{{.HostConfig.NetworkMode}}')"
  if [ "$mode" = "host" ]; then
    echo "[INFO] host network mode — Docker Desktop에서는 Windows 도달 여부를 따로 확인"
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
  cat <<EOF

[다음] 컨테이너 터미널에서 rosbridge 를 띄우고 MORAI 를 연결한다.
  docker exec -it ${CONTAINER} bash
  cd /root/ws && ./scripts/build_ws.sh && source devel/setup.bash
  ./scripts/rosbridge.sh                 # 포그라운드 유지

MORAI (Windows):
  Map            R_KR_PG_KATRI
  Network        config/morai/261110_VIP3_Network_26.R1.H3.json  Import -> Connect
  Sensor         config/morai/261110_VIP3_Sensor_26.R1.H3.json  Import -> Connect
  Ego Ctrl Cmd   ROS / /ctrl_cmd (프리셋에 포함)
  Local port     MORAI_BRIDGE_PORT 변경 시 프리셋의 접속 포트도 동일하게 변경
  Play

확인:  ./scripts/docker_ros_up.sh verify 15
EOF
}

cmd="${1:-help}"
echo "[DATA]     $VIP3_DATA -> /data (rw)"
echo "[IMAGE]    $VIP3_IMAGE"
echo "[NAME]     $CONTAINER"
case "$cmd" in
  config)   "${COMPOSE[@]}" config ;;
  build)    "${COMPOSE[@]}" build ;;
  up)       "${COMPOSE[@]}" up -d --no-build; show_status; next_steps ;;
  up-hostnet)
            "${COMPOSE_HOSTNET[@]}" up -d --force-recreate; show_status; next_steps ;;
  rebuild)  "${COMPOSE[@]}" build --no-cache ;;
  build-rtx50)
            docker build -t vip3/ros-noetic:dev -f "$REPO_ROOT/docker/ros-noetic/Dockerfile.legacy-twinlite" "$REPO_ROOT/docker/ros-noetic"
            "${COMPOSE_RTX50[@]}" build ;;
  up-rtx50) "${COMPOSE_RTX50[@]}" up -d --force-recreate; show_status; next_steps ;;
  status)   show_status ;;
  check)    check_bridge_port ;;
  verify)   verify_topics "${2:-15}" ;;
  shell)    require_running; docker exec -it "$CONTAINER" bash ;;
  build-ws) require_running; docker exec -it "$CONTAINER" bash -lc './scripts/build_ws.sh -j2 -l2' ;;
  bridge)   require_running; docker exec -it "$CONTAINER" bash -lc './scripts/rosbridge.sh' ;;
  svm)      require_running; docker exec -it "$CONTAINER" bash -lc 'source devel/setup.bash && exec roslaunch drivable_bev fisheye_svm.launch' ;;
  view)     require_running; docker exec -it "$CONTAINER" vip3-ros-gui rqt-image /perception/svm/panel ;;
  stop)     "${COMPOSE[@]}" stop ;;
  help|-h|--help)
    echo "빌드만: $0 build | rebuild (컨테이너 생성 없음)"
    echo "직접 실행: $0 up -> build-ws -> bridge / svm / view (각 터미널)"
    echo "관리: $0 config | status | check | verify [초] | shell | stop | down"
    echo "이름 변경: export VIP3_CONTAINER_NAME=원하는이름"
    ;;
  down)     "${COMPOSE[@]}" down ;;
  *)
    echo "사용법: $0 {build|up|up-hostnet|rebuild|build-rtx50|up-rtx50|status|check|verify [초]|shell|down|config}"
    exit 2
    ;;
esac
