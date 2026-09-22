#!/usr/bin/env bash
# 학습 컨테이너(vip3-perception-train) 제어. 데이터는 읽기 전용으로 마운트한다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="$REPO_ROOT/docker/perception-train/docker-compose.yaml"

export VIP3="${VIP3:-$REPO_ROOT}"
export VIP3_DATA="${VIP3_DATA:-$(dirname "$REPO_ROOT")/data}"
export TWINLITE_SOURCE="${TWINLITE_SOURCE:-$(dirname "$REPO_ROOT")/external/baselines/TwinLiteNetPlus}"
mkdir -p "$VIP3_DATA"

if [ ! -f "$TWINLITE_SOURCE/model/model.py" ]; then
  echo "[FAIL] TwinLiteNet+ upstream 이 없다: $TWINLITE_SOURCE" >&2
  echo "  git clone https://github.com/chequanghuy/TwinLiteNetPlus.git \"$TWINLITE_SOURCE\"" >&2
  echo "  git -C \"$TWINLITE_SOURCE\" checkout 90f1b8695ae311d5123b05f8534b2e11e42499d2" >&2
  exit 1
fi

COMPOSE=(docker compose -f "$COMPOSE_FILE")
echo "[DATA]     $VIP3_DATA -> /data (ro)"
echo "[TWINLITE] $TWINLITE_SOURCE -> /opt/baselines/TwinLiteNetPlus (ro)"

case "${1:-up}" in
  build)   "${COMPOSE[@]}" build ;;
  up)      "${COMPOSE[@]}" up -d --force-recreate
           docker ps --filter name=vip3-perception-train ;;
  rebuild) "${COMPOSE[@]}" build --no-cache
           "${COMPOSE[@]}" up -d --force-recreate ;;
  shell)   docker exec -it vip3-perception-train bash ;;
  down)    "${COMPOSE[@]}" down ;;
  *) echo "사용법: $0 {build|up|rebuild|shell|down}"; exit 2 ;;
esac
