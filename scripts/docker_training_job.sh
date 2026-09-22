#!/usr/bin/env bash
# 학습을 호출한 터미널과 분리해서 돌린다.
# VS Code 터미널이나 WSL 원격 연결이 끊겨도 학습이 죽지 않게 하는 것이 목적이다.
set -euo pipefail

CONTAINER="${VIP3_TRAIN_CONTAINER:-vip3-perception-train}"
ACTION="${1:-}"
JOB_NAME="${2:-}"

usage() {
  echo "사용법:"
  echo "  $0 start  <job-name> -- <command> [args...]"
  echo "  $0 status <job-name>"
  echo "  $0 logs   <job-name> [줄수]"
  echo "  $0 list"
}

if [[ "$ACTION" == "list" ]]; then
  docker exec "$CONTAINER" bash -lc \
    "ls -1 /root/ws/artifacts/perception_eval/jobs 2>/dev/null || echo '(job 없음)'"
  exit 0
fi

if [[ ! "$JOB_NAME" =~ ^[a-zA-Z0-9][a-zA-Z0-9._-]*$ ]]; then
  usage
  exit 2
fi

JOB_DIR="/root/ws/artifacts/perception_eval/jobs/$JOB_NAME"

case "$ACTION" in
  start)
    shift 2
    [[ "${1:-}" == "--" ]] && shift
    [[ "$#" -eq 0 ]] && { usage; exit 2; }
    docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true \
      || { echo "[ERROR] 컨테이너가 안 떠 있다: $CONTAINER"; exit 1; }
    docker exec "$CONTAINER" bash -lc "test ! -e '$JOB_DIR'" \
      || { echo "[ERROR] 같은 이름의 job 이 이미 있다: $JOB_NAME"; exit 1; }
    quoted=""
    for argument in "$@"; do
      printf -v q '%q' "$argument"
      quoted+="$q "
    done
    docker exec "$CONTAINER" bash -lc \
      "mkdir -p '$JOB_DIR' && printf '%s\n' '$quoted' > '$JOB_DIR/command.txt'"
    docker exec -d "$CONTAINER" bash -lc "
      cd /root/ws
      echo \$\$ > '$JOB_DIR/pid'
      date -Iseconds > '$JOB_DIR/started_at'
      set +e
      $quoted > '$JOB_DIR/console.log' 2>&1
      code=\$?
      printf '%s\n' \"\$code\" > '$JOB_DIR/exit_code'
      date -Iseconds > '$JOB_DIR/finished_at'
      exit \"\$code\"
    "
    echo "[STARTED] $JOB_NAME"
    echo "[LOG] artifacts/perception_eval/jobs/$JOB_NAME/console.log"
    ;;
  status)
    docker exec "$CONTAINER" bash -lc "
      if [[ -f '$JOB_DIR/exit_code' ]]; then
        echo \"completed exit_code=\$(cat '$JOB_DIR/exit_code')\"
      elif [[ -f '$JOB_DIR/pid' ]] && kill -0 \"\$(cat '$JOB_DIR/pid')\" 2>/dev/null; then
        echo \"running pid=\$(cat '$JOB_DIR/pid')\"
      elif [[ -d '$JOB_DIR' ]]; then
        echo interrupted_without_exit_code
      else
        echo job_not_found; exit 1
      fi
    "
    ;;
  logs)
    LINES="${3:-40}"
    [[ "$LINES" =~ ^[1-9][0-9]*$ ]] || { echo "[ERROR] 줄수는 양의 정수"; exit 2; }
    docker exec "$CONTAINER" bash -lc "tail -n '$LINES' '$JOB_DIR/console.log'"
    ;;
  *) usage; exit 2 ;;
esac
