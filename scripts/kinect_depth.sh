#!/usr/bin/env bash
#
# Публикатор глубины Kinect 360 (POST /api/kinect/depth на бортовой 8090).
#
#   ./scripts/kinect_depth.sh                # в фоне, pid в var/kinect-depth.pid
#   ./scripts/kinect_depth.sh --foreground  # для systemd (процесс не фонится)
#   ./scripts/kinect_depth.sh --list-backends # диагностика: что установлено/нет
#
# Без установленного драйвера (freenect / openni2 / rclpy) НЕ запускается и
# НЕ подсовывает демо-кадр: иначе карта выглядела бы живой при мёртвом датчике.
# Список драйверов: ./scripts/kinect_depth_publisher.py --list-backends
#
# Остановка: ./scripts/stop.sh (или systemctl stop robot-kinect-depth).
#
set -euo pipefail
cd "$(dirname "$0")/.."

VAR_DIR="${RC_VAR_DIR:-var}"
PIDFILE="$VAR_DIR/kinect-depth.pid"
LOGFILE="$VAR_DIR/kinect-depth.log"
PY=".venv/bin/python"
[[ -x "$PY" ]] || PY="python3"
FOREGROUND=0
[[ "${1:-}" == "--foreground" ]] && FOREGROUND=1

log() { printf '\033[1;32m[kinect]\033[0m %s\n' "$*"; }
mkdir -p "$VAR_DIR"

# Явная диагностика работает даже когда публикатор уже запущен: показываем
# все три бэкенда, а не только «ничего не найдено».
if [[ "${1:-}" == "--list-backends" ]]; then
  exec "$PY" scripts/kinect_depth_publisher.py --list-backends
fi

if [[ "$FOREGROUND" -eq 0 && -f "$PIDFILE" ]]; then
  pid="$(cat "$PIDFILE" 2>/dev/null || true)"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    log "публикатор уже работает (pid $pid)."
    exit 0
  fi
fi

# Проверяем драйверы ДО запуска: --source auto сам честно упадёт без них,
# но тогда systemd будет бесконечно перезапускать заведомо мёртвый процесс.
INSTALLED="$($PY scripts/kinect_depth_publisher.py --list-backends \
  | $PY -c 'import json,sys; print(",".join(json.load(sys.stdin).get("installed", [])))')"
if [[ -z "$INSTALLED" ]]; then
  log "драйверы Kinect не найдены — публикатор не запущен. См. requirements-kinect.txt."
  exit 0
fi

# rclpy виден только из окружения ROS 2; libfreenect/openni2 не требуют source.
if [[ -n "${RC_ROS_PREFIX:-}" && -f "$RC_ROS_PREFIX/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$RC_ROS_PREFIX/setup.bash" || true
fi

if [[ "$FOREGROUND" -eq 1 ]]; then
  log "публикатор глубины: $INSTALLED (foreground)."
  exec "$PY" scripts/kinect_depth_publisher.py --source auto
fi

"$PY" scripts/kinect_depth_publisher.py --source auto >> "$LOGFILE" 2>&1 &
pid=$!
echo "$pid" > "$PIDFILE"
log "публикатор глубины запущен: драйверы $INSTALLED, pid $pid, лог $LOGFILE."
