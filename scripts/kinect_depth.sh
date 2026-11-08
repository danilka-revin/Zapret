#!/usr/bin/env bash
#
# Публикатор глубины Kinect 360 (POST /api/kinect/depth на бортовой 8090).
#
#   ./scripts/kinect_depth.sh                # в фоне, pid в var/kinect-depth.pid
#   ./scripts/kinect_depth.sh --foreground  # для systemd (процесс не фонится)
#   ./scripts/kinect_depth.sh --list-backends # диагностика: что установлено/нет
#
# Требует выбранный RC_KINECT_PORT, libfreenect и PyUSB/libusb; иначе
# успешно завершится без потока, чтобы systemd не перезапускал пустой сервис.
# Список доступных драйверов: ./scripts/kinect_depth_publisher.py --list-backends
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
read_env() {
  local key="$1" value
  value="$(printenv "$key" 2>/dev/null || true)"
  if [[ -z "$value" ]]; then
    value="$(grep -E "^${key}=" .env 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
  fi
  value="${value%\"}"; value="${value#\"}"
  printf '%s' "$value"
}
for key in RC_KINECT_PORT RC_ROS_PREFIX; do
  if [[ -z "${!key:-}" ]]; then
    value="$(read_env "$key")"
    [[ -n "$value" ]] && export "$key=$value"
  fi
done
unset key value
FOREGROUND=0
[[ "${1:-}" == "--foreground" ]] && FOREGROUND=1

log() { printf '\033[1;32m[kinect]\033[0m %s\n' "$*"; }
mkdir -p "$VAR_DIR"

# rclpy виден только внутри окружения ROS 2. Подключаем его ДО проверки
# драйверов, включая --list-backends, иначе ROS-бэкенд всегда выглядел бы отсутствующим.
if [[ -z "${RC_ROS_PREFIX:-}" ]]; then
  for setup in /opt/ros/*/setup.bash; do
    [[ -f "$setup" ]] || continue
    RC_ROS_PREFIX="$(dirname "$setup")"
    export RC_ROS_PREFIX
    break
  done
fi
if [[ -n "${RC_ROS_PREFIX:-}" && -f "$RC_ROS_PREFIX/setup.bash" ]]; then
  # ROS Rolling setup can read unset variables; source it outside nounset mode.
  # shellcheck disable=SC1091
  set +u
  source "$RC_ROS_PREFIX/setup.bash" || log "ROS environment could not be sourced; continuing without it."
  set -u
fi

# Явная диагностика работает даже когда публикатор уже запущен: показываем
# все три бэкенда, а не только «ничего не найдено».
if [[ "${1:-}" == "--list-backends" ]]; then
  exec "$PY" scripts/kinect_depth_publisher.py --list-backends
fi

if [[ "$FOREGROUND" -eq 0 && -f "$PIDFILE" ]]; then
  pid="$(cat "$PIDFILE" 2>/dev/null || true)"
  running_port="$(cat "$PIDFILE.port" 2>/dev/null || true)"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    command="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    if [[ "$command" == *scripts/kinect_depth_publisher.py* ]]; then
      if [[ -n "${RC_KINECT_PORT:-}" && "$running_port" == "$RC_KINECT_PORT" ]]; then
        log "публикатор уже работает (pid $pid, $running_port)."
        exit 0
      fi
      log "порт Kinect изменился или сброшен; останавливаю старый publisher (pid $pid)."
      kill "$pid" 2>/dev/null || true
      for _ in {1..20}; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
      done
      kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
    else
      log "устаревший PID-файл Kinect; процесс $pid не затрагиваю."
    fi
  fi
  rm -f "$PIDFILE" "$PIDFILE.port"
fi

if [[ -z "${RC_KINECT_PORT:-}" ]]; then
  log "NUI Camera не выбран во вкладке «Устройства» (RC_KINECT_PORT); глубину не запускаю."
  exit 0
fi

# Проверяем драйверы ДО запуска: --source auto сам честно упадёт без них,
# но тогда systemd будет бесконечно перезапускать заведомо мёртвый процесс.
INSTALLED="$($PY scripts/kinect_depth_publisher.py --list-backends \
  | $PY -c 'import json,sys; print(",".join(json.load(sys.stdin).get("installed", [])))')"
if [[ ",${INSTALLED}," != *,freenect,* ]]; then
  log "для привязки RC_KINECT_PORT нужен backend freenect (найдены: ${INSTALLED:-нет}). См. requirements-kinect.txt."
  exit 0
fi
if ! "$PY" -c 'import usb.core, usb.backend.libusb1; assert usb.backend.libusb1.get_backend() is not None' >/dev/null 2>&1; then
  log "не найден PyUSB/libusb-1.0 backend: установите pyusb и libusb-1.0, иначе порт нельзя сопоставить с индексом libfreenect."
  exit 0
fi

if [[ "$FOREGROUND" -eq 1 ]]; then
  log "публикатор глубины: $INSTALLED (foreground)."
  exec "$PY" scripts/kinect_depth_publisher.py --source auto
fi

"$PY" scripts/kinect_depth_publisher.py --source auto >> "$LOGFILE" 2>&1 &
pid=$!
echo "$pid" > "$PIDFILE"
printf '%s\n' "$RC_KINECT_PORT" > "$PIDFILE.port"
log "публикатор глубины запущен: драйверы $INSTALLED, pid $pid, порт $RC_KINECT_PORT, лог $LOGFILE."
