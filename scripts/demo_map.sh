#!/usr/bin/env bash
# Интерфейс без робота одной командой: бортовой экран (8090) + синтетический
# скан комнаты, чтобы карта и планировщик маршрута были видны в браузере.
#
#   ./scripts/demo_map.sh            # экран на 0.0.0.0:8090 и демо-скан
#   ./scripts/demo_map.sh --port 8091
#
# Это демонстрация интерфейса: настоящего лидара, шасси и E-STOP здесь нет.
# Точки помечаются как API-скан, поэтому автономная тяга («Поехать») остаётся
# заблокированной — двигать нечего. Для работы с реальным роботом используйте
# ./run_kiosk.py (борт) и ./scripts/serve.sh (пульт оператора).
set -euo pipefail
cd "$(dirname "$0")/.."

log() { printf '\033[1;32m[демо]\033[0m %s\n' "$*"; }

PORT="${RC_KIOSK_PORT:-8090}"
HOST="0.0.0.0"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    -h|--help) sed -n '1,16p' "$0"; exit 0 ;;
    *) echo "неизвестный аргумент: $1" >&2; exit 2 ;;
  esac
done

./scripts/ensure_deps.sh

# Ни ROS 2, ни автопоиск USB здесь не нужны: робота нет, порты не трогаем.
export RC_SLAM_SOURCE=none
export RC_ROS_AUTO=0
export RC_LIDAR_AUTOSTART=0
export RC_KIOSK_OPEN=0
export RC_KIOSK_PORT="$PORT"

KIOSK_PID=""
SCAN_PID=""
cleanup() {
  [[ -n "$SCAN_PID" ]] && kill "$SCAN_PID" 2>/dev/null || true
  [[ -n "$KIOSK_PID" ]] && kill "$KIOSK_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if curl -fsS -o /dev/null --max-time 1 "http://127.0.0.1:${PORT}/api/lidar"; then
  log "экран уже запущен на порту ${PORT} — второй не поднимаю"
else
  log "поднимаю бортовой экран на ${HOST}:${PORT} (источник: none)"
  .venv/bin/python run_kiosk.py --source none --no-open --no-watch \
    --host "$HOST" --port "$PORT" > var/demo-kiosk.log 2>&1 &
  KIOSK_PID=$!
  for _ in $(seq 1 60); do
    curl -fsS -o /dev/null --max-time 1 "http://127.0.0.1:${PORT}/api/lidar" && break
    sleep 0.5
  done
fi

log "карта:  http://127.0.0.1:${PORT}/map"
log "маршрут: http://127.0.0.1:${PORT}/route"
log "WASD:   http://127.0.0.1:${PORT}/robot.html"
log "демо-скан комнаты → Ctrl+C останавливает всё"
.venv/bin/python scripts/demo_scan.py --url "http://127.0.0.1:${PORT}" &
SCAN_PID=$!
wait "$SCAN_PID"
