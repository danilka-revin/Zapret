#!/usr/bin/env bash
# Единственный запуск робота: веб-пульт (8080) + экран на борту (8090).
# Зависимости ставятся/проверяются один раз, окна и серверы не дублируются.
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"

log() { printf '\033[1;32m[robot]\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31m[robot]\033[0m %s\n' "$*" >&2; exit 1; }

mkdir -p var
exec 9>var/start-all.lock
if command -v flock >/dev/null 2>&1 && ! flock -n 9; then
  log "робот уже запущен этим ярлыком — второй комплект серверов не создаю."
  exit 0
fi

if [[ "${RC_SKIP_DEPS:-0}" != "1" ]]; then
  ./scripts/ensure_deps.sh || fail "не удалось установить зависимости."
fi
export RC_SKIP_DEPS=1

read_env() {
  local key="$1" fallback="$2" value
  value="$(grep -E "^${key}=" .env 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
  printf '%s' "${value:-$fallback}"
}
PORT="$(read_env RC_PORT 8080)"
KIOSK_PORT="$(read_env RC_KIOSK_PORT 8090)"

health() {
  local port="$1"
  .venv/bin/python -c '
import sys, urllib.request
try:
    response = urllib.request.urlopen("http://127.0.0.1:%s/healthz" % sys.argv[1], timeout=1)
    sys.exit(0 if response.status == 200 else 1)
except Exception:
    sys.exit(1)
' "$port" >/dev/null 2>&1
}
port_open() {
  .venv/bin/python -c '
import socket, sys
sock = socket.socket(); sock.settimeout(.3)
try:
    sock.connect(("127.0.0.1", int(sys.argv[1]))); sys.exit(0)
except OSError:
    sys.exit(1)
finally:
    sock.close()
' "$1" >/dev/null 2>&1
}

CONTROL_PID=""
KIOSK_PID=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ -n "$KIOSK_PID" ]] && kill -0 "$KIOSK_PID" 2>/dev/null; then
    # Фоновый bash child игнорирует SIGINT в неинтерактивной оболочке; SIGTERM
    # гарантированно прекращает run_kiosk.py и его HTTP-сервер.
    kill -TERM "$KIOSK_PID" 2>/dev/null || true
  fi
  if [[ -n "$CONTROL_PID" ]] && kill -0 "$CONTROL_PID" 2>/dev/null; then
    kill -TERM "$CONTROL_PID" 2>/dev/null || true
  fi
  [[ -z "$KIOSK_PID" ]] || wait "$KIOSK_PID" 2>/dev/null || true
  [[ -z "$CONTROL_PID" ]] || wait "$CONTROL_PID" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

log "запускаю оба интерфейса одним комплектом."

if health "$PORT"; then
  log "веб-пульт уже работает: http://127.0.0.1:${PORT}/"
  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] && command -v xdg-open >/dev/null 2>&1; then
    xdg-open "http://127.0.0.1:${PORT}/" >/dev/null 2>&1 || true
  fi
elif port_open "$PORT"; then
  fail "порт $PORT занят не веб-пультом. Освободите порт или задайте RC_PORT в .env."
else
  ./scripts/start.sh &
  CONTROL_PID=$!
  ready=""
  for _ in $(seq 1 45); do
    if health "$PORT"; then ready=1; break; fi
    if ! kill -0 "$CONTROL_PID" 2>/dev/null; then
      wait "$CONTROL_PID"; status=$?
      fail "веб-пульт не запустился (код $status)."
    fi
    sleep 1
  done
  [[ -n "$ready" ]] || fail "веб-пульт не ответил за 45 секунд."
fi

if health "$KIOSK_PORT"; then
  log "экран робота уже работает: http://127.0.0.1:${KIOSK_PORT}/kiosk"
  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    ./scripts/kiosk.sh &
    KIOSK_PID=$! # подключается к готовому серверу и открывает только экран
  fi
elif port_open "$KIOSK_PORT"; then
  fail "порт киоска $KIOSK_PORT занят другим приложением."
else
  ./scripts/kiosk.sh &
  KIOSK_PID=$!
fi

log "готово: пульт http://127.0.0.1:${PORT}/ · киоск http://127.0.0.1:${KIOSK_PORT}/kiosk"
log "для остановки нажмите Ctrl+C; можно также выполнить ./scripts/stop.sh."

children=()
[[ -z "$CONTROL_PID" ]] || children+=("$CONTROL_PID")
[[ -z "$KIOSK_PID" ]] || children+=("$KIOSK_PID")
if [[ ${#children[@]} -eq 0 ]]; then
  log "оба сервера уже обслуживаются отдельно; ярлыки браузеров открыты."
  exit 0
fi
set +e
wait -n "${children[@]}"
status=$?
set -e
exit "$status"
