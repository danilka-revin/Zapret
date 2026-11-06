#!/usr/bin/env bash
#
# Робот Zapret: лидар LDS-01 (TurtleBot3 Burger, HLS-LFCD2) через драйвер ROS 2.
#
#   ./scripts/lidar_lds01.sh            записать профиль в .env и показать статус
#   ./scripts/lidar_lds01.sh --start    записать профиль и запустить драйвер (в терминале)
#   ./scripts/lidar_lds01.sh --status   только статус: топик /scan, частота, драйвер
#   ./scripts/lidar_lds01.sh --print    показать, что будет записано, ничего не меняя
#
# Что записывается в .env (остальные строки файла не трогаются):
#
#   RC_LIDAR_SOURCE=ros        скан берётся из топика /scan, порт лидара открывает драйвер
#   RC_LIDAR_PROTOCOL=lds01    LDS-01/LDS-02, кадры 0xFA, 230400 бод
#   RC_LIDAR_BAUD=230400
#   RC_LIDAR_MOTOR=none        линию мотора не трогаем: вращение включает сам драйвер
#   RC_LIDAR_AUTOPROBE=0       чужие порты не перебираем — они нужны шасси и BMS
#
# После этого пульт (8080) и бортовой экран (8090) читают /scan, а не USB-порт:
# приложение и драйвер больше не отбирают друг у друга один и тот же адаптер.
#
# Сам драйвер ставит и запускает супервизор zapret_lidar (пакет
# hls_lfcd_lds_driver); установка — ./scripts/ros2_setup.sh, запуск —
# ./scripts/ros2_lidar.sh или этот скрипт с --start.
#
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
ROOT="$PWD"

PRESET="lds01"
DO_WRITE=1
DO_STATUS=1
DO_START=0

log()  { printf '\033[1;32m[lds01]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[lds01]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[lds01]\033[0m %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start)  DO_START=1; shift ;;
    --status) DO_WRITE=0; DO_STATUS=1; shift ;;
    --print)  DO_WRITE=0; DO_STATUS=0; shift ;;
    -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
    *) die "неизвестный аргумент: $1 (см. --help)" ;;
  esac
done

python_bin() {
  if [[ -x "$ROOT/.venv/bin/python" ]]; then echo "$ROOT/.venv/bin/python"; else echo python3; fi
}
PY="$(python_bin)"
command -v "$PY" >/dev/null 2>&1 || die "нет python3 — поставь зависимости: ./scripts/ensure_deps.sh"

# Настройки пишет тот же код, что и панель «Лидар»: проверка значений, атомарная
# запись, сохранение остальных строк .env. Дублировать это в bash нельзя —
# разъедется с приложением.
profile() {
  # .env берём этого проекта ($ROOT), а не «рядом с модулем»: на роботе проект
  # могут запускать из другого каталога, и профиль должен лечь туда же, куда
  # пишет панель «Лидар».
  "$PY" - "$ROOT" "$PRESET" "$1" "$ROOT/.env" <<'PY'
import json, sys
from pathlib import Path

sys.path.insert(0, sys.argv[1])
from robot_control.lidar import normalize_lidar_settings
from robot_control.lidar_settings import save_lidar_settings

preset, mode, env_path = sys.argv[2], sys.argv[3], Path(sys.argv[4])
if mode == "write":
    saved = save_lidar_settings({"preset": preset}, env_path=env_path)
else:
    saved = normalize_lidar_settings({"preset": preset})
print(json.dumps(saved, ensure_ascii=False, sort_keys=True))
PY
}

if [[ "$DO_WRITE" == "1" ]]; then
  saved="$(profile write)" || die "не удалось записать профиль в .env"
  log "профиль LDS-01 записан в .env: $saved"
elif [[ "$DO_STATUS" == "0" ]]; then
  saved="$(profile print)" || die "не удалось проверить профиль"
  log "будет записано в .env: $saved"
fi

if [[ "$DO_START" == "1" ]]; then
  [[ -x "$ROOT/scripts/ros2_lidar.sh" ]] || die "нет scripts/ros2_lidar.sh"
  log "запускаю драйвер LDS-01 через супервизор ROS 2…"
  exec "$ROOT/scripts/ros2_lidar.sh" --model lds01
fi

if [[ "$DO_STATUS" == "1" ]]; then
  if [[ -x "$ROOT/scripts/ros2_lidar.sh" ]]; then
    "$ROOT/scripts/ros2_lidar.sh" --status || warn \
      "статус не получен: проверь ./scripts/ros2_setup.sh --status и права на порт (группа dialout)"
  fi
  log "дальше: ./scripts/lidar_lds01.sh --start (или ./scripts/ros2_lidar.sh) — драйвер поднимется сам"
fi
