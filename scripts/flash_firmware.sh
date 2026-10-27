#!/usr/bin/env bash
# Автопрошивка плат колёс в те USB-порты, которые выбраны в настройках.
#
# Порты берутся оттуда же, откуда их берёт программа: `.env` →
# RC_WHEEL_FL_PORT … RC_WHEEL_RR_PORT (бортовой экран → «Тяга и руление» →
# «Настройка портов колёс»). Ничего не угадывается: нет порта в настройках —
# плата не прошивается, и об этом честно сказано в журнале.
#
# Прошивка собирается из firmware/motor_wheel/motor_wheel.ino с уже вписанным
# именем колеса (MODULE_ID), ровно как motor_wheel_FL.ino на рабочем столе:
# четыре платы, прошитые одним и тем же файлом, все называют себя FL.
#
# Повторный запуск ничего не прошивает: результат запоминается по хешу
# сгенерированного скетча и порту (var/firmware-flashed/<КОЛЕСО>). После
# обновления с GitHub хеш меняется — и платы прошиваются один раз. Неудачная
# попытка тоже запоминается: та же прошивка в тот же порт сама не повторяется
# (иначе каждый старт ждал бы неотвечающую плату) — только с --force.
#
# Коды возврата:
#   0  всё прошито (или прошивать нечего)
#   1  хотя бы одна плата не прошилась
#   2  нет arduino-cli — прошивка невозможна (шаг пропускается, не ошибка)
#   3  ни одного порта не выбрано в настройках
#
# Флаги: --check (только показать план), --force (прошить даже актуальные),
#        --reason ТЕКСТ (попадёт в журнал).
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"
VAR_DIR="${RC_VAR_DIR:-$ROOT/var}"
BUILD_DIR="$VAR_DIR/firmware-build"
STATE_DIR="$VAR_DIR/firmware-flashed"
LOG_FILE="$VAR_DIR/firmware-flash.log"
ENV_FILE="${RC_ENV_FILE:-$ROOT/.env}"
SKETCH="$ROOT/firmware/motor_wheel/motor_wheel.ino"
MODULES=(FL FR RL RR)

CHECK=0
FORCE=0
REASON=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) CHECK=1 ;;
    --force) FORCE=1 ;;
    --reason) shift; REASON="${1:-}"; ;;
    -h|--help)
      sed -n '2,32p' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) echo "[firmware] неизвестный аргумент: $1" >&2; exit 64 ;;
  esac
  shift
done

say() { printf '[firmware] %s\n' "$*"; }
warn() { printf '[firmware] %s\n' "$*" >&2; }

read_env() {
  local key="$1" fallback="${2:-}" value
  value="$(grep -E "^${key}=" "$ENV_FILE" 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
  printf '%s' "${value:-$fallback}"
}

if [[ "${RC_FIRMWARE_AUTOFLASH:-$(read_env RC_FIRMWARE_AUTOFLASH 1)}" == "0" ]]; then
  say "автопрошивка выключена (RC_FIRMWARE_AUTOFLASH=0) — платы не трогаю."
  exit 0
fi

FQBN="${RC_FIRMWARE_FQBN:-$(read_env RC_FIRMWARE_FQBN arduino:avr:mega)}"

if [[ ! -f "$SKETCH" ]]; then
  warn "нет исходника прошивки: $SKETCH"
  exit 1
fi

mkdir -p "$BUILD_DIR" "$STATE_DIR" || { warn "не удалось создать $VAR_DIR"; exit 1; }
: >"$LOG_FILE" 2>/dev/null || true

[[ -n "$REASON" ]] && say "причина: $REASON"

# Скетч колеса: имя каталога обязано совпадать с именем .ino (так работает
# arduino-cli), поэтому у каждого колеса свой каталог в var/, а не рядом с
# исходником в репозитории.
make_sketch() {
  local module="$1" dir="$BUILD_DIR/motor_wheel_$1" tmp
  mkdir -p "$dir" || return 1
  tmp="$dir/motor_wheel_$module.ino.tmp.$$"
  if ! sed -e "s/^#define MODULE_ID .*/#define MODULE_ID \"${module}\"/" \
           -- "$SKETCH" >"$tmp"; then
    rm -f "$tmp"
    return 1
  fi
  mv -f "$tmp" "$dir/motor_wheel_$module.ino" || return 1
  printf '%s' "$dir"
}

sketch_hash() {
  sha1sum "$1" 2>/dev/null | cut -d' ' -f1
}

# Кто вообще требует прошивки: без порта в настройках плату не трогаем.
todo=() unassigned=() absent=() plan=()
current=0
known_failed=0
for module in "${MODULES[@]}"; do
  port="$(read_env "RC_WHEEL_${module}_PORT" "")"
  if [[ -z "$port" ]]; then
    unassigned+=("$module")
    continue
  fi
  real="$(readlink -f "$port" 2>/dev/null || printf '%s' "$port")"
  if [[ ! -e "$real" ]]; then
    absent+=("$module ($port)")
    continue
  fi
  dir="$(make_sketch "$module")" || { warn "$module: не удалось собрать скетч"; continue; }
  hash="$(sketch_hash "$dir/motor_wheel_$module.ino")"
  if [[ $FORCE -ne 1 && -f "$STATE_DIR/$module.flash" ]]; then
    known="$(cat "$STATE_DIR/$module.flash" 2>/dev/null)"
    if [[ "$known" == "$hash $real" ]]; then
      plan+=("$module=$port уже с этой прошивкой")
      current=$((current + 1))
      continue
    fi
    if [[ "$known" == "$hash $real fail" ]]; then
      say "колесо $module: прошлая прошивка не удалась — проверьте кабель и питание," \
          "и запустите ./scripts/flash_firmware.sh --force"
      known_failed=$((known_failed + 1))
      continue
    fi
  fi
  todo+=("$module|$port|$real|$hash")
  plan+=("$module=$port прошить")
done

for module in "${unassigned[@]}"; do say "колесо $module: порт не выбран в настройках — пропущено"; done
for item in "${absent[@]}"; do say "колесо $item: устройства нет сейчас — пропущено"; done

if [[ ${#unassigned[@]} -eq ${#MODULES[@]} ]]; then
  say "порты колёс не выбраны: бортовой экран → «Тяга и руление» → «Настройка портов колёс»."
  say "выберите USB-порт для каждого колеса — после этого прошивка пойдёт сама."
  exit 3
fi

if [[ ${#todo[@]} -eq 0 ]]; then
  if [[ $known_failed -gt 0 ]]; then
    say "прошивать нечего: $known_failed плат(ы) с прошлой неудачей ждут --force."
  else
    say "прошивать нечего: у всех назначенных колёс уже эта прошивка."
  fi
  exit 0
fi

if [[ $CHECK -eq 1 ]]; then
  for line in "${plan[@]}"; do say "план: $line"; done
  say "проверка закончена, ничего не прошивал (--check)."
  exit 0
fi

if ! command -v arduino-cli >/dev/null 2>&1; then
  warn "arduino-cli не установлен — платы не прошиты автоматически."
  warn "варианты: sudo apt install arduino-cli, либо откройте motor_wheel_*.ino"
  warn "с рабочего стола в Arduino IDE и нажмите «Загрузить» для каждого колеса."
  for line in "${plan[@]}"; do warn "  $line"; done
  exit 2
fi

flashed=0 failed=0
for item in "${todo[@]}"; do
  IFS='|' read -r module port real hash <<<"$item"
  say "прошиваю $module → $port (arduino-cli, $FQBN)…"
  if arduino-cli compile --upload -p "$port" --fqbn "$FQBN" \
       "$BUILD_DIR/motor_wheel_$module" >>"$LOG_FILE" 2>&1; then
    printf '%s %s\n' "$hash" "$real" >"$STATE_DIR/$module.flash"
    say "колесо $module: прошито ($port)"
    flashed=$((flashed + 1))
  else
    printf '%s %s fail\n' "$hash" "$real" >"$STATE_DIR/$module.flash"
    tail -n 5 "$LOG_FILE" | while IFS= read -r line; do warn "  $line"; done
    warn "колесо $module: прошивка не удалась ($port), журнал: $LOG_FILE"
    failed=$((failed + 1))
  fi
done

say "итог: прошито $flashed, уже было актуально $current, ошибок $failed, без порта ${#unassigned[@]}, нет устройства ${#absent[@]}."
[[ $failed -eq 0 ]] || exit 1
exit 0
