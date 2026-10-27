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
# Прошиваем не «по файлу-отметке», а по факту: плата в каждой телеметрической
# строке сообщает `fw=` — протокол своей прошивки (scripts/firmware_probe.py).
# Если на плате другая версия, чем в коде (или версия неизвестна), плата
# прошивается заново. Отметка var/firmware-flashed/<КОЛЕСО> с хешем скетча
# экономит время: она избавляет от лишнего стирания и записи, но не отменяет
# проверку по @TLM.
#
# Повторной прошивки нет: плата с уже актуальной прошивкой не трогается.
# Неудачная попытка запоминается («fail»): та же прошивка в тот же порт сама не
# повторяется, чтобы не ждать неотвечающую плату при каждом старте — лечится
# --force.
#
# arduino-cli нужен для сборки и загрузки. Если его нет, скрипт один раз
# пытается поставить его сам (scripts/install_arduino_cli.sh, без sudo), а при
# неудаче честно говорит, что прошить можно копиями motor_wheel_*.ino с рабочего
# стола через Arduino IDE.
#
# Коды возврата:
#   0  всё прошито (или прошивать нечего)
#   1  хотя бы одна плата не прошилась
#   2  нет arduino-cli — прошивка невозможна (шаг пропускается, не ошибка)
#   3  ни одного порта не выбрано в настройках
#
# Флаги: --check (только показать план), --force (прошить даже актуальные),
#        --no-verify (не спрашивать плату про fw=), --reason ТЕКСТ (в журнал).
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
# Куда install_arduino_cli.sh кладёт бинарник: он может быть ещё не в PATH.
CLI_DIR="${RC_ARDUINO_CLI_DIR:-$HOME/.local/bin}"
[[ -d "$CLI_DIR" ]] && PATH="$CLI_DIR:$PATH"

CHECK=0
FORCE=0
VERIFY="${RC_FIRMWARE_VERIFY:-1}"
REASON=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) CHECK=1 ;;
    --force) FORCE=1 ;;
    --no-verify) VERIFY=0 ;;
    --reason) shift; REASON="${1:-}" ;;
    -h|--help)
      sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
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

# Версия прошивки, которая лежит в коде. Именно её должна сообщить плата в @TLM.
EXPECTED_FW="$(grep -oE 'FIRMWARE_PROTOCOL[[:space:]]*=[[:space:]]*[0-9]+' "$SKETCH" \
  | head -n 1 | grep -oE '[0-9]+$' || true)"

mkdir -p "$BUILD_DIR" "$STATE_DIR" || { warn "не удалось создать $VAR_DIR"; exit 1; }
: >"$LOG_FILE" 2>/dev/null || true

[[ -n "$REASON" ]] && say "причина: $REASON"
[[ -n "$EXPECTED_FW" ]] && say "в коде прошивка fw=$EXPECTED_FW ($FQBN)"

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

# Определить каталог, где лежит скетч колеса (уже собранный).
sketch_dir() {
  printf '%s' "$BUILD_DIR/motor_wheel_$1"
}

# --- опрос плат: какая прошивка на самом деле ---------------------------------
# PROBE_OUT — строка вида «FL=7 FR=7 RL=- RR=-». Пустая — платы не ответили.
PROBE_OUT=""
# Проверка «уже прошито» должна быть быстрой: плата при открытии порта
# перезагружается и печатает @TLM примерно через 2 с, а держать порты колёс
# закрытыми на старте дольше — значит задерживать подключение колёс.
PROBE_TIMEOUT="${RC_FIRMWARE_PROBE_TIMEOUT:-3}"
probe_run() {
  local modules=("$@") cmd=() args=() module python
  PROBE_OUT=""
  [[ ${#modules[@]} -eq 0 ]] && return 1
  if [[ -n "${RC_FIRMWARE_PROBE_CMD:-}" ]]; then
    cmd=("$RC_FIRMWARE_PROBE_CMD")
  else
    python="$ROOT/.venv/bin/python"
    [[ -x "$python" ]] || python="$(command -v python3 || true)"
    [[ -n "$python" && -f "$ROOT/scripts/firmware_probe.py" ]] || return 1
    cmd=("$python" "$ROOT/scripts/firmware_probe.py")
  fi
  args=(--env-file "$ENV_FILE" --timeout "$PROBE_TIMEOUT")
  for module in "${modules[@]}"; do args+=(--module "$module"); done
  PROBE_OUT="$("${cmd[@]}" "${args[@]}" 2>/dev/null || true)"
  [[ -n "$PROBE_OUT" ]]
}

# Версия прошивки конкретной платы по последнему опросу: «7» или «-».
probe_value() {
  local module="$1"
  grep -oE "(^| )${module}=([0-9]+|-)" <<<"$PROBE_OUT" | tail -n 1 | cut -d= -f2
}

# --- план ---------------------------------------------------------------------
todo=() current_records=() unassigned=() absent=() plan=()
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
    if [[ "$known" == "$hash $real" || "$known" == "$hash $real ok" ]]; then
      current_records+=("$module|$port|$real|$hash")
      continue
    fi
    if [[ "$known" == "$hash $real fail" ]]; then
      say "колесо $module: прошлая прошивка не удалась — проверьте кабель и питание," \
          "и запустите ./scripts/flash_firmware.sh --force"
      known_failed=$((known_failed + 1))
      continue
    fi
  fi
  todo+=("$module|$port|$real|$hash|")
done

for module in "${unassigned[@]}"; do say "колесо $module: порт не выбран в настройках — пропущено"; done
for item in "${absent[@]}"; do say "колесо $item: устройства нет сейчас — пропущено"; done

# Платы, которые по отметке «уже прошиты», всё равно спрашиваем про fw=:
# плату могли заменить, прошить старой копией с рабочего стола или прошивка
# могла не дойти. Это и есть «актуальная версия на самом деле», а не «мы так
# думаем по своему файлу».
current=0
if [[ ${#current_records[@]} -gt 0 ]]; then
  probe_modules=()
  for record in "${current_records[@]}"; do probe_modules+=("${record%%|*}"); done
  if [[ "$VERIFY" != "0" ]] && probe_run "${probe_modules[@]}" >/dev/null 2>&1; then
    for record in "${current_records[@]}"; do
      IFS='|' read -r module port real hash <<<"$record"
      fw="$(probe_value "$module")"
      if [[ "$fw" == "-" || -z "$fw" ]]; then
        say "колесо $module: плата не ответила — считаю прошивку актуальной (проверить: --force)"
        current=$((current + 1))
        plan+=("$module=$port уже с этой прошивкой")
        continue
      fi
      if [[ -n "$EXPECTED_FW" && "$fw" != "$EXPECTED_FW" ]]; then
        say "колесо $module: на плате fw=$fw, в коде fw=$EXPECTED_FW — прошиваю заново"
        todo+=("$module|$port|$real|$hash|на плате fw=$fw, нужна fw=$EXPECTED_FW")
        continue
      fi
      say "колесо $module: подтверждено по @TLM — fw=$fw, прошивка актуальна"
      plan+=("$module=$port уже с этой прошивкой")
      current=$((current + 1))
    done
  else
    current=${#current_records[@]}
    for record in "${current_records[@]}"; do
      IFS='|' read -r module port real hash <<<"$record"
      plan+=("$module=$port уже с этой прошивкой")
    done
    [[ "$VERIFY" == "0" ]] || say "платы не ответили на опрос — веру проверяю по отметке о прошивке"
  fi
fi

if [[ ${#unassigned[@]} -eq ${#MODULES[@]} ]]; then
  say "порты колёс не выбраны: бортовой экран → «Тяга и руление» → «Настройка портов колёс»."
  say "выберите USB-порт для каждого колеса — после этого прошивка пойдёт сама."
  exit 3
fi

if [[ ${#todo[@]} -eq 0 ]]; then
  if [[ $known_failed -gt 0 ]]; then
    say "прошивать нечего: $known_failed плат(ы) с прошлой неудачей ждут --force."
  elif [[ -n "$EXPECTED_FW" ]]; then
    say "прошивать нечего: все назначенные платы уже на прошивке fw=$EXPECTED_FW."
  else
    say "прошивать нечего: у всех назначенных колёс уже эта прошивка."
  fi
  exit 0
fi

for record in "${todo[@]}"; do
  IFS='|' read -r module port real hash why <<<"$record"
  plan+=("$module=$port прошить${why:+ ($why)}")
done

if [[ $CHECK -eq 1 ]]; then
  for line in "${plan[@]}"; do say "план: $line"; done
  say "проверка закончена, ничего не прошивал (--check)."
  exit 0
fi

# --- arduino-cli: есть, а если нет — попробовать поставить --------------------
if ! command -v arduino-cli >/dev/null 2>&1 && [[ -x "$CLI_DIR/arduino-cli" ]]; then
  PATH="$CLI_DIR:$PATH"
fi
if ! command -v arduino-cli >/dev/null 2>&1; then
  if [[ "${RC_FIRMWARE_AUTOCLI:-1}" != "0" && -f "$ROOT/scripts/install_arduino_cli.sh" ]]; then
    say "arduino-cli не найден — ставлю сам (без sudo, в $CLI_DIR)…"
    if bash "$ROOT/scripts/install_arduino_cli.sh" >>"$LOG_FILE" 2>&1; then
      PATH="$CLI_DIR:$PATH"
      say "arduino-cli установлен: $(command -v arduino-cli)"
    else
      tail -n 5 "$LOG_FILE" | while IFS= read -r line; do warn "  $line"; done
    fi
  fi
fi
if ! command -v arduino-cli >/dev/null 2>&1; then
  warn "arduino-cli не установлен — платы не прошиты автоматически."
  warn "варианты: подключите интернет (скрипт поставит его сам) или откройте"
  warn "motor_wheel_*.ino с рабочего стола в Arduino IDE и нажмите «Загрузить»."
  for line in "${plan[@]}"; do warn "  $line"; done
  exit 2
fi
# Ядро AVR должно быть установлено: без него compile упадёт «Platform not installed».
if ! arduino-cli core list 2>/dev/null | grep -q '^arduino:avr'; then
  say "ставлю ядро arduino:avr для arduino-cli…"
  if ! arduino-cli core install arduino:avr >>"$LOG_FILE" 2>&1; then
    warn "не удалось поставить ядро arduino:avr (нужен интернет) — см. $LOG_FILE"
    warn "платы не прошиты; повторите ./scripts/flash_firmware.sh, когда будет сеть."
    exit 2
  fi
fi

# --- прошивка -----------------------------------------------------------------
flashed=0 failed=0
flashed_modules=()
for record in "${todo[@]}"; do
  IFS='|' read -r module port real hash why <<<"$record"
  say "прошиваю $module → $port${why:+ ($why)}…"
  if arduino-cli compile --upload -p "$port" --fqbn "$FQBN" \
       "$(sketch_dir "$module")" >>"$LOG_FILE" 2>&1; then
    printf '%s %s ok\n' "$hash" "$real" >"$STATE_DIR/$module.flash"
    say "колесо $module: прошито ($port)"
    flashed=$((flashed + 1))
    flashed_modules+=("$module")
  else
    printf '%s %s fail\n' "$hash" "$real" >"$STATE_DIR/$module.flash"
    tail -n 5 "$LOG_FILE" | while IFS= read -r line; do warn "  $line"; done
    warn "колесо $module: прошивка не удалась ($port), журнал: $LOG_FILE"
    failed=$((failed + 1))
  fi
done

# --- подтверждение ------------------------------------------------------------
# «Загружено без ошибок» ещё не значит «плата работает на новой прошивке»:
# спрашиваем у неё самой, какой fw она сообщает.
if [[ ${#flashed_modules[@]} -gt 0 && "$VERIFY" != "0" && -n "$EXPECTED_FW" ]]; then
  say "проверяю, что платы действительно на fw=$EXPECTED_FW…"
  # После загрузки плате нужно время на перезапуск — ждём дольше, чем при
  # обычной проверке.
  PROBE_TIMEOUT="${RC_FIRMWARE_PROBE_TIMEOUT:-3}"
  [[ "$PROBE_TIMEOUT" -lt 6 ]] && PROBE_TIMEOUT=6
  if probe_run "${flashed_modules[@]}"; then
    for module in "${flashed_modules[@]}"; do
      fw="$(probe_value "$module")"
      case "$fw" in
        "$EXPECTED_FW")
          say "колесо $module: подтверждено по @TLM — fw=$fw" ;;
        "-"|"")
          say "колесо $module: не ответило на опрос (это не ошибка: проверьте журнал плат)" ;;
        *)
          warn "колесо $module: после прошивки плата сообщает fw=$fw вместо $EXPECTED_FW"
          printf '%s %s fail\n' "$(sketch_hash "$(sketch_dir "$module")/motor_wheel_$module.ino")" \
            "$(readlink -f "$(read_env "RC_WHEEL_${module}_PORT" "")" 2>/dev/null || true)" \
            >"$STATE_DIR/$module.flash"
          failed=$((failed + 1)) ;;
      esac
    done
  else
    say "платы не ответили на опрос после прошивки — проверьте их телеметрию на экране"
  fi
fi

say "итог: прошито $flashed, уже было актуально $current, ошибок $failed, без порта ${#unassigned[@]}, нет устройства ${#absent[@]}."
[[ $failed -eq 0 ]] || exit 1
exit 0
