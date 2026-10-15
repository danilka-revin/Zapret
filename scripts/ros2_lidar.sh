#!/usr/bin/env bash
#
# Запуск лидара робота Zapret через ROS 2.
#
#   ./scripts/ros2_lidar.sh                    запустить (модель и порт — из .env)
#   ./scripts/ros2_lidar.sh --model lds01 --port /dev/ttyUSB0
#   ./scripts/ros2_lidar.sh --status           что видно: топик /scan и его частота
#   ./scripts/ros2_lidar.sh --print            показать команду, ничего не запуская
#
# Что запускается: супервизор лидара (пакет zapret_lidar) — он сам подбирает
# ROS 2-драйвер под подключённый лидар (rplidar_ros, hls_lfcd_lds_driver,
# ldlidar_stl_ros2, turtlebot3_node), следит за свежестью /scan и публикует
# состояние в /zapret/lidar/status. Веб-пульт и бортовой экран читают /scan и
# показывают состояние в панели «Лидар».
#
# Перед первым запуском: ./scripts/ros2_setup.sh (проверит ROS 2 и соберёт ws).
# Автозапуск при загрузке: ./scripts/ros2_setup.sh --service
#
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"

MODEL=""
PORT=""
BAUD=""
PREFIX_ARG=""
DO_STATUS=0
DO_PRINT=0
DO_BUILD=0
ENGINE=""

log()  { printf '\033[1;32m[lidar]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[lidar]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[lidar]\033[0m %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --model)  MODEL="${2:-}"; shift 2 ;;
    --port)   PORT="${2:-}"; shift 2 ;;
    --baud)   BAUD="${2:-}"; shift 2 ;;
    --prefix) PREFIX_ARG="${2:-}"; shift 2 ;;
    --status) DO_STATUS=1; shift ;;
    --print)  DO_PRINT=1; shift ;;
    --build)  DO_BUILD=1; shift ;;
    --engine) ENGINE="${2:-}"; shift 2 ;;   # venv | ros2
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) die "неизвестный аргумент: $1 (см. --help)" ;;
  esac
done

# Значение настройки: сначала переменная окружения (её же видит приложение и
# служба systemd через EnvironmentFile), потом файл .env — как в Config.from_env.
read_env() {
  local key="$1" fallback="${2:-}" value
  value="$(printenv "$key" 2>/dev/null || true)"
  if [[ -z "$value" ]]; then
    value="$(grep -E "^${key}=" .env 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
  fi
  printf '%s' "${value:-$fallback}"
}

# --- 1. что запускать: модель, порт, скорость -------------------------------- #
[[ -n "$MODEL" ]] || MODEL="$(read_env RC_LIDAR_PROTOCOL auto)"
[[ -n "$PORT"  ]] || PORT="$(read_env RC_LIDAR_PORT '')"
[[ -n "$BAUD"  ]] || BAUD="$(read_env RC_LIDAR_BAUD '')"
SCAN_TOPIC="$(read_env RC_LIDAR_ROS_TOPIC /scan)"
STATUS_TOPIC="$(read_env RC_LIDAR_ROS_STATUS_TOPIC /zapret/lidar/status)"

# --- 2. префикс ROS 2 и собранное рабочее пространство ----------------------- #
find_prefix() {
  local candidate
  if [[ -n "$PREFIX_ARG" && -d "$PREFIX_ARG" ]]; then echo "$PREFIX_ARG"; return 0; fi
  if [[ -n "${RC_ROS_PREFIX:-}" && -d "${RC_ROS_PREFIX}" ]]; then echo "${RC_ROS_PREFIX}"; return 0; fi
  if [[ -n "${ROS_DISTRO:-}" && -d "/opt/ros/${ROS_DISTRO}" ]]; then echo "/opt/ros/${ROS_DISTRO}"; return 0; fi
  for candidate in /opt/ros/*; do
    [[ -d "$candidate" ]] || continue
    echo "$candidate"; return 0
  done
  return 1
}

PREFIX="$(find_prefix || true)"
# Собранное рабочее пространство: обычно внутри проекта, но путь можно задать
# (RC_ROS_WS_INSTALL) — например, если пакет собран в другом месте.
INSTALL="${RC_ROS_WS_INSTALL:-$ROOT/ros2_ws/install}"
WS_SETUP=""
[[ -f "$INSTALL/setup.bash" ]] && WS_SETUP="$INSTALL/setup.bash"

have() { command -v "$1" >/dev/null 2>&1; }

python_bin() {
  if [[ -x .venv/bin/python ]]; then echo "$ROOT/.venv/bin/python"; else echo "python3"; fi
}

python_site() {
  local base="$1" version
  version="$("$(python_bin)" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || echo 3.10)"
  local dir
  for dir in "$base/lib/python$version/site-packages" "$base/share/zapret_lidar"; do
    [[ -d "$dir" ]] && printf '%s:' "$dir"
  done
}

# Супервизор запускаем из .venv проекта: там pyserial, которым супервизор быстро
# проверяет порты (автоподбор модели лидара), а rclpy приходит из ROS 2 через
# PYTHONPATH. Если .venv нет — идём обычным путём ``ros2 launch``.
choose_engine() {
  if [[ -n "$ENGINE" ]]; then echo "$ENGINE"; return 0; fi
  if [[ -x .venv/bin/python && -n "$WS_SETUP" ]]; then echo venv; return 0; fi
  if have ros2; then echo ros2; return 0; fi
  echo ""
}

ENGINE="$(choose_engine)"

ros_args=()
[[ -n "$MODEL" && "$MODEL" != "auto" ]] && ros_args+=(-p "model:=$MODEL") || ros_args+=(-p "model:=auto")
[[ -n "$PORT"  ]] && ros_args+=(-p "port:=$PORT")
[[ -n "$BAUD"  ]] && ros_args+=(-p "baud:=$BAUD")
ros_args+=(-p "scan_topic:=$SCAN_TOPIC" -p "status_topic:=$STATUS_TOPIC")

# Команда сборки аргументов в одну строку нужна и для запуска, и для --print.
print_command() {
  case "$ENGINE" in
    venv)
      echo "$ROOT/.venv/bin/python -m zapret_lidar.supervisor --ros-args ${ros_args[*]}"
      ;;
    ros2)
      echo "ros2 launch zapret_lidar lidar.launch.py model:=$MODEL${PORT:+ port:=$PORT}${BAUD:+ baud:=$BAUD} scan_topic:=$SCAN_TOPIC status_topic:=$STATUS_TOPIC"
      ;;
    *)
      echo "(нечего запускать: нет .venv с ROS 2 или команды ros2)"
      ;;
  esac
}

if [[ $DO_PRINT -eq 1 ]]; then
  echo "ROS 2:      ${PREFIX:-не найден}"
  echo "рабочее пр-во: ${WS_SETUP:-не собрано}"
  echo "движок:     ${ENGINE:-нет}"
  echo "команда:    $(print_command)"
  exit 0
fi

[[ -n "$PREFIX" ]] || { warn "ROS 2 не найден"; die "запусти: ./scripts/ros2_setup.sh --install"; }

if [[ $DO_BUILD -eq 1 || -z "$WS_SETUP" ]]; then
  log "собираю рабочее пространство…"
  ./scripts/ros2_setup.sh --drivers && WS_SETUP="$INSTALL/setup.bash" || true
  [[ -f "$INSTALL/setup.bash" ]] || die "рабочее пространство не собрано: ./scripts/ros2_setup.sh"
  WS_SETUP="$INSTALL/setup.bash"
fi

# shellcheck disable=SC1090
source "$PREFIX/setup.bash" 2>/dev/null || warn "не удалось выполнить source $PREFIX/setup.bash"
# shellcheck disable=SC1090
source "$WS_SETUP" 2>/dev/null || warn "не удалось выполнить source $WS_SETUP"

# --- 3. режим проверки -------------------------------------------------------- #
if [[ $DO_STATUS -eq 1 ]]; then
  echo "ROS 2:       $PREFIX"
  echo "топик скана: $SCAN_TOPIC"
  if ! have ros2; then
    warn "команда ros2 не найдена — проверка частоты недоступна"
    exit 1
  fi
  echo
  ros2 topic list 2>/dev/null | grep -E "scan|zapret" || warn "топиков /scan и /zapret/lidar/status пока нет"
  echo
  log "состояние супервизора:"
  timeout 5 ros2 topic echo --once "$STATUS_TOPIC" 2>/dev/null \
    || warn "супервизор не публикует состояние — он вообще запущен? (./scripts/ros2_lidar.sh)"
  echo
  log "частота скана (5 с):"
  timeout 6 ros2 topic hz --window 10 "$SCAN_TOPIC" 2>/dev/null \
    || warn "в $SCAN_TOPIC точек нет: лидар не отдаёт скан (см. состояние супервизора выше)"
  exit 0
fi

# --- 4. запуск --------------------------------------------------------------- #
case "$ENGINE" in
  venv)
    # PYTHONPATH: наш собранный пакет + python-пакеты ROS 2 (rclpy, sensor_msgs).
    EXTRA="$(python_site "$INSTALL/zapret_lidar")$(python_site "$PREFIX")"
    export PYTHONPATH="${EXTRA}${PYTHONPATH:-}"
    log "запускаю супервизор лидара (модель: $MODEL, порт: ${PORT:-автопоиск})"
    exec "$ROOT/.venv/bin/python" -m zapret_lidar.supervisor --ros-args "${ros_args[@]}"
    ;;
  ros2)
    log "запускаю: ros2 launch zapret_lidar lidar.launch.py"
    exec ros2 launch zapret_lidar lidar.launch.py \
      "model:=$MODEL" ${PORT:+"port:=$PORT"} ${BAUD:+"baud:=$BAUD"} \
      "scan_topic:=$SCAN_TOPIC" "status_topic:=$STATUS_TOPIC"
    ;;
  *)
    die "нечем запускать: нет .venv рядом с ROS 2 и нет команды ros2 → ./scripts/ros2_setup.sh"
    ;;
esac
