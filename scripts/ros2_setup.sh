#!/usr/bin/env bash
#
# ROS 2 для лидара робота Zapret: проверка, установка пакетов драйверов и сборка
# рабочего пространства ros2_ws.
#
#   ./scripts/ros2_setup.sh                 проверить, собрать ws, включить ROS-режим
#   ./scripts/ros2_setup.sh --check         только показать, что найдено (код 0/1)
#   ./scripts/ros2_setup.sh --install       поставить ROS 2 (apt), если его нет
#   ./scripts/ros2_setup.sh --drivers       поставить пакеты драйверов лидаров (apt)
#   ./scripts/ros2_setup.sh --ldlidar       склонировать и собрать ldlidar_stl_ros2
#   ./scripts/ros2_setup.sh --service       поставить службу robot-lidar (systemd)
#   ./scripts/ros2_setup.sh --no-env        не трогать .env (RC_LIDAR_SOURCE=ros)
#   ./scripts/ros2_setup.sh --json          машинный вывод проверки
#
# Что происходит по умолчанию: находим ROS 2 (/opt/ros/<дистрибутив> или
# RC_ROS_PREFIX), ставим в .env RC_LIDAR_SOURCE=ros (лидар читается из /scan),
# собираем пакет zapret_lidar через colcon и печатаем, чем запускать.
# Ничего не ломаем: если ROS 2 нет — печатаем точные команды установки и
# выходим с кодом 1, ничего не трогая.
#
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"

DISTRO_ARG=""
DO_INSTALL=0
DO_DRIVERS=0
DO_LDLIDAR=0
DO_SERVICE=0
DO_BUILD=1
CHECK_ONLY=0
WRITE_ENV=1
AS_JSON=0

log()  { printf '\033[1;32m[ros2]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[ros2]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[ros2]\033[0m %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --distro)     DISTRO_ARG="${2:-}"; shift 2 ;;
    --install)    DO_INSTALL=1; shift ;;
    --drivers)    DO_DRIVERS=1; shift ;;
    --ldlidar)    DO_LDLIDAR=1; shift ;;
    --service)    DO_SERVICE=1; shift ;;
    --no-build)   DO_BUILD=0; shift ;;
    --check)      CHECK_ONLY=1; DO_BUILD=0; WRITE_ENV=0; shift ;;
    --no-env)     WRITE_ENV=0; shift ;;
    --json)       AS_JSON=1; shift ;;
    -h|--help)    sed -n '2,19p' "$0"; exit 0 ;;
    *) die "неизвестный аргумент: $1 (см. --help)" ;;
  esac
done

SUDO=""
if [[ $EUID -ne 0 ]] && command -v sudo >/dev/null 2>&1; then SUDO=sudo; fi

have() { command -v "$1" >/dev/null 2>&1; }

# --- 1. дистрибутив и префикс ROS 2 ----------------------------------------- #
ubuntu_codename() {
  # jammy → humble, noble → jazzy: соответствие дистрибутивов Ubuntu и ROS 2.
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    case "${VERSION_CODENAME:-}" in
      jammy) echo humble ;;
      noble) echo jazzy ;;
      focal) echo foxy ;;
    esac
  fi
}

DEFAULT_DISTRO="$(ubuntu_codename || true)"
DEFAULT_DISTRO="${DEFAULT_DISTRO:-humble}"

find_prefix() {
  local distro="$1" candidate
  if [[ -n "${RC_ROS_PREFIX:-}" && -d "${RC_ROS_PREFIX}" ]]; then
    echo "${RC_ROS_PREFIX}"; return 0
  fi
  if [[ -n "$distro" && -d "/opt/ros/$distro" ]]; then echo "/opt/ros/$distro"; return 0; fi
  for candidate in /opt/ros/*; do
    [[ -d "$candidate" ]] || continue
    echo "$candidate"; return 0
  done
  return 1
}

PREFIX="$(find_prefix "$DISTRO_ARG" || true)"
DISTRO="${DISTRO_ARG:-${ROS_DISTRO:-}}"
if [[ -z "$DISTRO" && -n "$PREFIX" ]]; then DISTRO="$(basename "$PREFIX")"; fi
DISTRO="${DISTRO:-$DEFAULT_DISTRO}"

python_bin() {
  if [[ -x .venv/bin/python ]]; then echo ".venv/bin/python"; else echo "python3"; fi
}

prefix_pythonpath() {
  local prefix="$1" version dir
  version="$("$(python_bin)" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || echo 3.10)"
  for dir in "$prefix/lib/python$version/site-packages" \
             "$prefix/lib/python3/dist-packages" \
             "$prefix/lib/python3/site-packages"; do
    [[ -d "$dir" ]] && printf '%s:' "$dir"
  done
}

rclpy_ok() {
  local prefix="$1" path
  path="$(prefix_pythonpath "$prefix")"
  PYTHONPATH="${path}${PYTHONPATH:-}" "$(python_bin)" -c 'import rclpy' >/dev/null 2>&1
}

ament_packages() {
  local prefix="$1" dir
  for dir in "$prefix"/share/ament_index/resource_index/packages "$ROOT"/ros2_ws/install/*/share/ament_index/resource_index/packages; do
    [[ -d "$dir" ]] || continue
    find "$dir" -maxdepth 1 -type f -printf '%f\n' 2>/dev/null
  done | sort -u
}

#: Пакеты драйверов лидаров, которые ставятся из apt.
APT_DRIVERS=(ros-"$DISTRO"-hls-lfcd-lds-driver ros-"$DISTRO"-rplidar-ros ros-"$DISTRO"-turtlebot3-node)

install_ros2() {
  log "ставлю ROS 2 $DISTRO (ros-base) и инструменты сборки…"
  $SUDO apt update -y || die "apt update не сработал — проверь сеть и права"
  # Официальный способ подключить репозиторий ROS 2 (пакет ros2-apt-source).
  if ! $SUDO apt install -y "ros-$DISTRO-ros-base" ros-dev-tools python3-colcon-common-extensions python3-serial; then
    warn "пакеты ROS 2 не поставились напрямую — подключаю репозиторий ROS 2…"
    $SUDO apt install -y software-properties-common curl || die "нет software-properties-common"
    local codename
    codename="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
    $SUDO curl -sSL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" \
      -o /usr/share/keyrings/ros-archive-keyring.gpg || die "не скачался ключ ROS 2"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $codename main" \
      | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
    $SUDO apt update -y
    $SUDO apt install -y "ros-$DISTRO-ros-base" ros-dev-tools \
      python3-colcon-common-extensions python3-serial || die "ROS 2 $DISTRO не поставился"
  fi
}

install_drivers() {
  log "ставлю пакеты драйверов лидаров (apt)…"
  local package
  for package in "${APT_DRIVERS[@]}"; do
    if $SUDO apt install -y "$package" >/dev/null 2>&1; then
      log "поставлен $package"
    else
      warn "нет в apt: $package (не страшно, если у робота другой лидар)"
    fi
  done
  if ! dpkg -s python3-serial >/dev/null 2>&1; then
    warn "нет python3-serial: проверка портов супервизором работать не будет (сам лидар — будет)"
  fi
}

clone_ldlidar() {
  local target="ros2_ws/src/ldlidar_stl_ros2"
  if [[ -d "$target" ]]; then
    log "ldlidar_stl_ros2 уже склонирован ($target)"
    return 0
  fi
  have git || die "нужен git, чтобы склонировать ldlidar_stl_ros2"
  log "клонирую ldlidar_stl_ros2 (LD06/LD19/STL-27L)…"
  git clone --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git "$target" \
    || die "не удалось склонировать ldlidar_stl_ros2 (нужна сеть)"
}

build_workspace() {
  have colcon || { warn "нет colcon: поставь python3-colcon-common-extensions или запусти с --install"; return 1; }
  log "собираю ros2_ws (пакет zapret_lidar)…"
  (
    set -e
    # shellcheck disable=SC1090
    [[ -n "$PREFIX" && -f "$PREFIX/setup.bash" ]] && . "$PREFIX/setup.bash"
    cd ros2_ws
    colcon build --symlink-install \
      --packages-select zapret_lidar ${RC_ROS_EXTRA_PACKAGES:+$RC_ROS_EXTRA_PACKAGES}
  ) || return 1
  log "собрано: $ROOT/ros2_ws/install"
  return 0
}

install_service() {
  have systemctl || { warn "systemd нет — службу не ставлю"; return 0; }
  [[ -f scripts/robot-lidar.service ]] || { warn "нет scripts/robot-lidar.service"; return 0; }
  local user="${SUDO_USER:-${USER:-$(id -un)}}"
  log "ставлю службу robot-lidar (лидар поднимается до приложений)…"
  $SUDO cp scripts/robot-lidar.service /etc/systemd/system/ || die "не скопировалась служба"
  $SUDO sed -i "s|/opt/robot-control|$ROOT|g; s|User=robotctl|User=$user|g; s|Group=robotctl|Group=$user|g" \
    /etc/systemd/system/robot-lidar.service
  $SUDO systemctl daemon-reload
  log "включить сейчас: sudo systemctl enable --now robot-lidar"
}

write_env() {
  [[ -f .env ]] || { warn "нет .env — включение ROS-режима пропускаю"; return 0; }
  local changed=0
  touch_env() {
    local key="$1" value="$2" current
    current="$(grep -E "^${key}=" .env 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
    if [[ -z "$current" ]]; then
      printf '%s=%s\n' "$key" "$value" >> .env
      changed=1
    elif [[ "$current" != "$value" ]]; then
      sed -i "s|^${key}=.*|${key}=${value}|" .env
      changed=1
      log "в .env: ${key} было «${current}» → стало «${value}»"
    fi
  }
  touch_env RC_LIDAR_SOURCE ros
  touch_env RC_LIDAR_ROS_TOPIC /scan
  [[ $changed -eq 1 ]] && log "лидар читается из ROS 2 (/scan): .env → RC_LIDAR_SOURCE=ros" || true
}

# --- 2. проверка ------------------------------------------------------------- #
PACKAGES="$(ament_packages "${PREFIX:-/nonexistent}" || true)"
RCLPY="нет"
[[ -n "$PREFIX" ]] && rclpy_ok "$PREFIX" && RCLPY="да"

if [[ $CHECK_ONLY -eq 1 ]]; then
  if [[ $AS_JSON -eq 1 ]]; then
    printf '{"distro":"%s","prefix":"%s","rclpy":%s,"packages":[%s],"workspace":"%s"}\n' \
      "$DISTRO" "${PREFIX:-}" \
      "$([[ "$RCLPY" == "да" ]] && echo true || echo false)" \
      "$(printf '%s\n' "$PACKAGES" | sed '/^$/d;s/.*/"&"/' | paste -sd, -)" \
      "$([[ -d ros2_ws/install ]] && echo "$ROOT/ros2_ws/install" || echo "")"
  else
    echo "ROS 2:      ${PREFIX:-не найден}"
    echo "дистрибутив: $DISTRO"
    echo "rclpy:      $RCLPY"
    echo "пакеты драйверов:"; printf '%s\n' "$PACKAGES" | sed '/^$/d;s/^/  • /'
    [[ -d ros2_ws/install ]] && echo "рабочее пространство: $ROOT/ros2_ws/install" \
                             || echo "рабочее пространство: не собрано (./scripts/ros2_setup.sh)"
  fi
  [[ -n "$PREFIX" && "$RCLPY" == "да" ]] && exit 0 || exit 1
fi

if [[ -z "$PREFIX" ]]; then
  warn "ROS 2 не найден (нет /opt/ros/$DISTRO)"
  if [[ $DO_INSTALL -eq 1 ]]; then
    install_ros2
    PREFIX="$(find_prefix "$DISTRO_ARG" || true)"
    [[ -n "$PREFIX" ]] || die "ROS 2 поставился, но префикс не найден — проверь /opt/ros"
    rclpy_ok "$PREFIX" && RCLPY="да"
  else
    cat <<EOF

Лидар через ROS 2 требует сам ROS 2. Поставить одной командой:

  ./scripts/ros2_setup.sh --install           # ROS 2 $DISTRO (ros-base) + инструменты сборки
  ./scripts/ros2_setup.sh --install --drivers # и пакеты драйверов лидаров

Если ROS 2 уже стоит в нестандартном месте — укажи его:
  RC_ROS_PREFIX=/путь/к/ros $0

Пока ROS 2 нет, лидар можно читать старым способом (без ROS):
  в .env оставь RC_LIDAR_SOURCE=serial
EOF
    exit 1
  fi
fi

log "ROS 2: $PREFIX (дистрибутив $DISTRO), rclpy: $RCLPY"
[[ "$RCLPY" == "да" ]] || warn "rclpy в $(python_bin) не импортируется — проверь PYTHONPATH и пакет ros-$DISTRO-rclpy"

[[ $DO_DRIVERS -eq 1 ]] && install_drivers
[[ $DO_LDLIDAR -eq 1 ]] && clone_ldlidar
[[ $DO_BUILD -eq 1 ]] && build_workspace
[[ $WRITE_ENV -eq 1 ]] && write_env
[[ $DO_SERVICE -eq 1 ]] && install_service

echo
log "готово"
echo "  запустить лидар:   ./scripts/ros2_lidar.sh"
echo "  проверить статус:  ./scripts/ros2_lidar.sh --status"
echo "  панель «Лидар» в веб-пульте показывает драйвер, порт и частоту скана"
