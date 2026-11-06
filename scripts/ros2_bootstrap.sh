#!/usr/bin/env bash
# Privileged startup preflight for ROS 2 and lidar drivers.
# Invoked by robot-ros2-setup.service as root, before the web applications.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

read_env() {
  local key="$1" fallback="$2" value
  value="$(grep -E "^${key}=" .env 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
  value="${value%\"}"; value="${value#\"}"
  value="${value%\'}"; value="${value#\'}"
  printf '%s' "${value:-$fallback}"
}

ros_auto="${RC_ROS_AUTO:-$(read_env RC_ROS_AUTO 1)}"
lidar_source="${RC_LIDAR_SOURCE:-$(read_env RC_LIDAR_SOURCE auto)}"
case "${ros_auto,,}" in
  0|false|no|off|нет)
    echo "ROS 2 auto-install disabled (RC_ROS_AUTO=$ros_auto); skipping."
    exit 0 ;;
esac
case "${lidar_source,,}" in
  serial)
    echo "Lidar is configured for direct USB serial; ROS 2 setup is not needed."
    exit 0 ;;
esac

if [[ ! -f scripts/ros2_setup.sh ]]; then
  echo "ROS 2 setup script is missing from $ROOT; cannot install lidar drivers." >&2
  exit 1
fi

echo "Checking/installing ROS 2 and lidar drivers before starting the robot servers."
exec bash scripts/ros2_setup.sh --quiet
