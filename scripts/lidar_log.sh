#!/usr/bin/env bash
# Preserve the lidar/setup process output in var/ and in the caller's journal.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
ROOT="$PWD"
LOG_FILE="${RC_LIDAR_LOG_FILE:-$ROOT/var/lidar.log}"
[[ "$LOG_FILE" = /* ]] || LOG_FILE="$ROOT/$LOG_FILE"
if [[ $# -eq 0 ]]; then
  echo "usage: lidar_log.sh COMMAND [ARG ...]" >&2
  exit 2
fi
umask 027
mkdir -p "$(dirname "$LOG_FILE")" || exit 1
if [[ -f "$LOG_FILE" && $(wc -c < "$LOG_FILE") -gt 8388608 ]]; then
  mv -f "$LOG_FILE" "$LOG_FILE.1" || exit 1
fi
{
  printf '\n===== %s START =====\n' "$(date --iso-8601=seconds 2>/dev/null || date)"
  printf 'cwd=%s uid=%s gid=%s command=' "$ROOT" "$(id -u)" "$(id -g)"
  printf '%q ' "$@"
  printf '\n'
} | tee -a "$LOG_FILE"
export RC_LIDAR_LOG_ACTIVE=1
set +e
"$@" 2>&1 | tee -a "$LOG_FILE"
statuses=( "${PIPESTATUS[@]}" )
exit_code=${statuses[0]:-1}
printf '===== %s EXIT code=%s =====\n' "$(date --iso-8601=seconds 2>/dev/null || date)" "$exit_code" | tee -a "$LOG_FILE"
exit "$exit_code"
