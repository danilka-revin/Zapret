#!/usr/bin/env bash
# Создаёт .venv и проверяет зависимости. Базовые колёса лежат в vendor/.
# OpenCV для USB-камер ставится отдельно и не обязателен для запуска пульта.
set -euo pipefail
cd "$(dirname "$0")/.."

log() { printf '\033[1;32m[deps]\033[0m %s\n' "$*"; }

usage() {
  echo 'Использование: ./scripts/ensure_deps.sh [--camera]'
  echo '  --camera: обязательно восстановить и проверить OpenCV для USB-камер.'
  echo '  RC_DEPS_OFFLINE=1: брать колёса только из vendor/. RC_DEPS_CAMERA=0: пропустить необязательный OpenCV.'
}

CAMERA_REQUIRED=0
case "$#:${1:-}" in
  0:) ;;
  1:--camera) CAMERA_REQUIRED=1 ;;
  1:-h|1:--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

PYTHON3="${RC_PYTHON3:-$(command -v python3)}"
if [[ -z "$PYTHON3" ]] || ! "$PYTHON3" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)' 2>/dev/null; then
  HAVE_VERSION=""
  if [[ -n "$PYTHON3" ]]; then HAVE_VERSION="$("$PYTHON3" --version 2>&1 || true)"; fi
  [[ -n "$HAVE_VERSION" ]] || HAVE_VERSION="python3 не найден"
  echo "Нужен Python 3.9+ (выбранный интерпретатор: $HAVE_VERSION)." >&2
  echo "Полная установка: ./scripts/install_ubuntu.sh (или ./scripts/install_python.sh)." >&2
  exit 1
fi

if [[ -x .venv/bin/python ]] && ! .venv/bin/python -c 'import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)' 2>/dev/null; then
  log "существующий .venv создан на неподдерживаемом Python — пересоздаю его на $PYTHON3…"
  rm -rf .venv
fi

if [[ ! -x .venv/bin/python ]]; then
  log "создаю .venv на основе $PYTHON3 ($($PYTHON3 --version 2>&1))…"
  if ! "$PYTHON3" -m venv .venv 2>/dev/null && ! "$PYTHON3" -m venv --without-pip .venv 2>/dev/null; then
    if [[ "${RC_DEPS_OFFLINE:-0}" != "1" ]] && command -v apt >/dev/null 2>&1; then
      PYTHON_VERSION="$("$PYTHON3" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
      VENV_PACKAGE="python${PYTHON_VERSION}-venv"
      log "ставлю $VENV_PACKAGE через apt…"
      if command -v sudo >/dev/null 2>&1 && [[ $EUID -ne 0 ]]; then SUDO=sudo; else SUDO=; fi
      $SUDO apt update -y && $SUDO apt install -y "$VENV_PACKAGE"
      "$PYTHON3" -m venv --without-pip .venv
    else
      echo "Не удалось создать venv на основе $PYTHON3, а apt недоступен." >&2
      exit 1
    fi
  fi
fi

# pip в venv: сначала ensurepip, иначе — колесо pip из vendor/ (офлайн)
if [[ ! -x .venv/bin/pip ]]; then
  .venv/bin/python -m ensurepip --default-pip 2>/dev/null || true
fi
if [[ ! -x .venv/bin/pip ]]; then
  PIPWHL="$(ls vendor/pip-*.whl 2>/dev/null | head -1 || true)"
  if [[ -n "$PIPWHL" ]]; then
    log "ставлю pip из vendor (офлайн)…"
    PYTHONPATH="$PIPWHL" .venv/bin/python -m pip install -q --no-index --find-links vendor pip
  fi
fi
[[ -x .venv/bin/pip ]] || { echo "Не удалось поставить pip в .venv." >&2; exit 127; }

# Какие модули обязаны импортироваться в .venv: flask/gunicorn — веб-пульт,
# serial (pyserial) — USB-лидар и UART-модули, em (Empy) — ROS 2 rosidl/rclpy.
# Проверяем именно импорт, а не только хэш requirements.txt: при прерванной
# установке модуль может отсутствовать, хотя хэш уже записан.
REQUIRED_MODULES=(flask gunicorn serial em)

missing_deps() {
  local missing=() module
  for module in "${REQUIRED_MODULES[@]}"; do
    .venv/bin/python -c "import $module" >/dev/null 2>&1 || missing+=("$module")
  done
  printf '%s\n' "${missing[@]}"
}

install_deps() {
  log "ставлю зависимости…"
  if [[ "${RC_DEPS_OFFLINE:-0}" == "1" ]]; then
    log "USB-обновление: ставлю только из vendor (офлайн)…"
    .venv/bin/pip install -q --no-index --find-links vendor -r requirements.txt
  elif ! .venv/bin/pip install -q -r requirements.txt 2>/dev/null; then
    log "сети нет — ставлю из vendor (офлайн)…"
    .venv/bin/pip install -q --no-index --find-links vendor -r requirements.txt
  fi
}

REQ_HASH="$(sha256sum requirements.txt | awk '{print $1}')"
CACHED="$(cat .venv/.req-hash 2>/dev/null || true)"
MISSING="$(missing_deps)"
if [[ "$REQ_HASH" != "$CACHED" ]]; then
  install_deps
elif [[ -n "$MISSING" ]]; then
  log "в .venv не хватает: $(echo "$MISSING" | tr '\n' ' ')— ставлю заново"
  install_deps
else
  log "зависимости уже установлены"
fi

MISSING="$(missing_deps)"
if [[ -n "$MISSING" ]]; then
  echo "Не удалось установить в .venv: $(echo "$MISSING" | tr '\n' ' ')" >&2
  echo "Без pyserial не читается USB-лидар, а без Empy ROS 2 не импортирует rclpy." >&2
  echo "Поставь зависимости вручную и повтори запуск:" >&2
  echo "  .venv/bin/pip install --no-index --find-links vendor -r requirements.txt" >&2
  exit 1
fi
if [[ "$REQ_HASH" != "$CACHED" ]]; then
  echo "$REQ_HASH" > .venv/.req-hash
fi
log "проверка: $(.venv/bin/python -c 'import serial
from importlib.metadata import version
print("flask", version("flask"), "· pyserial", serial.VERSION)')"

# Probe the same interpreter as robot-kiosk, not only package metadata. A
# broken cv2 namespace, missing V4L2 backend or NumPy ABI also means no video.
probe_camera() {
  .venv/bin/python - <<'PY_CHECK_CAMERA'
import sys
import cv2
import numpy as np

if not callable(getattr(cv2, "VideoCapture", None)):
    raise RuntimeError("cv2 загружен без VideoCapture")
if sys.platform.startswith("linux") and not cv2.videoio_registry.hasBackend(cv2.CAP_V4L2):
    raise RuntimeError("сборка OpenCV не поддерживает V4L2")
ok, jpeg = cv2.imencode(".jpg", np.zeros((8, 8, 3), dtype=np.uint8))
if not ok or cv2.imdecode(jpeg, cv2.IMREAD_COLOR) is None:
    raise RuntimeError("OpenCV не кодирует/не декодирует JPEG")
print(f"OpenCV {cv2.__version__} · NumPy {np.__version__} · JPEG OK")
PY_CHECK_CAMERA
}

install_camera_wheels() {
  # Reinstall even if there is metadata for the same version: a different
  # cv2 distribution or an interrupted update may have overwritten its files.
  local -a opts=(--disable-pip-version-check --no-input --only-binary=:all:
                 --force-reinstall --no-deps -c requirements-camera.txt)
  local output
  if output=$(PIP_CONFIG_FILE=/dev/null PIP_FIND_LINKS= .venv/bin/python -m pip install \
      "${opts[@]}" --no-index --find-links vendor "$@" 2>&1); then
    return 0
  fi
  if [[ "${RC_DEPS_OFFLINE:-0}" == "1" ]]; then
    printf '%s\n' "$output" >&2
    return 1
  fi
  .venv/bin/python -m pip install "${opts[@]}" --find-links vendor --retries 1 --timeout 8 "$@"
}

ensure_camera() {
  local status
  if status=$(probe_camera 2>&1); then
    log "$status"
    return 0
  fi
  log "OpenCV в .venv недоступен: $(printf '%s\n' "$status" | tail -n 1)"
  log "восстанавливаю зависимости USB-камер (без PyTorch и YOLO)…"
  local -a packages=(opencv-python-headless)
  # Do not replace a working NumPy: it may be shared by ROS/PyTorch.
  if ! .venv/bin/python -c 'import numpy' >/dev/null 2>&1; then
    packages+=(numpy)
  fi
  if install_camera_wheels "${packages[@]}" && status=$(probe_camera 2>&1); then
    log "$status"
    return 0
  fi
  printf '%s\n' 'USB-камеры недоступны, базовый пульт может работать без них.' \
    "Причина проверки: $(printf '%s\n' "$status" | tail -n 1)" \
    'На бортовом ПК: ./scripts/ensure_deps.sh --camera, затем перезапустите киоск.' \
    'Без сети нужны совместимые колёса OpenCV и NumPy (см. docs/DEPLOY_UBUNTU.md).' >&2
  return 1
}

if [[ "$CAMERA_REQUIRED" == "1" || "${RC_DEPS_CAMERA:-1}" == "1" ]]; then
  if ! ensure_camera && [[ "$CAMERA_REQUIRED" == "1" ]]; then
    exit 1
  fi
fi
