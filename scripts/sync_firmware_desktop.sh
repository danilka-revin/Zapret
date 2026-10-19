#!/usr/bin/env bash
# Кладёт на рабочий стол актуальные прошивки Arduino: мастер и ноду.
#
# Запускается сам при каждом старте через ярлык «Запустить робот»
# (scripts/start_desktop.sh) и при создании ярлыка (scripts/make_shortcut.sh),
# поэтому после обновления версии («Обновить с GitHub») на рабочем столе при
# следующем запуске лежат те же файлы, что и в firmware/ репозитория.
# Копии зеркальные: правки на рабочем столе при следующем запуске заменятся
# версией из репозитория — исходники правятся в firmware/motor_master и
# firmware/motor_node.
#
# Файл переписывается только если содержимое изменилось: актуальная копия
# не трогается (даже время изменения сохраняется). Ошибка копирования не
# критична для запуска робота — вызывающий скрипт предупреждает и идёт дальше.
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"

# Рабочий стол ищем так же, как make_shortcut.sh: иначе прошивки лягут не
# рядом с ярлыком (на русской Ubuntu это может быть ~/Рабочий стол).
DESK=""
if command -v xdg-user-dir >/dev/null 2>&1; then
  DESK="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
[[ -z "$DESK" || ! -d "$DESK" ]] && DESK="$HOME/Desktop"
mkdir -p "$DESK" || { echo "[firmware] не удалось создать папку рабочего стола: $DESK" >&2; exit 1; }

FAILED=0
sync_one() {
  local src="$1" title="$2" dst tmp
  dst="$DESK/$(basename "$src")"
  if [[ ! -f "$src" ]]; then
    echo "[firmware] нет исходника прошивки $title: $src" >&2
    FAILED=1
    return
  fi
  if [[ -f "$dst" ]] && cmp -s "$src" "$dst"; then
    echo "[firmware] $title: на рабочем столе уже актуальная версия ($(basename "$src"))"
    return
  fi
  # Через временный файл + mv: прерванное копирование не оставит на рабочем
  # столе половину прошивки, которую Arduino IDE потом не соберёт.
  tmp="$dst.tmp.$$"
  if cp -- "$src" "$tmp" && mv -f -- "$tmp" "$dst"; then
    echo "[firmware] $title: прошивка обновлена на рабочем столе: $dst"
  else
    rm -f "$tmp"
    echo "[firmware] не удалось скопировать прошивку $title в $dst" >&2
    FAILED=1
  fi
}

# Нода для конкретного колеса: адрес и имя уже вписаны, править файл не нужно.
# Три платы, прошитые одним и тем же motor_node.ino, отвечают на одном адресе 2
# — мастер не видит ни одного колеса; готовые файлы исключают эту ошибку.
make_node() {
  local module="$1" address="$2" src dst tmp
  src="$ROOT/firmware/motor_node/motor_node.ino"
  dst="$DESK/motor_node_${module}.ino"
  if [[ ! -f "$src" ]]; then
    echo "[firmware] нет исходника прошивки ноды: $src" >&2
    FAILED=1
    return
  fi
  tmp="$dst.tmp.$$"
  if ! sed -e "s/^#define NODE_ADDRESS .*/#define NODE_ADDRESS ${address}/" \
           -e "s/^#define MODULE_ID .*/#define MODULE_ID \"${module}\"/" \
           -- "$src" >"$tmp"; then
    rm -f "$tmp"
    echo "[firmware] не удалось собрать motor_node_${module}.ino" >&2
    FAILED=1
    return
  fi
  if [[ -f "$dst" ]] && cmp -s "$tmp" "$dst"; then
    rm -f "$tmp"
    echo "[firmware] нода ${module} (адрес ${address}): на рабочем столе уже актуальная версия"
    return
  fi
  if mv -f -- "$tmp" "$dst"; then
    echo "[firmware] нода ${module} (адрес ${address}): прошивка обновлена на рабочем столе: $dst"
  else
    rm -f "$tmp"
    echo "[firmware] не удалось скопировать прошивку ноды ${module} в $dst" >&2
    FAILED=1
  fi
}

sync_one "$ROOT/firmware/motor_master/motor_master.ino" "мастер (FL)"
sync_one "$ROOT/firmware/motor_node/motor_node.ino" "нода (шаблон: адрес и имя в начале файла)"
make_node FR 2
make_node RL 3
make_node RR 4

exit "$FAILED"
