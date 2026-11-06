#!/usr/bin/env bash
# Пароль администратора через окно системы (SUDO_ASKPASS) — без терминала.
#
# Установка ROS 2 и драйверов лидара требует sudo. На роботе приложение
# запускается ярлыком, автозапуском или панелью, и терминала рядом нет:
# `sudo` без него просто отказывается спрашивать пароль, установка не идёт, а
# оператору приходилось открывать консоль и печатать команды руками.
#
# Этот скрипт вызывает `sudo -A` и показывает окно ввода пароля тем инструментом,
# который есть в системе (zenity, kdialog, yad, x11-ssh-askpass). Пароль
# печатается в stdout — так требует протокол SUDO_ASKPASS — и нигде не хранится.
#
#   SUDO_ASKPASS=scripts/askpass_gui.sh sudo -A -v      # обновить «пропуск» sudo
#
# Код 1 — окна нет (нет графической сессии или ни одного из инструментов):
# вызывающий скрипт честно скажет, что нужен пароль, и не станет врать, будто
# установка идёт.
set -u

TITLE="Права администратора"
TEXT="${1:-Для установки ROS 2 и драйверов лидара нужен пароль администратора робота.}"

if [[ -n "${RC_ASKPASS_TEXT:-}" ]]; then
  TEXT="$RC_ASKPASS_TEXT"
fi

gui() {
  [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]
}

gui || exit 1

if command -v zenity >/dev/null 2>&1; then
  exec zenity --password --title="$TITLE" --timeout=180
fi
if command -v kdialog >/dev/null 2>&1; then
  exec kdialog --title "$TITLE" --password "$TEXT"
fi
if command -v yad >/dev/null 2>&1; then
  exec yad --title="$TITLE" --text="$TEXT" --entry --hide-text --center --timeout=180
fi
for candidate in x11-ssh-askpass ssh-askpass /usr/lib/ssh/x11-ssh-askpass; do
  if command -v "$candidate" >/dev/null 2>&1; then
    exec "$candidate" "$TEXT"
  fi
done
exit 1
