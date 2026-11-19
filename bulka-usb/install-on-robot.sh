#!/bin/sh
# Одноразовая настройка робота для обновления с флешки без интернета.
# Запускать на роботе из каталога Git-клона проекта:
#   sh /путь/к/флешке/install-on-robot.sh --auto
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO=$PWD
AUTO=1
MOUNT=

usage() {
    cat <<'EOF'
Использование: install-on-robot.sh [--auto|--no-auto] [--mount /точка]

  --auto      включить RC_USB_UPDATE_AUTO=1 в .env (по умолчанию);
  --no-auto   выключить автоустановку;
  --mount     точка монтирования флешки, если она не в /media или /run/media.

Запускайте из каталога Git-клона проекта робота.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --auto) AUTO=1 ;;
        --no-auto) AUTO=0 ;;
        --mount) shift; MOUNT=${1:-} ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Неизвестный аргумент: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

die() { echo "Ошибка: $*" >&2; exit 1; }

command -v git >/dev/null 2>&1 || die "нужен git."
command -v ssh-keygen >/dev/null 2>&1 || die "нужен ssh-keygen (openssh-client)."
command -v findmnt >/dev/null 2>&1 || die "нужен findmnt (util-linux)."
[ -d "$REPO/.git" ] || die "запустите скрипт из каталога Git-клона проекта (нет .git)."

PUB=$SCRIPT_DIR/bulka-usb-update.pub
PACKAGE=$SCRIPT_DIR/robot-update.zip
[ -f "$PUB" ] || die "рядом со скриптом нет bulka-usb-update.pub."
[ -f "$PACKAGE" ] || die "рядом со скриптом нет robot-update.zip."

# 1. Доверенный ключ выпуска — один раз. Не верим ключу, найденному в других местах:
# добавляем ровно тот, который пришёл в комплекте с пакетом.
TRUST=$REPO/data/usb-update-signers
mkdir -p "$REPO/data"
KEY_LINE=$(cat "$PUB")
if grep -Fq "bulka-update $KEY_LINE" "$TRUST" 2>/dev/null; then
    echo "Доверенный ключ уже есть в $TRUST"
else
    printf 'bulka-update %s\n' "$KEY_LINE" >> "$TRUST"
    echo "Доверенный ключ добавлен в $TRUST"
fi

# 2. Автоустановка с флешки. Даже с ней обновление не начнётся без E-STOP
# и доверенной подписи.
ENV_FILE=$REPO/.env
touch "$ENV_FILE"
if [ "$AUTO" = 1 ]; then
    if ! grep -Eq '^RC_USB_UPDATE_AUTO=1[[:space:]]*$' "$ENV_FILE"; then
        printf 'RC_USB_UPDATE_AUTO=1\n' >> "$ENV_FILE"
        echo "Включена автоустановка с флешки (RC_USB_UPDATE_AUTO=1)."
    else
        echo "Автоустановка уже включена."
    fi
else
    if grep -Eq '^RC_USB_UPDATE_AUTO=' "$ENV_FILE"; then
        sed -i 's/^RC_USB_UPDATE_AUTO=.*/RC_USB_UPDATE_AUTO=0/' "$ENV_FILE"
        echo "Автоустановка выключена (RC_USB_UPDATE_AUTO=0)."
    fi
fi

# 3. Точка монтирования. Пульт ищет пакет в корне томов под /media и /run/media,
# либо в точке RC_USB_UPDATE_MOUNT. Если флешка примонтирована в другое место,
# задаём её явно (и только если это не корневая файловая система).
if [ -z "$MOUNT" ]; then
    MOUNT=$(findmnt -rn -o TARGET --target "$PACKAGE" 2>/dev/null || true)
    MOUNT_GIVEN=0
else
    MOUNT_GIVEN=1
fi
case "$MOUNT" in
    ""|"/") : ;;
    "/media/"*|"/run/media/"*) : ;;
    *)
        if [ "$MOUNT_GIVEN" = 1 ]; then
            if grep -Eq '^RC_USB_UPDATE_MOUNT=' "$ENV_FILE"; then
                sed -i "s|^RC_USB_UPDATE_MOUNT=.*|RC_USB_UPDATE_MOUNT=$MOUNT|" "$ENV_FILE"
            else
                printf 'RC_USB_UPDATE_MOUNT=%s\n' "$MOUNT" >> "$ENV_FILE"
            fi
            echo "Указана точка монтирования флешки: $MOUNT"
        else
            echo "Флешка смонтирована в $MOUNT. Если пульт её не найдёт,"
            echo "добавьте RC_USB_UPDATE_MOUNT=$MOUNT в .env или запустите"
            echo "скрипт ещё раз с аргументом --mount $MOUNT."
        fi
        ;;
esac

echo
echo "Готово. Дальше:"
echo "  1. Остановите робота и включите E-STOP."
echo "  2. Перезапустите веб-пульт, чтобы он прочитал .env."
echo "  3. Оставьте флешку в роботе: пульт проверит подпись и установит пакет."
echo "  4. Ход установки: $REPO/var/update-state.json"
