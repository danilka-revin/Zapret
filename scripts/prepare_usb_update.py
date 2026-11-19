#!/usr/bin/env python3
"""Собрать готовую флешку для обновления робота без интернета.

В отличие от ``make_usb_update.py``, который только упаковывает подписанный
Git-bundle, этот скрипт делает всё остальное:

* создаёт (или переиспользует) ключ подписи выпуска;
* упаковывает последний коммит GitHub-ветки в ``robot-update.zip``;
* кладёт рядом публичный ключ ``bulka-usb-update.pub``;
* пишет ``INSTALL.txt`` с пошаговой инструкцией;
* пишет ``install-on-robot.sh`` — одноразовую настройку робота (доверенный
  ключ, автоустановка с флешки, при необходимости точка монтирования).

Публичный ключ можно проверить на роботе тем же файлом ``.pub``. Закрытый ключ
остаётся в ``bulka-release/`` и на флешку не попадает: он нужен только для
будущих выпусков.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_control import usb_update  # noqa: E402
from scripts import make_usb_update  # noqa: E402

DEFAULT_KEY = Path("bulka-release/bulka-usb-update")
DEFAULT_OUT = Path("bulka-usb")

INSTALL_TEMPLATE = """ОФЛАЙН-ОБНОВЛЕНИЕ BULKA (флешка, без интернета)
================================================

Версия пакета : {revision}
Ветка         : {branch}
Файл          : robot-update.zip ({size})
Доверенный ключ: bulka-usb-update.pub

На роботе НЕ нужен интернет. Пульт читает подписанный пакет прямо с флешки.

ЧТО УЖЕ ЕСТЬ НА ФЛЕШКЕ
---------------------
robot-update.zip       — подписанный пакет обновления (единственный нужный роботу файл);
bulka-usb-update.pub   — публичный ключ выпуска (для одноразовой настройки робота);
install-on-robot.sh    — настройка робота: доверенный ключ + автоустановка;
INSTALL.txt            — эта инструкция.

ОДИН РАЗ НА РОБОТЕ
------------------
1. Скопируйте содержимое этой папки в корень флешки и вставьте её в робот.
2. На роботе, из каталога Git-клона проекта, запустите:

       sh /путь/к/флешке/install-on-robot.sh --auto

   (если флешка смонтирована не в /media или /run/media, добавьте
   ``--mount /точка/монтирования``).

   Скрипт добавит доверенный ключ в ``data/usb-update-signers`` и включит
   автоустановку ``RC_USB_UPDATE_AUTO=1`` в ``.env``. Интернет не нужен.

3. Остановите робота и включите E-STOP. Перезапустите веб-пульт, чтобы он
   прочитал ``.env`` (./scripts/serve.sh или служба robot-control).

ДАЛЬШЕ — САМО
--------------
Пульт каждые несколько секунд проверяет флешку. С включённым E-STOP он берёт
пакет, проверяет подпись и ставит обновление. Не извлекайте флешку до конца.
Ход установки: ``var/update-state.json`` в каталоге проекта.

РУЧНОЙ ВАРИАНТ (без автоустановки)
----------------------------------
В пульте откройте «Обновить с GitHub» → «С флешки (без интернета)» →
«Проверить» → «Установить обновление». E-STOP тоже нужен.

ПРОВЕРКИ
-------
* На роботе должны быть установлены ``git``, ``ssh-keygen`` (openssh-client)
  и ``findmnt`` (util-linux).
* Проект на роботе — Git-клон на ветке ``{branch}``, рабочая копия чистая.
* ``.env``, ``data/``, ``var/`` и ``.venv/`` пакет не перезаписывает.
* Если версия в пакете не новее установленной, пульт просто напишет
  «Изменений нет».

СБОРКА НОВОГО ПАКЕТА
--------------------
На ПК с интернетом, в обычном Git-клоне проекта (не --depth 1):

    python3 scripts/prepare_usb_update.py

Ключ подписи лежит в ``bulka-release/bulka-usb-update`` (закрытый) и
``bulka-release/bulka-usb-update.pub`` (публичный). Закрытый ключ храните
отдельно и не кладите на флешку.
"""

ROBOT_SCRIPT = """#!/bin/sh
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
    printf 'bulka-update %s\\n' "$KEY_LINE" >> "$TRUST"
    echo "Доверенный ключ добавлен в $TRUST"
fi

# 2. Автоустановка с флешки. Даже с ней обновление не начнётся без E-STOP
# и доверенной подписи.
ENV_FILE=$REPO/.env
touch "$ENV_FILE"
if [ "$AUTO" = 1 ]; then
    if ! grep -Eq '^RC_USB_UPDATE_AUTO=1[[:space:]]*$' "$ENV_FILE"; then
        printf 'RC_USB_UPDATE_AUTO=1\\n' >> "$ENV_FILE"
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
                printf 'RC_USB_UPDATE_MOUNT=%s\\n' "$MOUNT" >> "$ENV_FILE"
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
"""


def ensure_key(path: Path) -> Path:
    if path.exists():
        return path
    if not shutil.which("ssh-keygen"):
        raise SystemExit("Нужен ssh-keygen (openssh-client), чтобы создать ключ подписи.")
    path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-C", "bulka-usb-update", "-f", str(path)],
        check=True,
    )
    return path


def verify_package(path: Path, public_key: Path) -> None:
    """Проверяем подпись тем же доверенным файлом, что будет на роботе.

    Не передаём ключ из аргументов: если с флешкой уедет другой ``.pub``,
    проверка покажет это прямо при сборке.
    """
    import tempfile

    with tempfile.TemporaryDirectory(prefix="bulka-kit-check-") as tmp:
        trust = Path(tmp) / "usb-update-signers"
        trust.write_text(f"{usb_update.SIGNER} {public_key.read_text().strip()}\n", encoding="utf-8")
        with usb_update.verified_package(path, trust) as (stream, manifest):
            usb_update.verify_bundle_bytes(stream, manifest)
            print(f"Подпись и контрольная сумма в порядке: {manifest.branch}@{manifest.revision[:12]}")


def cleanup_temp_keys() -> None:
    """Сборка могла создать временные ``.sig`` рядом с ключом — убираем."""
    for path in DEFAULT_KEY.parent.glob("*.sig") if DEFAULT_KEY.parent.is_dir() else ():
        path.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Собрать флешку для офлайн-обновления BULKA.")
    parser.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT,
                        help=f"папка флешки (по умолчанию {DEFAULT_OUT})")
    parser.add_argument("--key", type=Path, default=DEFAULT_KEY,
                        help=f"закрытый ключ подписи (по умолчанию {DEFAULT_KEY})")
    parser.add_argument("--branch", default="main", help="ветка GitHub (по умолчанию main)")
    parser.add_argument("--revision", help="конкретный коммит из истории ветки")
    parser.add_argument("--base", help="полный хеш текущей версии робота (пакет только с изменениями)")
    parser.add_argument("--no-zip", action="store_true",
                        help=f"создать {usb_update.PACKAGE_NAME} вместо {usb_update.ZIP_NAME}")
    args = parser.parse_args()

    key = ensure_key(args.key.expanduser().resolve())
    public_key = key.with_suffix(".pub")
    out = args.output.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    try:
        package, revision = make_usb_update.make_package(
            ROOT, out, key, branch=args.branch, base=args.base,
            as_zip=not args.no_zip, target_revision=args.revision)
    except (make_usb_update.UsbUpdateError, subprocess.CalledProcessError, OSError) as exc:
        cleanup_temp_keys()
        print(f"Не удалось собрать пакет: {exc}", file=sys.stderr)
        return 1

    shutil.copyfile(public_key, out / public_key.name)
    (out / "install-on-robot.sh").write_text(ROBOT_SCRIPT, encoding="utf-8")
    os.chmod(out / "install-on-robot.sh", 0o755)
    size = package.stat().st_size
    (out / "INSTALL.txt").write_text(
        INSTALL_TEMPLATE.format(revision=revision, branch=args.branch, size=f"{size / (1 << 20):.0f} МиБ"),
        encoding="utf-8")

    try:
        verify_package(package, public_key)
    except Exception as exc:  # noqa: BLE001 — сборка уже прошла, но проверка обязательна
        cleanup_temp_keys()
        print(f"Пакет собран, но проверить подпись не удалось: {exc}", file=sys.stderr)
        return 1
    cleanup_temp_keys()

    print(f"Флешка готова: {out}")
    print(f"  Пакет     : {package.name} ({size / (1 << 20):.0f} МиБ)")
    print(f"  Версия    : {revision}")
    print(f"  Ключ      : {public_key}")
    print("  На флешку кладите только файлы из этой папки; закрытый ключ — нет.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
