#!/usr/bin/env bash
# =============================================================================
# Zapret Control — установка / запуск / обновление одной командой.
#
#   curl -fsSL https://raw.githubusercontent.com/danilka-revin/Zapret/main/install.sh \
#     -o /tmp/zc-install.sh && bash /tmp/zc-install.sh
#
# Команды:
#   (без аргументов)   установить (или обновить), скачать зависимости,
#                      создать ярлык и запустить приложение
#   update             обновить код и зависимости, перезапустить интерфейс
#   gui | launch       только запустить интерфейс (с проверкой, что окно живое)
#   repair             починить установку: ярлык на столе, права, перенос из /root
#   doctor             самодиагностика: что мешает запуску
#   uninstall          полностью удалить (служба, ярлык, данные, sudoers)
#
# Ключи:
#   --user ИМЯ     для кого ставить (по умолчанию — пользователь рабочего стола)
#   --app-dir ПУТЬ куда ставить код (по умолчанию ~/.local/share/zapret-control)
#   --no-launch    не запускать интерфейс после установки
#   --no-deps      не скачивать nfqws и стратегии (скачает приложение само)
#   --help         краткая справка
#
# Приложение можно запускать и через sudo, и из-под обычного пользователя:
# код, зависимости, ярлык и права NOPASSWD всегда оформляются на живого
# пользователя рабочего стола, а не на root (иначе окно не появляется, а на
# столе нет значка).
#
# Интерфейс написан на Qt6 (PySide6) — установщик сам ставит Qt-библиотеки
# и модуль PySide6-Essentials, отдельных действий от пользователя не нужно.
# =============================================================================

set -euo pipefail

# --- пути и настройки --------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_URL="${ZAPRET_REPO_URL:-https://github.com/danilka-revin/Zapret}"
BRANCH="${ZAPRET_BRANCH:-main}"
REPO_SLUG="$(printf '%s' "$REPO_URL" | sed -e 's#^https\{0,1\}://[^/]*/##' -e 's#\.git$##' || true)"
APP_ID="zapret-control"
SYS_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

C_G="\033[32m"; C_B="\033[34m"; C_Y="\033[33m"; C_R="\033[31m"; C_N="\033[0m"
info()   { echo -e "${C_B}[*]${C_N} $*"; }
ok()     { echo -e "${C_G}[+]${C_N} $*"; }
warn()   { echo -e "${C_Y}[!]${C_N} $*"; }
fail()   { echo -e "${C_R}[-]${C_N} $*" >&2; exit 1; }
indent() { sed 's/^/    /'; }

usage() {
    cat <<'HELP'
Zapret Control — установка одной командой.

  bash install.sh                установить/обновить, скачать зависимости,
                                 создать ярлык и запустить интерфейс
  bash install.sh update         обновить код + зависимости и перезапустить
  bash install.sh gui            только запустить интерфейс
  bash install.sh repair         починить установку (ярлык на столе, права,
                                 перенос из /root, запуск окна)
  bash install.sh doctor         диагностика: что мешает работе
  bash install.sh status         краткая сводка об установке
  bash install.sh uninstall      удалить всё (служба, ярлык, данные, sudoers)

Ключи:
  --user ИМЯ       установить для конкретного пользователя
  --app-dir ПУТЬ   каталог приложения (по умолчанию ~/.local/share/zapret-control)
  --no-launch      не запускать интерфейс
  --no-deps        не скачивать nfqws и стратегии
HELP
}

# --- разбор аргументов -------------------------------------------------------
CMD="install"
RUN_USER_REQUESTED="${ZAPRET_USER:-}"
APP_DIR="${ZAPRET_APP_DIR:-}"
DO_LAUNCH=1
DO_DEPS=1

while [ $# -gt 0 ]; do
    case "$1" in
        install|update|gui|launch|run|repair|doctor|status|uninstall) CMD="$1" ;;
        --user)      RUN_USER_REQUESTED="${2:-}"; shift ;;
        --user=*)    RUN_USER_REQUESTED="${1#*=}" ;;
        --app-dir)   APP_DIR="${2:-}"; shift ;;
        --app-dir=*) APP_DIR="${1#*=}" ;;
        --no-launch) DO_LAUNCH=0 ;;
        --no-deps)   DO_DEPS=0 ;;
        -h|--help|help) usage; exit 0 ;;
        *) warn "Непонятный аргумент: $1 (поможет --help)" ;;
    esac
    shift
done

# --- для кого устанавливаем ---------------------------------------------------
AM_ROOT=0
[ "$(id -u)" -eq 0 ] && AM_ROOT=1

human_user() {
    # существует, не root и имеет домашний каталог пользователя
    local name="$1" home
    [ -n "$name" ] || return 1
    [ "$name" != "root" ] || return 1
    id "$name" >/dev/null 2>&1 || return 1
    home="$(getent passwd "$name" 2>/dev/null | cut -d: -f6 || true)"
    case "$home" in /home/*|/var/home/*|/data/*|/mnt/*) return 0 ;; *) return 1 ;; esac
}

active_desktop_user() {
    local name id type remote active uid dir
    # 1) sudo -i / sudo bash → SUDO_USER
    if [ -n "${SUDO_USER:-}" ] && human_user "$SUDO_USER"; then
        echo "$SUDO_USER"; return 0
    fi
    # 2) активная графическая сессия в logind
    if command -v loginctl >/dev/null 2>&1; then
        for id in $(loginctl list-sessions --no-legend 2>/dev/null | awk '{print $1}' || true); do
            case "$id" in ""|*[!0-9]*) continue ;; esac
            type="$(loginctl show-session "$id" -p Type -H 2>/dev/null | cut -d= -f2 || true)"
            remote="$(loginctl show-session "$id" -p Remote -H 2>/dev/null | cut -d= -f2 || true)"
            active="$(loginctl show-session "$id" -p Active -H 2>/dev/null | cut -d= -f2 || true)"
            name="$(loginctl show-session "$id" -p User -H 2>/dev/null | cut -d= -f2 || true)"
            [ "$remote" = "yes" ] && continue
            [ "$active" = "yes" ] || continue
            case "$type" in x11|wayland|mir) ;; *) continue ;; esac
            if human_user "$name"; then echo "$name"; return 0; fi
        done
    fi
    # 3) каталоги /run/user/<uid> — бывают только у живых сеансов
    if [ -d /run/user ]; then
        for dir in $(ls -1dt /run/user/*/ 2>/dev/null || true); do
            uid="$(basename "$dir" 2>/dev/null || true)"
            case "$uid" in ""|*[!0-9]*) continue ;; esac
            [ "$uid" -ge 1000 ] || continue
            name="$(id -un "$uid" 2>/dev/null || true)"
            if human_user "$name"; then echo "$name"; return 0; fi
        done
    fi
    # 4) кто сидит за консолью
    if command -v w >/dev/null 2>&1; then
        name="$(w -h 2>/dev/null | awk '{print $1}' | grep -vx root | head -1 || true)"
        if human_user "$name"; then echo "$name"; return 0; fi
    fi
    # 5) единственный обычный пользователь в системе
    if [ "$(awk -F: '$3>=1000 && $3<65000 && $7!~/nologin|false/ {print $1}' /etc/passwd \
            | grep -vx root | wc -l || true)" = "1" ]; then
        name="$(awk -F: '$3>=1000 && $3<65000 && $7!~/nologin|false/ {print $1}' /etc/passwd \
                 | grep -vx root | head -1 || true)"
        if [ -n "$name" ]; then echo "$name"; return 0; fi
    fi
    return 1
}

if [ -n "$RUN_USER_REQUESTED" ] && ! human_user "$RUN_USER_REQUESTED"; then
    warn "Пользователь '$RUN_USER_REQUESTED' не найден (или это root) — определяю сам"
    RUN_USER_REQUESTED=""
fi
DETECTED_USER=""
if [ -n "$RUN_USER_REQUESTED" ]; then
    DETECTED_USER="$RUN_USER_REQUESTED"
else
    DETECTED_USER="$(active_desktop_user 2>/dev/null || true)"
fi
if [ -z "$DETECTED_USER" ]; then
    if [ "$AM_ROOT" -eq 1 ]; then
        warn "Не удалось найти пользователя рабочего стола — ставлю в $(getent passwd root | cut -d: -f6)"
    fi
    DETECTED_USER="$(id -un)"
fi

RUN_USER="$DETECTED_USER"
RUN_UID="$(id -u "$RUN_USER" 2>/dev/null || id -u)"
RUN_GID="$(id -g "$RUN_USER" 2>/dev/null || id -g)"
RUN_HOME="$(getent passwd "$RUN_UID" 2>/dev/null | cut -d: -f6 || true)"
if [ -z "$RUN_HOME" ]; then
    RUN_HOME="$(eval echo "~$RUN_USER" 2>/dev/null || echo "$HOME")"
fi
[ -d "$RUN_HOME" ] || RUN_HOME="$HOME"

if [ -z "$APP_DIR" ]; then
    APP_DIR="$RUN_HOME/.local/share/$APP_ID"
fi
APP_LOG="$APP_DIR/zapret-control.log"

# Окружение для «пользовательских» шагов: pip --user, git, ярлыки, запуск окна.
TARGET_ENV=(
    "HOME=$RUN_HOME"
    "USER=$RUN_USER"
    "LOGNAME=$RUN_USER"
    "PATH=$RUN_HOME/.local/bin:$SYS_PATH"
    "ZAPRET_APP_DIR=$APP_DIR"
    "SUDO_USER=$RUN_USER"
)
if [ -d "/run/user/$RUN_UID" ]; then
    TARGET_ENV+=("XDG_RUNTIME_DIR=/run/user/$RUN_UID")
fi
if [ -n "${LANG:-}" ]; then
    TARGET_ENV+=("LANG=$LANG")
fi
if [ -n "${XDG_SESSION_TYPE:-}" ]; then
    TARGET_ENV+=("XDG_SESSION_TYPE=$XDG_SESSION_TYPE")
fi

if command -v sudo >/dev/null 2>&1; then SUDO="sudo"; else SUDO=""; fi

# --- выполнение команд от имени нужного пользователя / root ------------------
_user_prefix() {
    if [ "$AM_ROOT" -eq 1 ] && [ "$RUN_UID" != "0" ]; then
        if command -v runuser >/dev/null 2>&1; then
            runuser -u "$RUN_USER" -- "$@"
        elif command -v setpriv >/dev/null 2>&1; then
            setpriv --reuid="$RUN_UID" --regid="$RUN_GID" --init-groups "$@"
        elif command -v sudo >/dev/null 2>&1; then
            sudo -u "$RUN_USER" "$@"
        elif command -v su >/dev/null 2>&1; then
            su "$RUN_USER" -c "$(printf '%q ' "$@")"
        else
            fail "Нет runuser/setpriv/sudo — запустите установку от пользователя $RUN_USER"
        fi
    else
        "$@"
    fi
}

as_user() { _user_prefix env -i "${TARGET_ENV[@]}" "$@"; }
as_root() {
    if [ "$AM_ROOT" -eq 1 ]; then
        "$@"
    elif [ -n "$SUDO" ]; then
        $SUDO "$@"
    else
        "$@"
    fi
}
py_user() { as_user python3 "$@"; }
py_root() { as_root env "${TARGET_ENV[@]}" python3 "$@"; }

is_source_tree() { [ -f "$SCRIPT_DIR/run.py" ]; }
app_ready() { [ -f "$APP_DIR/run.py" ]; }

ensure_app_dir() {
    as_user mkdir -p "$APP_DIR" || fail "Не могу создать $APP_DIR"
    if [ "$AM_ROOT" -eq 1 ] && [ "$RUN_UID" != "0" ]; then
        as_root chown -R "$RUN_UID:$RUN_GID" "$APP_DIR" 2>/dev/null || true
    fi
}

# --- перенос «корневой» установки (следствие установки через sudo -i) --------
relocate_root_install() {
    [ "$AM_ROOT" -eq 1 ] && [ "$RUN_UID" != "0" ] || return 0
    local old="/root/.local/share/$APP_ID" item
    [ -d "$old" ] || return 0
    info "Найдена установка в $old — переношу зависимости в $APP_DIR"
    ensure_app_dir
    for item in nfqws deps cache config.json update.json; do
        if [ -e "$old/$item" ] && [ ! -e "$APP_DIR/$item" ]; then
            mv "$old/$item" "$APP_DIR/$item" 2>/dev/null \
                || cp -a "$old/$item" "$APP_DIR/$item" 2>/dev/null || true
        fi
    done
    rm -rf "$old" 2>/dev/null || true
    rm -f "/root/.local/share/applications/$APP_ID.desktop" 2>/dev/null || true
    as_root chown -R "$RUN_UID:$RUN_GID" "$APP_DIR" 2>/dev/null || true
    ok "Готово: скачанные файлы используем как есть."
}

# --- установка недостающих системных пакетов ---------------------------------
# Обязательные библиотеки Qt6 (без них интерфейс не запустится вовсе)
declare -A QT_CORE_PACKAGES=(
    [apt-get]="libgl1 libegl1 libxkbcommon0 libdbus-1-3 libfontconfig1 libglib2.0-0 python3-pip"
    [dnf]="mesa-libGL mesa-libEGL libxkbcommon dbus-libs python3-pip"
    [pacman]="libglvnd libxkbcommon dbus fontconfig python-pip"
    [zypper]="libGL1 libxkbcommon0 libdbus-1-3 fontconfig python3-pip"
    [apk]="mesa-gl libxkbcommon dbus-libs fontconfig py3-pip"
    [xbps-install]="libglvnd libxkbcommon dbus fontconfig python3-pip"
    [emerge]="virtual/opengl x11-libs/libxkbcommon dev-libs/dbus-glib"
)

# Дополнительные библиотеки для плагина xcb (окна X11) и Wayland.
# Ставятся «по возможности»: если у дистрибутива другое имя пакета, установка
# не срывается, а пользователь получает точную подсказку из run.py doctor.
declare -A QT_XCB_PACKAGES=(
    [apt-get]="libxkbcommon-x11-0 libxcb1 libxcb-cursor0 libxcb-xinerama0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 libxcb-shape0 libwayland-client0 libwayland-cursor0 libwayland-egl1"
    [dnf]="libxkbcommon-x11 xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil"
    [pacman]="libxkbcommon-x11 xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil wayland"
    [zypper]="libxkbcommon-x11-0 xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil"
    [apk]="libxcb xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil"
    [xbps-install]="libxkbcommon-x11 libxcb xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil"
    [emerge]="x11-libs/libxkbcommon-x11 x11-libs/xcb-util-cursor x11-libs/xcb-util-wm x11-libs/xcb-util-image x11-libs/xcb-util-keysyms x11-libs/xcb-util-renderutil"
)

detect_pm() {
    local c
    for c in apt-get dnf pacman zypper apk xbps-install emerge; do
        if command -v "$c" >/dev/null 2>&1; then echo "$c"; return 0; fi
    done
    echo ""
}

install_packages() {
    local missing=()
    local qt_missing=0
    if as_user python3 -c "import PySide6" >/dev/null 2>&1; then
        :
    elif as_user python3 --version >/dev/null 2>&1; then
        qt_missing=1
    else
        missing+=(python3)
        qt_missing=1
    fi
    command -v git  >/dev/null 2>&1 || missing+=(git)
    command -v curl >/dev/null 2>&1 || missing+=(curl)
    if ! command -v nft >/dev/null 2>&1 && ! command -v iptables >/dev/null 2>&1; then
        missing+=(nftables)
    fi
    if [ ${#missing[@]} -eq 0 ] && [ "$qt_missing" -eq 0 ]; then
        return 0
    fi

    info "Требуются системные пакеты: ${missing[*]:-нет}"
    local pm=""
    pm="$(detect_pm || true)"
    if [ -z "$pm" ]; then
        warn "Пакетный менеджер не определён. Установите вручную: ${missing[*]}"
        return 0
    fi

    if [ "$qt_missing" -eq 1 ] && [ -n "${QT_CORE_PACKAGES[$pm]:-}" ]; then
        # shellcheck disable=SC2206 — список пакетов разбиваем по пробелам
        missing+=(${QT_CORE_PACKAGES[$pm]})
        info "Добавляю библиотеки Qt6: ${QT_CORE_PACKAGES[$pm]}"
    fi

    local rc=0
    case "$pm" in
        apt-get)
            as_root apt-get update -y >/dev/null 2>&1 || true
            as_root apt-get install -y "${missing[@]}" >/dev/null 2>&1 || rc=1
            ;;
        dnf)          as_root dnf install -y "${missing[@]}" >/dev/null 2>&1 || rc=1 ;;
        pacman)       as_root pacman -S --needed --noconfirm "${missing[@]}" >/dev/null 2>&1 || rc=1 ;;
        zypper)       as_root zypper --non-interactive install "${missing[@]}" >/dev/null 2>&1 || rc=1 ;;
        apk)          as_root apk add "${missing[@]}" >/dev/null 2>&1 || rc=1 ;;
        xbps-install) as_root xbps-install -y "${missing[@]}" >/dev/null 2>&1 || rc=1 ;;
        emerge)       as_root emerge -av "${missing[@]}" >/dev/null 2>&1 || rc=1 ;;
    esac
    if [ "$rc" -eq 0 ]; then
        ok "Системные пакеты установлены."
    else
        warn "Не удалось установить пакеты автоматически. Установите их вручную: ${missing[*]}"
    fi

    # Пакеты для плагина xcb — необязательный шаг, ошибки только предупреждаем
    if [ "$qt_missing" -eq 1 ] && [ -n "${QT_XCB_PACKAGES[$pm]:-}" ]; then
        info "Дополнительно ставлю библиотеки окон (xcb/Wayland)…"
        case "$pm" in
            apt-get)      as_root apt-get install -y ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            dnf)          as_root dnf install -y ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            pacman)       as_root pacman -S --needed --noconfirm ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            zypper)       as_root zypper --non-interactive install ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            apk)          as_root apk add ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            xbps-install) as_root xbps-install -y ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            emerge)       as_root emerge -n ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
        esac
    fi
}

# --- установка/обновление кода приложения ------------------------------------
sync_local_tree() {
    # копируем исходники в APP_DIR, не трогая зависимости и кэш
    ensure_app_dir
    tar -C "$SCRIPT_DIR" \
        --exclude=.git --exclude=deps --exclude=cache \
        --exclude=__pycache__ --exclude='*.pyc' \
        -cf - . | as_user sh -c "tar -C '$APP_DIR' -xf -"
    as_root chown -R "$RUN_UID:$RUN_GID" "$APP_DIR" 2>/dev/null || true
    ok "Код скопирован в $APP_DIR"
}

fetch_archive() {
    # Запасной путь без git: tar.gz из codeload (работает и при блокировке git-протокола)
    local tmp dest root rc=0
    tmp="$(mktemp -d || echo /tmp/zc-fetch-$$)"
    dest="$tmp/extract"
    mkdir -p "$dest" 2>/dev/null || return 1
    info "Пробую скачать архив кода…"
    if curl -fsSL "https://codeload.github.com/$REPO_SLUG/tar.gz/refs/heads/$BRANCH" \
            2>/dev/null | tar -xz -C "$dest" 2>/dev/null; then
        root="$(find "$dest" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | head -1 || true)"
        if [ -n "$root" ]; then
            ensure_app_dir
            (cd "$root" && tar -cf - .) | as_user sh -c "tar -C '$APP_DIR' -xf -" || rc=1
            as_root chown -R "$RUN_UID:$RUN_GID" "$APP_DIR" 2>/dev/null || true
            rm -rf "$tmp" 2>/dev/null || true
            if [ "$rc" -eq 0 ]; then
                ok "Код загружен из архива."
                return 0
            fi
        fi
    fi
    rm -rf "$tmp" 2>/dev/null || true
    return 1
}

install_code() {
    ensure_app_dir
    if is_source_tree; then
        if [ -d "$SCRIPT_DIR/.git" ] && command -v git >/dev/null 2>&1; then
            info "Обновление исходников (git pull)…"
            as_user git -C "$SCRIPT_DIR" pull --ff-only 2>&1 | indent \
                || warn "git pull не удался — использую текущую версию."
        fi
        sync_local_tree
    elif [ -d "$APP_DIR/.git" ] && command -v git >/dev/null 2>&1; then
        info "Обновление кода…"
        as_user git -C "$APP_DIR" fetch --depth 1 origin "$BRANCH" >/dev/null 2>&1 \
            && as_user git -C "$APP_DIR" reset --hard FETCH_HEAD >/dev/null 2>&1 \
            || warn "Обновление кода не удалось — оставляю текущую версию."
    elif command -v git >/dev/null 2>&1; then
        info "Загрузка приложения из $REPO_URL (пользователь $RUN_USER)…"
        if ! as_user git clone --depth 1 --branch "$BRANCH" "$REPO_URL" "$APP_DIR" \
                >/dev/null 2>&1; then
            fetch_archive || fail "Не удалось получить код из $REPO_URL"
        fi
        ok "Приложение загружено."
    else
        fetch_archive || fail "Нет git, архив скачать не удалось"
    fi
    app_ready || fail "Код не на месте: $APP_DIR/run.py не найден"
    py_user "$APP_DIR/run.py" record-version >/dev/null 2>&1 || true
}

install_pyside() {
    if as_user python3 -c "import PySide6" >/dev/null 2>&1; then
        ok "Qt6-интерфейс (PySide6) уже установлен."
        return 0
    fi
    info "Устанавливаю Qt6-интерфейс (PySide6-Essentials) для $RUN_USER…"
    if ! as_user python3 -m pip --version >/dev/null 2>&1; then
        warn "pip недоступен — приложение поставит PySide6 само при первом запуске."
        return 1
    fi
    local installed=0 args
    for args in "--user" "--user --break-system-packages" "--break-system-packages"; do
        # shellcheck disable=SC2086 — аргументы pip разбираем как список
        if as_user python3 -m pip install $args --quiet --disable-pip-version-check \
                PySide6-Essentials >/dev/null 2>&1; then
            installed=1
            break
        fi
    done
    if [ "$installed" -eq 1 ] && as_user python3 -c "import PySide6" >/dev/null 2>&1; then
        ok "PySide6 установлен."
        return 0
    fi
    warn "Не удалось поставить PySide6 автоматически — интерфейс попробует при запуске."
    warn "Вручную (от имени $RUN_USER): python3 -m pip install --user PySide6-Essentials"
    return 1
}

ensure_dependencies() {
    [ "$DO_DEPS" -eq 1 ] || return 0
    info "Скачивание зависимостей (nfqws + стратегии)…"
    if ! py_user "$APP_DIR/run.py" ensure-deps 2>&1 | indent; then
        warn "Скачивание зависимостей не удалось — приложение догрузит их само."
    fi
}

install_desktop_shortcut() {
    info "Ярлык: меню приложений + рабочий стол…"
    if py_user "$APP_DIR/run.py" shortcut install 2>&1 | indent; then
        return 0
    fi
    warn "Ярлык не создан — кладу простую .desktop-запись вручную."
    local dir="$RUN_HOME/.local/share/applications"
    if as_user mkdir -p "$dir" 2>/dev/null; then
        as_user sh -c "cat > '$dir/$APP_ID.desktop'" <<EOF || true
[Desktop Entry]
Type=Application
Name=Zapret Control
Comment=Обход замедления YouTube, Discord и Telegram
Exec=python3 $APP_DIR/run.py gui
Icon=$APP_ID
Terminal=false
Categories=Network;Utility;System;
EOF
        as_user chmod 644 "$dir/$APP_ID.desktop" 2>/dev/null || true
        ok "Ярлык в меню: $dir/$APP_ID.desktop"
    fi
}

launch_gui() {
    [ "$DO_LAUNCH" -eq 1 ] || return 0
    if [ ! -d "$RUN_HOME" ]; then
        warn "Домашний каталог $RUN_HOME недоступен — интерфейс не запускаю."
        return 0
    fi
    info "Запуск интерфейса (пользователь $RUN_USER)…"
    local output
    if output="$(py_user "$APP_DIR/run.py" launch 2>&1)"; then
        printf '%s\n' "$output" | indent
        return 0
    fi
    printf '%s\n' "$output" | indent
    warn "Окно не появилось — diagnostics:"
    py_user "$APP_DIR/run.py" doctor 2>&1 | tail -40 | indent || true
    warn "Починка одной командой: bash $0 repair"
    return 1
}

print_summary() {
    echo ""
    ok "Zapret Control готов."
    cat <<EOF

    код и данные    $APP_DIR
    пользователь    $RUN_USER
    запуск          python3 $APP_DIR/run.py launch   (или значок на рабочем столе)
    лог             $APP_LOG
    диагностика    python3 $APP_DIR/run.py doctor
    починка         bash $0 repair
    обновление     кнопка «Обновить и перезапустить» в окне или bash $0 update

EOF
}

# ---------------------------------------------------------------------- ремонт
# Подкоманду `repair` знает только свежий run.py. Если в каталоге установки лежит
# копия старше этой команды, чиним кодом рядом с установщиком — молча печатать
# usage вместо починки нельзя.
run_repair() {
    local entry=""
    if [ -f "$APP_DIR/run.py" ] && grep -q '"repair"' "$APP_DIR/run.py" 2>/dev/null; then
        entry="$APP_DIR/run.py"
    elif [ -f "$SCRIPT_DIR/run.py" ]; then
        entry="$SCRIPT_DIR/run.py"
    fi
    if [ -z "$entry" ]; then
        warn "Нечем запускать починку — обновите установщик:"
        echo "  curl -fsSL https://raw.githubusercontent.com/danilka-revin/Zapret/$BRANCH/install.sh -o /tmp/zc-install.sh && bash /tmp/zc-install.sh repair"
        return 0
    fi
    local extra=("repair" "--no-launch")
    [ "$DO_DEPS" -eq 1 ] || extra+=("--no-deps")
    if [ "$(id -u)" -eq 0 ]; then
        py_root "$entry" "${extra[@]}"
    else
        env "${TARGET_ENV[@]}" python3 "$entry" "${extra[@]}"
    fi
}

# --- команды ------------------------------------------------------------------
case "$CMD" in
    uninstall)
        warn "Удаление Zapret Control…"
        if app_ready; then
            py_user "$APP_DIR/run.py" service remove >/dev/null 2>&1 || true
            py_user "$APP_DIR/run.py" shortcut remove >/dev/null 2>&1 || true
        fi
        as_root rm -f "/etc/sudoers.d/$APP_ID" >/dev/null 2>&1 || true
        rm -rf "$APP_DIR" 2>/dev/null || true
        rm -rf "/root/.local/share/$APP_ID" 2>/dev/null || true
        for dir in /home/*; do
            [ -d "$dir" ] || continue
            rm -f "$dir/.local/share/applications/$APP_ID.desktop" 2>/dev/null || true
            rm -f "$dir/Desktop/$APP_ID.desktop" 2>/dev/null || true
            rm -f "$dir/Рабочий стол/$APP_ID.desktop" 2>/dev/null || true
            rm -rf "$dir/.local/share/icons/hicolor"/*/"apps/$APP_ID.png" 2>/dev/null || true
        done
        rm -f /tmp/zapret-control.log 2>/dev/null || true
        ok "Приложение удалено (системные пакеты Qt я не трогаю)."
        ;;

    gui|launch|run)
        app_ready || install_code
        launch_gui || true
        ;;

    repair)
        info "Починка установки (пользователь $RUN_USER)…"
        app_ready || install_code
        run_repair 2>&1 | indent || true
        if [ "$DO_LAUNCH" -eq 1 ]; then
            launch_gui || true
        fi
        print_summary
        ;;

    doctor)
        if app_ready; then
            py_user "$APP_DIR/run.py" doctor
        else
            info "Установка ещё не выполнена — сначала: bash install.sh"
        fi
        ;;

    status)
        echo "Пользователь:  $RUN_USER"
        echo "Каталог:       $APP_DIR"
        if app_ready; then
            echo "Версия:        $(py_user "$APP_DIR/run.py" version 2>/dev/null || echo '?')"
            py_user "$APP_DIR/run.py" report 2>/dev/null | indent || true
        else
            echo "Состояние:     не установлено (запустите: bash install.sh)"
        fi
        ;;

    update)
        info "Обновление Zapret Control…"
        install_packages
        if app_ready; then
            update_args=("update")
            if [ "$DO_LAUNCH" -eq 1 ]; then
                update_args+=("--restart")
            fi
            if ! py_user "$APP_DIR/run.py" "${update_args[@]}" 2>&1 | indent; then
                warn "Самообновление не удалось — ставлю вручную (код + зависимости + ярлык)…"
                install_code
                install_pyside || true
                ensure_dependencies
                install_desktop_shortcut
            fi
        else
            install_code
            install_pyside || true
            ensure_dependencies
            install_desktop_shortcut
        fi
        ok "Обновление завершено."
        ;;

    install|*)
        info "Установка Zapret Control… (пользователь: $RUN_USER)"
        install_packages
        install_pyside || true
        relocate_root_install
        install_code
        ensure_dependencies
        install_desktop_shortcut
        echo ""
        info "Настройка прав без пароля (nft/nfqws) для $RUN_USER…"
        if [ "$AM_ROOT" -eq 1 ] || [ -n "$SUDO" ]; then
            if ! py_root "$APP_DIR/run.py" permissions install 2>&1 | indent; then
                warn "Права не настроены — сделайте это из приложения (кнопка «Права без пароля»)."
            fi
        else
            warn "sudo не найден — права без пароля не настроены (кнопка в приложении)."
        fi
        if [ "$AM_ROOT" -eq 1 ] && [ "$RUN_UID" != "0" ]; then
            info "Раздаю файлы каталога пользователю $RUN_USER…"
            as_root chown -R "$RUN_UID:$RUN_GID" "$APP_DIR" 2>/dev/null || true
        fi
        launch_gui || true
        print_summary
        ;;
esac
