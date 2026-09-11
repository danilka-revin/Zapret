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
#   update             обновить код и зависимости
#   gui                только запустить интерфейс
#   uninstall          полностью удалить (служба, ярлык, данные)
#
# Интерфейс написан на Qt6 (PySide6) — установщик сам ставит Qt-библиотеки
# и модуль PySide6-Essentials, отдельных действий от пользователя не нужно.
# =============================================================================

set -euo pipefail

# --- пути и настройки --------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="${ZAPRET_APP_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/zapret-control}"
REPO_URL="${ZAPRET_REPO_URL:-https://github.com/danilka-revin/Zapret}"
BRANCH="${ZAPRET_BRANCH:-main}"

C_G="\033[32m"; C_B="\033[34m"; C_Y="\033[33m"; C_R="\033[31m"; C_N="\033[0m"
info()  { echo -e "${C_B}[*]${C_N} $*"; }
ok()    { echo -e "${C_G}[+]${C_N} $*"; }
warn()  { echo -e "${C_Y}[!]${C_N} $*"; }
fail()  { echo -e "${C_R}[-]${C_N} $*"; exit 1; }

if command -v sudo >/dev/null 2>&1; then SUDO="sudo"; else SUDO=""; fi

# Библиотеки, нужные Qt6 для запуска (модуль PySide6 ставится отдельно, через pip)
# Обязательные библиотеки Qt6 (без них интерфейс не запустится вовсе)
declare -A QT_CORE_PACKAGES=(
    [apt-get]="libgl1 libegl1 libxkbcommon0 libdbus-1-3 libfontconfig1 libglib2.0-0 python3-pip"
    [dnf]="mesa-libGL mesa-libEGL libxkbcommon dbus-libs fontconfig python3-pip"
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
    [apt-get]="libxkbcommon-x11-0 libxcb1 libxcb-cursor0 libxcb-xinerama0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 libxcb-shape0 libwayland-client0 libwayland-cursor0"
    [dnf]="libxkbcommon-x11 xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil"
    [pacman]="libxkbcommon-x11 xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil wayland"
    [zypper]="libxkbcommon-x11-0 xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil"
    [apk]="libxcb xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil"
    [xbps-install]="libxkbcommon-x11 libxcb xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms xcb-util-renderutil"
    [emerge]="x11-libs/libxkbcommon-x11 x11-libs/xcb-util-cursor x11-libs/xcb-util-wm x11-libs/xcb-util-image x11-libs/xcb-util-keysyms x11-libs/xcb-util-renderutil"
)

is_source_tree() { [ -f "$SCRIPT_DIR/run.py" ]; }

# --- установка недостающих системных пакетов ---------------------------------
install_packages() {
    local missing=()
    local qt_missing=0
    if command -v python3 >/dev/null 2>&1; then
        python3 -c "import PySide6" >/dev/null 2>&1 || qt_missing=1
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
    for c in apt-get dnf pacman zypper apk xbps-install emerge; do
        command -v "$c" >/dev/null 2>&1 && { pm="$c"; break; }
    done
    [ -z "$pm" ] && fail "Не удалось определить пакетный менеджер. Установите вручную: ${missing[*]}"

    if [ "$qt_missing" -eq 1 ] && [ -n "${QT_CORE_PACKAGES[$pm]:-}" ]; then
        # shellcheck disable=SC2206 — список пакетов разбиваем по пробелам
        missing+=(${QT_CORE_PACKAGES[$pm]})
        info "Добавляю библиотеки Qt6: ${QT_CORE_PACKAGES[$pm]}"
    fi

    local rc=0
    case "$pm" in
        apt-get)
            $SUDO apt-get update -y >/dev/null 2>&1 || true
            $SUDO apt-get install -y "${missing[@]}" || rc=1
            ;;
        dnf)
            local pkgs=()
            for p in "${missing[@]}"; do
                case "$p" in
                    python3) pkgs+=(python3) ;;
                    *)       pkgs+=("$p") ;;
                esac
            done
            $SUDO dnf install -y "${pkgs[@]}" || rc=1
            ;;
        pacman)
            $SUDO pacman -S --needed --noconfirm "${missing[@]}" || rc=1
            ;;
        zypper)
            $SUDO zypper --non-interactive install "${missing[@]}" || rc=1
            ;;
        apk)
            $SUDO apk add "${missing[@]}" || rc=1
            ;;
        xbps-install)
            $SUDO xbps-install -y "${missing[@]}" || rc=1
            ;;
        emerge)
            $SUDO emerge -av "${missing[@]}" || rc=1
            ;;
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
            apt-get)      $SUDO apt-get install -y ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            dnf)          $SUDO dnf install -y ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            pacman)       $SUDO pacman -S --needed --noconfirm ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            zypper)       $SUDO zypper --non-interactive install ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            apk)          $SUDO apk add ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            xbps-install) $SUDO xbps-install -y ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
            emerge)       $SUDO emerge -n ${QT_XCB_PACKAGES[$pm]} >/dev/null 2>&1 || true ;;
        esac
    fi
}

# --- установка/обновление кода приложения ------------------------------------
sync_local_tree() {
    # копируем исходники в APP_DIR, не трогая зависимости и кэш
    mkdir -p "$APP_DIR"
    tar -C "$SCRIPT_DIR" \
        --exclude=.git --exclude=deps --exclude=cache \
        --exclude=__pycache__ --exclude='*.pyc' \
        -cf - . | tar -C "$APP_DIR" -xf -
    ok "Код скопирован в $APP_DIR"
}

install_code() {
    if is_source_tree; then
        if [ -d "$SCRIPT_DIR/.git" ]; then
            info "Обновление исходников (git pull)…"
            git -C "$SCRIPT_DIR" pull --ff-only 2>/dev/null || warn "git pull не удался — использую текущую версию."
        fi
        sync_local_tree
    else
        if [ -d "$APP_DIR/.git" ]; then
            info "Обновление кода…"
            git -C "$APP_DIR" fetch --depth 1 origin "$BRANCH" 2>/dev/null \
                && git -C "$APP_DIR" reset --hard FETCH_HEAD \
                || warn "Обновление кода не удалось."
        else
            info "Загрузка приложения из $REPO_URL…"
            rm -rf "$APP_DIR"
            git clone --depth 1 --branch "$BRANCH" "$REPO_URL" "$APP_DIR" \
                || fail "Не удалось клонировать $REPO_URL"
            ok "Приложение загружено."
        fi
    fi
}

install_pyside() {
    if python3 -c "import PySide6" >/dev/null 2>&1; then
        ok "Qt6-интерфейс (PySide6) уже установлен."
        return 0
    fi
    info "Устанавливаю Qt6-интерфейс (PySide6-Essentials)…"
    local pip="python3 -m pip"
    if ! python3 -m pip --version >/dev/null 2>&1; then
        warn "pip недоступен — приложение поставит PySide6 само при первом запуске."
        return 1
    fi
    local ok_install=0
    for args in "--user" "--user --break-system-packages" "--break-system-packages"; do
        if $pip install $args --quiet --disable-pip-version-check PySide6-Essentials; then
            ok_install=1
            break
        fi
    done
    if [ "$ok_install" -eq 1 ] && python3 -c "import PySide6" >/dev/null 2>&1; then
        ok "PySide6 установлен."
        return 0
    fi
    warn "Не удалось поставить PySide6 автоматически. Интерфейс попробует сделать это при запуске."
    warn "Вручную: python3 -m pip install --user PySide6-Essentials"
    return 1
}

launch_gui() {
    cd "$APP_DIR"
    if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
        warn "Графический сервер не обнаружен. Запустите вручную:"
        echo "    python3 $APP_DIR/run.py gui"
        return 0
    fi
    # Проверяем, что Qt действительно может открыть окно (плагин xcb на месте)
    if ! python3 -c "from PySide6.QtWidgets import QApplication; QApplication([])" \
            >/tmp/zc-qt-check.log 2>&1; then
        warn "Qt не смог запуститься. Подробности:"
        sed 's/^/    /' /tmp/zc-qt-check.log | head -5
        warn "Выполните самодиагностику: python3 $APP_DIR/run.py doctor"
        return 1
    fi
    info "Запуск интерфейса…"
    nohup python3 run.py gui >/tmp/zapret-control.log 2>&1 &
    ok "Zapret Control запущен."
    info "Если окно не появилось, лог запуска: /tmp/zapret-control.log"
}

# --- основной сценарий --------------------------------------------------------
CMD="${1:-install}"

case "$CMD" in
    uninstall)
        warn "Удаление Zapret Control…"
        if [ -f "$APP_DIR/run.py" ]; then
            (cd "$APP_DIR" && python3 run.py service remove) || true
            (cd "$APP_DIR" && python3 run.py shortcut remove) || true
        fi
        rm -rf "$APP_DIR"
        ok "Приложение удалено."
        ;;
    gui|launch|run)
        [ -f "$APP_DIR/run.py" ] || install_code
        launch_gui
        ;;
    update)
        info "Обновление Zapret Control…"
        install_packages
        install_pyside || true
        install_code
        (cd "$APP_DIR" && python3 run.py ensure-deps) \
            || warn "Обновление зависимостей не удалось — проверьте соединение."
        (cd "$APP_DIR" && python3 run.py shortcut install)
        ok "Обновление завершено."
        ;;
    install|*)
        info "Установка Zapret Control…"
        install_packages
        install_pyside || true
        install_code
        info "Скачивание зависимостей (nfqws + стратегии)…"
        (cd "$APP_DIR" && python3 run.py ensure-deps) || warn "Скачивание зависимостей не удалось — сделайте это в приложении."
        (cd "$APP_DIR" && python3 run.py shortcut install)
        ok "Ярлык приложения создан."
        echo ""
        if [ -n "$SUDO" ] && sudo -n true 2>/dev/null; then
            info "Настройка прав sudo (NOPASSWD для nft/nfqws)…"
            (cd "$APP_DIR" && python3 run.py permissions install) || warn "Права не настроены."
        else
            warn "Права sudo не настроены — настройте позже из приложения (кнопка «Права sudo»)."
        fi
        launch_gui
        ;;
esac
