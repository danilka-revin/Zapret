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

is_source_tree() { [ -f "$SCRIPT_DIR/run.py" ]; }

# --- установка недостающих системных пакетов ---------------------------------
install_packages() {
    local missing=()
    if command -v python3 >/dev/null 2>&1; then
        python3 -c "import tkinter" >/dev/null 2>&1 || missing+=(python3-tk)
    else
        missing+=(python3 python3-tk)
    fi
    command -v git  >/dev/null 2>&1 || missing+=(git)
    command -v curl >/dev/null 2>&1 || missing+=(curl)
    if ! command -v nft >/dev/null 2>&1 && ! command -v iptables >/dev/null 2>&1; then
        missing+=(nftables)
    fi
    [ ${#missing[@]} -eq 0 ] && return 0

    info "Требуются пакеты: ${missing[*]}"
    local pm=""
    for c in apt-get dnf pacman zypper apk xbps-install emerge; do
        command -v "$c" >/dev/null 2>&1 && { pm="$c"; break; }
    done
    [ -z "$pm" ] && fail "Не удалось определить пакетный менеджер. Установите вручную: ${missing[*]}"

    local rc=0
    case "$pm" in
        apt-get)
            # имена пакетов совпадают (python3-tk, nftables)
            $SUDO apt-get update -y >/dev/null 2>&1 || true
            $SUDO apt-get install -y "${missing[@]}" || rc=1
            ;;
        dnf)
            local pkgs=()
            for p in "${missing[@]}"; do
                case "$p" in
                    python3-tk) pkgs+=(python3-tkinter) ;;
                    python3)    pkgs+=(python3) ;;
                    *)          pkgs+=("$p") ;;
                esac
            done
            $SUDO dnf install -y "${pkgs[@]}" || rc=1
            ;;
        pacman)
            local pkgs=()
            for p in "${missing[@]}"; do
                case "$p" in python3-tk) pkgs+=(tk) ;; *) pkgs+=("$p") ;; esac
            done
            $SUDO pacman -S --needed --noconfirm "${pkgs[@]}" || rc=1
            ;;
        zypper)
            $SUDO zypper --non-interactive install "${missing[@]}" || rc=1
            ;;
        apk)
            local pkgs=()
            for p in "${missing[@]}"; do
                case "$p" in python3-tk) pkgs+=(py3-tkinter) ;; *) pkgs+=("$p") ;; esac
            done
            $SUDO apk add "${pkgs[@]}" || rc=1
            ;;
        xbps-install)
            local pkgs=()
            for p in "${missing[@]}"; do
                case "$p" in python3-tk) pkgs+=(python3-tkinter) ;; *) pkgs+=("$p") ;; esac
            done
            $SUDO xbps-install -y "${pkgs[@]}" || rc=1
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

launch_gui() {
    cd "$APP_DIR"
    if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
        info "Запуск интерфейса…"
        nohup python3 run.py gui >/dev/null 2>&1 &
        ok "Zapret Control запущен."
    else
        warn "Графический сервер не обнаружен. Запустите вручную:"
        echo "    python3 $APP_DIR/run.py gui"
    fi
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
        install_code
        (cd "$APP_DIR" && python3 run.py ensure-deps) \
            || warn "Обновление зависимостей не удалось — проверьте соединение."
        (cd "$APP_DIR" && python3 run.py shortcut install)
        ok "Обновление завершено."
        ;;
    install|*)
        info "Установка Zapret Control…"
        install_packages
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
