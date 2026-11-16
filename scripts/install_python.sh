#!/usr/bin/env bash
#
# Python 3.9+ на Ubuntu одной командой в терминале:
#
#   ./scripts/install_python.sh            # поставить/проверить python3 (3.9+)
#   ./scripts/install_python.sh --check    # только проверить, ничего не менять
#   ./scripts/install_python.sh 3.11       # конкретная версия (3.9 … 3.13)
#
# Что делает:
#   1) ищет уже установленный подходящий Python; если системный python3 старый,
#      делает найденную версию доступной как /usr/local/bin/python3;
#   2) на Ubuntu берёт python3.X из apt, а чего в репозиториях нет — из PPA
#      deadsnakes (на 20.04 в системе только 3.8, на 22.04 — 3.10);
#   3) если apt не помог (нет сети до PPA, не-Ubuntu, 32-битная ARM) — ставит
#      готовую сборку python-build-standalone в /usr/local/pythonX.Y и делает
#      python3 ссылкой на неё;
#   4) печатает версию и путь. Повторный запуск безопасен.
#
# Флаги:
#   --check        только проверка (код 0 — версия подходит, 1 — нет)
#   --apt          только apt/PPA, без готовых сборок
#   --from-source  сразу ставить готовую сборку, без apt
#   --min VERSION  минимальная версия (по умолчанию 3.9)
#   -h, --help     эта справка
#
set -euo pipefail
cd "$(dirname "$0")/.."

WANT=""                  # пусто = любая версия >= MIN_VERSION
MIN_VERSION="3.9"
MODE="install"           # install | check
ALLOW_APT=1
ALLOW_SOURCE=1
INSTALL_DIR="${RC_PYTHON_PREFIX:-/usr/local}"
OS_RELEASE="${RC_PYTHON_OS_RELEASE:-/etc/os-release}"
APT_DIR="${RC_PYTHON_APT_DIR:-/etc/apt}"
STANDALONE_REPO="${RC_PYTHON_STANDALONE_REPO:-astral-sh/python-build-standalone}"
STANDALONE_RELEASE="${RC_PYTHON_STANDALONE_RELEASE:-}"   # пусто = последний релиз
INSTALLED=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check)       MODE="check"; shift ;;
    --apt)         ALLOW_SOURCE=0; shift ;;
    --from-source) ALLOW_APT=0; shift ;;
    --min)         MIN_VERSION="${2:?--min требует версию, например --min 3.11}"; shift 2 ;;
    -h|--help)     awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
    [0-9]*)        WANT="$1"; shift ;;
    *) echo "неизвестный аргумент: $1 (см. --help)" >&2; exit 1 ;;
  esac
done

log()  { printf '\033[1;32m[python]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[python]\033[0m %s\n' "$*"; }
err()  { printf '\033[1;31m[python]\033[0m %s\n' "$*" >&2; }
have() { command -v "$1" >/dev/null 2>&1; }

SUDO=""
if [[ $EUID -ne 0 ]] && have sudo; then SUDO=sudo; fi
# --check ничего не меняет, поэтому права нужны только для установки: так его
# можно запускать и без sudo, и на машине, где sudo нет вовсе.
if [[ "$MODE" != "check" ]]; then
  [[ $EUID -eq 0 ]] || [[ -n "$SUDO" ]] || { err "нужен root или sudo."; exit 1; }
fi

# --- 1. что уже есть --------------------------------------------------------- #
# Версии сравниваем на чистом bash: скрипт ставит Python, поэтому не должен
# зависеть от того, что системный python3 рабочий.
split_mm() {  # split_mm 3.11.4 → MM_MAJOR=3 MM_MINOR=11
  [[ "$1" =~ ^([0-9]+)\.([0-9]+) ]] || return 1
  MM_MAJOR="${BASH_REMATCH[1]}"; MM_MINOR="${BASH_REMATCH[2]}"
  return 0
}

version_ok() {  # version_ok X.Y > 0, если X.Y >= MIN_VERSION (и == WANT, если задан)
  local version="$1" want="${2:-$WANT}" min="${3:-$MIN_VERSION}"
  local have_major have_minor need_major need_minor
  split_mm "$version" || return 1
  have_major="$MM_MAJOR"; have_minor="$MM_MINOR"
  split_mm "$min" || return 1
  need_major="$MM_MAJOR"; need_minor="$MM_MINOR"
  if (( have_major < need_major )) \
     || { (( have_major == need_major )) && (( have_minor < need_minor )); }; then
    return 1
  fi
  if [[ -n "$want" ]]; then
    split_mm "$want" || return 1
    (( have_major == MM_MAJOR && have_minor == MM_MINOR )) || return 1
  fi
  return 0
}

py_version() {  # печатает X.Y установленного интерпретатора
  "$1" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || true
}

find_python() {  # ищет интерпретатор нужной версии; печатает путь
  local candidate version numeric
  if [[ -n "$WANT" ]]; then
    candidate="python${WANT}"
    if have "$candidate" && version_ok "$(py_version "$candidate")" \
       && "$candidate" -c 'import venv' >/dev/null 2>&1; then
      command -v "$candidate"; return 0
    fi
    return 1
  fi
  if have python3; then
    version="$(py_version python3)"
    if [[ -n "$version" ]] && version_ok "$version" \
       && python3 -c 'import venv' >/dev/null 2>&1; then
      command -v python3; return 0
    fi
  fi
  # прочие python3.NN — от новых к старым; каталоги можно переопределить
  # (RC_PYTHON_SEARCH_DIRS) для нестандартных систем и тестов.
  local dirs
  dirs="${RC_PYTHON_SEARCH_DIRS:-/usr/local/bin /usr/bin}"
  # shellcheck disable=SC2086
  for numeric in $(ls $dirs/python3.[0-9]* 2>/dev/null \
                   | sed 's/.*python3\.\([0-9]*\)$/\1/' | sort -rn -u); do
    # shellcheck disable=SC2086
    for candidate in $dirs/python3.$numeric; do
      [[ -x "$candidate" ]] || continue
      version="$(py_version "$candidate")"
      [[ -n "$version" ]] || continue
      if version_ok "$version" && "$candidate" -c 'import venv' >/dev/null 2>&1; then
        echo "$candidate"; return 0
      fi
    done
  done
  return 1
}

report_ok() { log "готово: $1 — $("$1" --version 2>&1) (нужен ${WANT:-$MIN_VERSION}+)"; }

if FOUND="$(find_python)"; then
  if [[ "$MODE" == "check" ]]; then
    report_ok "$FOUND"
    exit 0
  fi
  # Частая ситуация на Ubuntu 20.04: python3 остаётся системным 3.8,
  # но python3.9/3.11 уже установлен. Недостаточно просто «найти» его:
  # установщик проекта затем вызывал python3 и снова получал старую версию.
  # Сделаем найденный подходящий интерпретатор доступным как python3 в
  # /usr/local/bin (не трогая /usr/bin/python3 и Python, нужный самой Ubuntu).
  if ! have python3 || ! version_ok "$(py_version python3)"; then
    $SUDO mkdir -p "$INSTALL_DIR/bin"
    $SUDO ln -sf "$FOUND" "$INSTALL_DIR/bin/python3"
    log "python3 направлен на подходящий интерпретатор: $INSTALL_DIR/bin/python3"
  fi
  report_ok "$FOUND"
  log "ставить Python не нужно. Для проекта дальше: ./scripts/install_ubuntu.sh"
  exit 0
fi

CURRENT="$(have python3 && python3 --version 2>&1 || echo 'python3 не установлен')"
if [[ "$MODE" == "check" ]]; then
  err "подходящего Python нет ($CURRENT), нужен ${WANT:-$MIN_VERSION}+."
  err "Исправить: ./scripts/install_python.sh"
  exit 1
fi
log "подходящего Python нет ($CURRENT) — ставлю ${WANT:-$MIN_VERSION}+…"

# --- 2. какая это система ---------------------------------------------------- #
read_os() {
  local key value
  [[ -r "$OS_RELEASE" ]] || return 0
  while IFS='=' read -r key value || [[ -n "$key" ]]; do
    value="${value%\"}"; value="${value#\"}"; value="${value%\'}"; value="${value#\'}"
    case "$key" in
      ID)               OS_ID="$value" ;;
      PRETTY_NAME)      OS_PRETTY="$value" ;;
      VERSION_ID)       OS_VERSION_ID="$value" ;;
      VERSION_CODENAME) OS_CODENAME="$value" ;;
      UBUNTU_CODENAME)  OS_UBUNTU_CODENAME="$value" ;;
    esac
  done < "$OS_RELEASE"
}
OS_ID=""; OS_PRETTY=""; OS_VERSION_ID=""; OS_CODENAME=""; OS_UBUNTU_CODENAME=""
read_os
# Linux Mint, Pop!_OS и т.п. называют себя иначе, но репозитории у них от Ubuntu.
OS_CODENAME="${OS_UBUNTU_CODENAME:-$OS_CODENAME}"

# Версия, которую ставим, если её не указали явно.
default_target() {
  case "$OS_CODENAME" in
    focal) echo "3.11" ;;   # 20.04: в системе 3.8, ставим 3.11 из deadsnakes
    jammy) echo "3.11" ;;   # 22.04: в системе 3.10, 3.11 из deadsnakes
    *)     echo "3.12" ;;   # 24.04 и новее: 3.12 обычно уже в системе
  esac
}
TARGET="${WANT:-$(default_target)}"

apt_has_target() {  # есть ли пакет python3.X в текущих репозиториях
  local version="$1"
  [[ -n "$version" ]] || return 1
  apt-cache show "python${version}" 2>/dev/null | grep -q '^Version:'
}

# --- 3. apt / PPA deadsnakes ------------------------------------------------- #
add_deadsnakes() {
  local version="$1"
  if apt_has_target "$version"; then return 0; fi   # уже есть в репозиториях
  if ! have add-apt-repository; then
    log "ставлю software-properties-common (нужен для PPA deadsnakes)…"
    $SUDO apt-get update -y || true
    $SUDO apt-get install -y software-properties-common ca-certificates || return 1
  fi
  log "подключаю PPA deadsnakes — там есть python${version} для этой системы…"
  # -n: не открывать интерактивный редактор. Ключ PPA подписан Launchpad, как и
  # остальные пакеты apt, поэтому отдельных подтверждений не требуется.
  if $SUDO add-apt-repository -y -n ppa:deadsnakes/ppa >/dev/null 2>&1; then
    $SUDO apt-get update -y || true
    return 0
  fi
  # add-apt-repository не сработал — прописываем репозиторий и ключ вручную.
  local list_dir="$APT_DIR/sources.list.d" keyring="$APT_DIR/trusted.gpg.d/deadsnakes.gpg" key=""
  local key_url="https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF23C5A6CF475977595C89F51BA6932366A755776"
  $SUDO mkdir -p "$list_dir"
  printf 'deb https://ppa.launchpadcontent.net/deadsnakes/ppa/ubuntu %s main\n' "$OS_CODENAME" \
    | $SUDO tee "$list_dir/deadsnakes-ppa.list" >/dev/null
  if have curl; then
    key="$(curl -fsSL "$key_url" 2>/dev/null || true)"
  elif have wget; then
    key="$(wget -qO- "$key_url" 2>/dev/null || true)"
  fi
  if [[ -n "$key" ]]; then
    printf '%s\n' "$key" | $SUDO gpg --dearmor --yes -o "$keyring" 2>/dev/null || true
  fi
  $SUDO apt-get update -y || return 1
}

install_from_apt() {
  local version="$1"
  have apt-get || return 1
  add_deadsnakes "$version" || return 1
  log "ставлю python${version}, venv и pip через apt…"
  $SUDO apt-get install -y "python${version}" "python${version}-venv" \
                           "python${version}-dev" python3-pip ca-certificates || return 1
  INSTALLED="$(command -v "python${version}" 2>/dev/null || true)"
  [[ -n "$INSTALLED" ]] || return 1
  # python3 → новая версия, чтобы её видели скрипты проекта и venv.
  $SUDO ln -sf "$INSTALLED" "/usr/local/bin/python3" 2>/dev/null || true
  return 0
}

# --- 4. готовая сборка CPython (без компиляции и без PPA) -------------------- #
# python-build-standalone: уже собранный CPython с pip и venv внутри, работает
# на любой Ubuntu/glibc-системе — запасной путь, когда apt не помог.
standalone_release() {
  if [[ -n "$STANDALONE_RELEASE" ]]; then
    printf '%s' "$STANDALONE_RELEASE"; return 0
  fi
  have curl || return 1
  curl -fsSL "https://api.github.com/repos/$STANDALONE_REPO/releases/latest" 2>/dev/null \
    | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p' | head -1
}

install_from_source() {
  local version="$1" arch release tar_name url tmp dir prefix
  case "$(uname -m)" in
    x86_64|amd64)  arch="x86_64" ;;
    aarch64|arm64) arch="aarch64" ;;
    *) err "готовой сборки Python для $(uname -m) нет — поставьте Python вручную."; return 1 ;;
  esac
  release="$(standalone_release || true)"
  [[ -n "$release" ]] || { err "не удалось узнать релиз python-build-standalone (нужен internet + curl)."; return 1; }
  # Точное имя файла берём из GitHub API: схема имён в релизах менялась.
  # install_only.tar.gz — полный вариант, install_only_stripped.tar.gz — без
  # отладочных символов (тоже подходит); stripped берём только как запасной.
  tar_name=""
  if [[ -z "$tar_name" ]] && have curl; then
    tar_name="$(curl -fsSL "https://api.github.com/repos/$STANDALONE_REPO/releases/tags/$release" 2>/dev/null \
      | grep -o '"name": *"cpython-'"$version"'[^"]*'"$arch"'-unknown-linux-gnu-install_only\.tar\.gz"' \
      | head -1 | sed 's/.*"name": *"//; s/"$//')"
    if [[ -z "$tar_name" ]]; then
      tar_name="$(curl -fsSL "https://api.github.com/repos/$STANDALONE_REPO/releases/tags/$release" 2>/dev/null \
        | grep -o '"name": *"cpython-'"$version"'[^"]*'"$arch"'-unknown-linux-gnu-install_only_stripped\.tar\.gz"' \
        | head -1 | sed 's/.*"name": *"//; s/"$//')"
    fi
  fi
  [[ -n "$tar_name" ]] || tar_name="cpython-${version}+${release}-${arch}-unknown-linux-gnu-install_only.tar.gz"
  url="https://github.com/$STANDALONE_REPO/releases/download/$release/$tar_name"

  log "ставлю Python ${version} готовой сборкой python-build-standalone (${release}, ${arch}): ${tar_name}"
  tmp="$(mktemp -d)"
  # shellcheck disable=SC2064
  trap "rm -rf '$tmp'" EXIT
  if have curl; then
    curl -fL --retry 3 -o "$tmp/python.tar.gz" "$url" || return 1
  elif have wget; then
    wget -O "$tmp/python.tar.gz" "$url" || return 1
  else
    err "нужен curl или wget, чтобы скачать Python."; return 1
  fi
  tar -xzf "$tmp/python.tar.gz" -C "$tmp" || return 1
  dir="$(find "$tmp" -maxdepth 2 -type d -name python 2>/dev/null | head -1)"
  [[ -n "$dir" && -x "$dir/bin/python3" ]] || { err "в архиве нет python3."; return 1; }

  log "распаковываю в ${INSTALL_DIR}/python${version}…"
  $SUDO mkdir -p "$INSTALL_DIR/bin"
  $SUDO rm -rf "${INSTALL_DIR:?}/python${version}"
  $SUDO cp -a "$dir" "${INSTALL_DIR}/python${version}"
  prefix="$INSTALL_DIR/python${version}/bin"
  $SUDO ln -sf "$prefix/python3" "$INSTALL_DIR/bin/python${version}"
  $SUDO ln -sf "$prefix/python3" "$INSTALL_DIR/bin/python3.${version}"
  if ! have python3 || ! version_ok "$(py_version "$(command -v python3)")"; then
    $SUDO ln -sf "$prefix/python3" "$INSTALL_DIR/bin/python3"
  fi
  INSTALLED="$prefix/python3"
  return 0
}

# --- 5. пробуем по порядку --------------------------------------------------- #
if [[ $ALLOW_APT -eq 1 ]]; then
  install_from_apt "$TARGET" || warn "apt/PPA не сработали — пробую готовую сборку."
fi
if [[ -z "$INSTALLED" && $ALLOW_SOURCE -eq 1 ]]; then
  install_from_source "$TARGET" || true
fi

if [[ -z "$INSTALLED" ]]; then
  err "не удалось поставить Python ${TARGET} автоматически."
  err "Вручную: sudo apt-get install -y python${TARGET} python${TARGET}-venv"
  err "или:     ./scripts/install_python.sh --from-source"
  exit 1
fi

if ! FOUND="$(find_python)"; then
  if [[ -x "$INSTALLED" ]] && version_ok "$(py_version "$INSTALLED")"; then
    FOUND="$INSTALLED"
  else
    err "Python поставлен ($INSTALLED), но не запускается. Проверьте вручную: $INSTALLED --version"
    exit 1
  fi
fi

report_ok "$FOUND"
log "дальше: ./scripts/install_ubuntu.sh — он создаст .venv и поднимет робота."
