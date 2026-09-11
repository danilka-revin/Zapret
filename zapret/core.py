"""
Ядро Zapret Control: скачивание зависимостей, парсинг стратегий,
настройка файрвола (nftables/iptables) и запуск nfqws.

Построено по мотивам zapret-discord-youtube-linux (Sergeydigl3),
zapret (bol-van) и стратегий Flowseal.
"""

import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import tarfile
import tempfile
import time
from pathlib import Path
from urllib import request

from . import (
    APP_NAME,
    GAME_FILTER_OFF_PORTS,
    GAME_FILTER_PORTS,
    NFT_CHAIN_POST,
    NFT_CHAIN_PRE,
    NFT_MARK,
    NFT_QUEUE_NUM,
    NFT_TABLE,
    STRATEGIES_DEFAULT_REV,
    STRATEGIES_REPO,
    ZAPRET_REPO,
    app_dir,
)
from . import config as config_mod

# ----------------------------------------------------------------------------
# Пути
# ----------------------------------------------------------------------------

def deps_dir() -> Path:
    return app_dir() / "deps"


def strategies_dir() -> Path:
    """Директория со стратегиями Flowseal (содержит *.bat, bin/, lists/)."""
    return deps_dir() / "flowseal"


def nfqws_path() -> Path:
    return app_dir() / "nfqws"


def extras_dir() -> Path:
    """Каталог с поставляемыми файлами (telegram.bat, списки Telegram).

    При установке они лежат в APP_DIR/extras, а при запуске прямо из клона —
    рядом с кодом. Возвращаем первый существующий вариант.
    """
    for p in (app_dir() / "extras",
              Path(__file__).resolve().parent.parent / "extras"):
        if (p / "telegram.bat").exists() or (p / "list-telegram.txt").exists():
            return p
    return app_dir() / "extras"


# ----------------------------------------------------------------------------
# Привилегии и выполнение
# ----------------------------------------------------------------------------

def is_root() -> bool:
    return os.geteuid() == 0


def elevation_prefix() -> list:
    """Префикс для повышения привилегий без интерактивного пароля.

    Порядок: root (без префикса) → sudo -n → doas -n.
    """
    if is_root():
        return []
    if which("sudo"):
        return ["sudo", "-n"]
    if which("doas"):
        return ["doas", "-n"]
    return []


class _RunResult:
    """Заглушка результата, когда утилита повышения привилегий отсутствует."""

    returncode = 127

    def __init__(self):
        self.stdout = ""
        self.stderr = ""


def run_privileged(args, input_text=None, check=True, cwd=None, capture=False):
    cmd = elevation_prefix() + list(args)
    try:
        return subprocess.run(
            cmd,
            input=input_text,
            text=(input_text is not None),
            check=check,
            cwd=cwd,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.PIPE if capture else None,
        )
    except FileNotFoundError as exc:
        if check:
            raise RuntimeError(
                f"Не найдена утилита повышения привилегий (sudo/doas) "
                f"или команда '{args[0]}'"
            ) from exc
        return _RunResult()


def which(prog) -> str | None:
    return shutil.which(prog)


# ----------------------------------------------------------------------------
# Платформа
# ----------------------------------------------------------------------------

def platform_dir() -> str:
    import platform as _p

    machine = _p.machine().lower()
    if _p.system() == "Linux":
        mapping = {
            "x86_64": "linux-x86_64", "amd64": "linux-x86_64",
            "i686": "linux-x86", "i386": "linux-x86",
            "aarch64": "linux-arm64", "arm64": "linux-arm64",
            "armv7l": "linux-arm", "armv6l": "linux-arm",
            "mips64": "linux-mips64", "mips64el": "linux-mips64el",
            "mipsel": "linux-mipsel", "mips": "linux-mips",
            "ppc64le": "linux-ppc", "ppc64": "linux-ppc", "ppc": "linux-ppc",
        }
        if machine in mapping:
            return mapping[machine]
        raise RuntimeError(f"Неподдерживаемая архитектура Linux: {machine}")
    if _p.system() == "FreeBSD":
        return "freebsd-x86_64"
    raise RuntimeError(f"Неподдерживаемая ОС: {_p.system()}")


# ----------------------------------------------------------------------------
# Скачивание
# ----------------------------------------------------------------------------

_USER_AGENT = "Mozilla/5.0 (X11; Linux x86_64) ZapretControl/" + "1.0"


def _http_get(url: str, timeout: int = 120) -> bytes:
    req = request.Request(url, headers={"User-Agent": _USER_AGENT})
    with request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


def _http_get_curl(url: str, timeout: int = 300) -> bytes:
    """Загрузка через curl: сам идёт по редиректам и переживает обрывы."""
    curl = shutil.which("curl")
    if not curl:
        raise RuntimeError("curl не найден")
    proc = subprocess.run(
        [curl, "-L", "-f", "-s", "-S", "--retry", "2", "--retry-delay", "1",
         "--connect-timeout", "20", "--max-time", str(timeout), url],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.decode("utf-8", "replace").strip()
                           or f"curl завершился с кодом {proc.returncode}")
    if not proc.stdout:
        raise RuntimeError("сервер вернул пустой ответ")
    return proc.stdout


def _download_to(url: str, dest: Path, progress_cb=None, attempts: int = 3) -> None:
    """Скачивает файл: сначала curl, затем встроенный urllib, с повторами."""
    if progress_cb:
        progress_cb(f"Скачивание {url}")
    try:
        dest.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise RuntimeError(f"Нет доступа к каталогу {dest.parent}: {exc}") from exc

    fetchers = []
    if shutil.which("curl"):
        fetchers.append(_http_get_curl)
    fetchers.append(_http_get)
    errors = []
    for attempt in range(1, attempts + 1):
        for fetcher in fetchers:
            try:
                data = fetcher(url)
            except Exception as exc:  # noqa: BLE001 — пробуем следующий способ
                errors.append(f"{fetcher.__name__}: {exc}")
                continue
            if data:
                dest.write_bytes(data)
                return
        if attempt < attempts:
            if progress_cb:
                progress_cb(f"Повторная попытка {attempt + 1}/{attempts}…")
            time.sleep(1.5 * attempt)
    raise RuntimeError("Не удалось скачать " + url + " — " + "; ".join(errors[-3:]))


def latest_release_tag(repo: str) -> str:
    url = f"https://api.github.com/repos/{repo}/releases/latest"
    data = json.loads(_http_get(url).decode("utf-8", "replace"))
    return data["tag_name"]


def ensure_nfqws(version: str, progress_cb=None) -> Path:
    """Скачивает бинарник nfqws из релизов bol-van/zapret."""
    dest = nfqws_path()
    tag = version if version and version != "latest" else latest_release_tag(ZAPRET_REPO)
    archive_name = f"zapret-{tag}.tar.gz"
    url = f"https://github.com/{ZAPRET_REPO}/releases/download/{tag}/{archive_name}"

    plat = platform_dir()
    if progress_cb:
        progress_cb(f"Загрузка nfqws {tag} ({plat})…")

    tmp = tempfile.mkdtemp(prefix="zapret-")
    try:
        archive = Path(tmp) / archive_name
        _download_to(url, archive, progress_cb)
        member_path = None
        with tarfile.open(archive, "r:gz") as tf:
            for member in tf.getmembers():
                name = member.name
                if member.isfile() and f"binaries/{plat}/nfqws" in name and name.endswith("nfqws"):
                    member_path = member
                    break
            if member_path is None:
                raise RuntimeError(
                    f"Бинарник nfqws не найден в архиве для платформы {plat}"
                )
            data = tf.extractfile(member_path).read()
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
        dest.chmod(0o755)
        if progress_cb:
            progress_cb(f"nfqws сохранён: {dest}")
        return dest
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# Транслитерация имён .bat-файлов (как в rename_bat.sh upstream)
_TRANSLIT = str.maketrans({
    "а": "a", "б": "b", "в": "v", "г": "g", "д": "d", "е": "e", "ё": "yo",
    "ж": "zh", "з": "z", "и": "i", "й": "y", "к": "k", "л": "l", "м": "m",
    "н": "n", "о": "o", "п": "p", "р": "r", "с": "s", "т": "t", "у": "u",
    "ф": "f", "х": "h", "ц": "ts", "ч": "ch", "ш": "sh", "щ": "sch", "ъ": "",
    "ы": "y", "ь": "", "э": "e", "ю": "yu", "я": "ya",
    "А": "A", "Б": "B", "В": "V", "Г": "G", "Д": "D", "Е": "E", "Ё": "Yo",
    "Ж": "Zh", "З": "Z", "И": "I", "Й": "Y", "К": "K", "Л": "L", "М": "M",
    "Н": "N", "О": "O", "П": "P", "Р": "R", "С": "S", "Т": "T", "У": "U",
    "Ф": "F", "Х": "H", "Ц": "Ts", "Ч": "Ch", "Ш": "Sh", "Щ": "Sch", "Ъ": "",
    "Ы": "Y", "Ь": "", "Э": "E", "Ю": "Yu", "Я": "Ya",
})


def _sanitize_bat_name(name: str) -> str:
    name = name.translate(_TRANSLIT).lower()
    name = re.sub(r"[\s()]+", "_", name)
    name = re.sub(r"__+", "_", name)
    name = re.sub(r"_+\.bat$", ".bat", name)
    return name


def _copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, dst)


def ensure_strategies(rev: str, progress_cb=None) -> Path:
    """Скачивает стратегии Flowseal и дополняет их файлами Telegram."""
    sd = strategies_dir()
    rev = rev or STRATEGIES_DEFAULT_REV
    url = f"https://codeload.github.com/{STRATEGIES_REPO}/tar.gz/{rev}"

    tmp = tempfile.mkdtemp(prefix="zapret-strat-")
    try:
        archive = Path(tmp) / "flowseal.tar.gz"
        _download_to(url, archive, progress_cb)
        extract_to = Path(tmp) / "extract"
        extract_to.mkdir(parents=True, exist_ok=True)
        with tarfile.open(archive, "r:gz") as tf:
            tf.extractall(extract_to)
        # Корень архива — одна подпапка вида zapret-discord-youtube-<rev>/
        roots = [p for p in extract_to.iterdir() if p.is_dir()]
        if not roots:
            raise RuntimeError("Архив стратегий пуст")
        src_root = roots[0]

        if progress_cb:
            progress_cb("Установка стратегий…")

        # Замена существующего каталога стратегий
        if sd.exists():
            shutil.rmtree(sd, ignore_errors=True)
        deps_dir().mkdir(parents=True, exist_ok=True)
        shutil.copytree(src_root, sd)

        # Переименование .bat (транслитерация русских имён)
        for p in sd.glob("*.bat"):
            new_name = _sanitize_bat_name(p.name)
            if new_name != p.name and not (p.parent / new_name).exists():
                p.rename(p.parent / new_name)

        _provision_lists(progress_cb)
        return sd
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _provision_lists(progress_cb=None) -> None:
    """Добавляет списки Telegram и пользовательские списки в каталог стратегий."""
    sd = strategies_dir()
    lists = sd / "lists"
    lists.mkdir(parents=True, exist_ok=True)

    # Пользовательские списки (на них ссылаются стратегии general*.bat)
    for name in ("list-general-user.txt", "list-exclude-user.txt", "ipset-exclude-user.txt"):
        p = lists / name
        if not p.exists():
            p.write_text("", encoding="utf-8")

    # Файлы Telegram из extras
    ex = extras_dir()
    for name in ("list-telegram.txt", "ipset-telegram.txt"):
        src = ex / name
        if src.exists():
            _copy_file(src, lists / name)
        elif not (lists / name).exists():
            # fallback — генерируем из пакета
            from . import telegram as tg
            content = (
                tg.telegram_domains_text() if name.startswith("list-")
                else tg.telegram_ips_text()
            )
            (lists / name).write_text(content, encoding="utf-8")

    # Стратегия "только Telegram"
    telegram_bat = ex / "telegram.bat"
    if telegram_bat.exists():
        _copy_file(telegram_bat, sd / "telegram.bat")
    elif not (sd / "telegram.bat").exists():
        from . import telegram as tg
        (sd / "telegram.bat").write_text(tg.TELEGRAM_STRATEGY_BAT, encoding="utf-8")


def ensure_deps(version: str = "latest", rev: str = "", progress_cb=None) -> None:
    ensure_nfqws(version, progress_cb)
    ensure_strategies(rev, progress_cb)


def deps_ready() -> bool:
    return nfqws_path().exists() and strategies_dir().exists()


# ----------------------------------------------------------------------------
# Стратегии
# ----------------------------------------------------------------------------

def list_strategies() -> list:
    """Имена доступных стратегий (general*/discord* + telegram)."""
    sd = strategies_dir()
    if not sd.exists():
        return []
    out = []
    for p in sd.glob("*.bat"):
        base = p.name
        if base.startswith("general") or base.startswith("discord") or base == "telegram.bat":
            out.append(base)
    out = sorted(set(out))
    if "general.bat" in out:
        out.remove("general.bat")
        out.insert(0, "general.bat")
    return out


def resolve_strategy(name: str) -> Path | None:
    sd = strategies_dir()
    candidates = [name]
    if not name.endswith(".bat"):
        candidates += [f"{name}.bat", f"general_{name}.bat"]
    for cand in candidates:
        p = sd / cand
        if p.exists():
            return p
    # регистронезависимый поиск
    wanted = {c.lower() for c in candidates}
    for p in sd.glob("*.bat"):
        if p.name.lower() in wanted:
            return p
    return None


def merge_ports(a: str, b: str) -> str:
    parts = set()
    for chunk in (a, b):
        for tok in str(chunk).split(","):
            tok = tok.strip()
            if tok:
                parts.add(tok)

    def sort_key(tok: str):
        head = tok.split("-")[0]
        try:
            return (0, int(head))
        except ValueError:
            return (1, head)

    return ",".join(sorted(parts, key=sort_key))


def _clean_block(block: str) -> str:
    return re.sub(r"\s+", " ", block.strip())


def parse_strategy_text(text: str, gamefilter_tcp: bool, gamefilter_udp: bool):
    """Возвращает (tcp_ports, udp_ports, [блоки nfqws])."""
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    # Windows-перенос строки: символ ^ в конце строки склеивает строки
    text = re.sub(r"\^[ \t]*\n", " ", text)
    text = text.replace("%BIN%", "bin/").replace("%LISTS%", "lists/")

    if gamefilter_tcp:
        text = text.replace("%GameFilterTCP%", GAME_FILTER_PORTS)
    else:
        text = text.replace(",%GameFilterTCP%", "")
        text = text.replace("%GameFilterTCP%,", "")
        text = text.replace("%GameFilterTCP%", GAME_FILTER_OFF_PORTS)

    if gamefilter_udp:
        text = text.replace("%GameFilterUDP%", GAME_FILTER_PORTS)
    else:
        text = text.replace(",%GameFilterUDP%", "")
        text = text.replace("%GameFilterUDP%,", "")
        text = text.replace("%GameFilterUDP%", GAME_FILTER_OFF_PORTS)

    tcp = udp = ""
    m = re.search(r"--wf-tcp=([0-9,\-]+)", text)
    if m:
        tcp = m.group(1)
    m = re.search(r"--wf-udp=([0-9,\-]+)", text)
    if m:
        udp = m.group(1)

    blocks = []
    for m in re.finditer(r"--filter-(?:tcp|udp)=", text):
        start = m.start()
        nxt = text.find("--new", start)
        if nxt == -1:
            block = text[start:]
        else:
            block = text[start:nxt + len("--new")]
        block = _clean_block(block)
        if block:
            blocks.append(block)
    return tcp, udp, blocks


def parse_strategy(path, gamefilter_tcp: bool, gamefilter_udp: bool):
    path = Path(path)
    return parse_strategy_text(path.read_text(encoding="utf-8", errors="replace"),
                               gamefilter_tcp, gamefilter_udp)


def build_effective_blocks(cfg: dict):
    """Итоговые блоки nfqws с учётом базовой стратегии и Telegram."""
    base = resolve_strategy(cfg.get("strategy", "general.bat"))
    if base is None:
        raise RuntimeError(
            f"Стратегия '{cfg.get('strategy')}' не найдена. Сначала скачайте зависимости."
        )
    tcp, udp, blocks = parse_strategy(base, cfg.get("gamefilter_tcp", False),
                                      cfg.get("gamefilter_udp", False))

    if cfg.get("telegram", True):
        from . import telegram as tg
        t2, u2, b2 = parse_strategy_text(
            tg.TELEGRAM_STRATEGY_BAT,
            cfg.get("gamefilter_tcp", False),
            cfg.get("gamefilter_udp", False),
        )
        tcp = merge_ports(tcp, t2)
        udp = merge_ports(udp, u2)
        blocks.extend(b2)

    return tcp, udp, blocks


# ----------------------------------------------------------------------------
# Файрвол
# ----------------------------------------------------------------------------

def detect_backend(pref: str = "auto") -> str:
    if pref == "nftables":
        return "nftables"
    if pref == "iptables":
        return "iptables"
    if which("nft"):
        return "nftables"
    if which("iptables"):
        return "iptables"
    raise RuntimeError("Не найден nftables или iptables. Установите один из них.")


def _nft_ruleset(tcp_ports: str, udp_ports: str, interface: str) -> str:
    oif = f'oifname "{interface}"' if interface and interface != "any" else ""
    iif = f'iifname "{interface}"' if interface and interface != "any" else ""

    rules_pre = []
    rules_post = []

    if tcp_ports:
        rules_post.append(
            f"{oif} meta mark and {NFT_MARK} == 0 tcp dport {{{tcp_ports}}} "
            f"ct original packets 1-6 queue num {NFT_QUEUE_NUM} bypass comment \"zapret\""
        )
        rules_pre.append(
            f"{iif} tcp sport {{{tcp_ports}}} "
            f"ct reply packets 1-3 queue num {NFT_QUEUE_NUM} bypass comment \"zapret\""
        )
    if udp_ports:
        rules_post.append(
            f"{oif} meta mark and {NFT_MARK} == 0 udp dport {{{udp_ports}}} "
            f"ct original packets 1-6 queue num {NFT_QUEUE_NUM} bypass comment \"zapret\""
        )

    def fmt_rules(rules):
        return "\n    ".join(rules) if rules else ""

    return f"""table {NFT_TABLE} {{
  chain {NFT_CHAIN_PRE} {{
    type filter hook prerouting priority filter; policy accept;
    {fmt_rules(rules_pre)}
  }}
  chain {NFT_CHAIN_POST} {{
    type filter hook postrouting priority mangle; policy accept;
    {fmt_rules(rules_post)}
  }}
}}
"""


def firewall_setup(tcp_ports: str, udp_ports: str, interface: str, backend: str = "auto") -> None:
    backend = detect_backend(backend)
    if backend == "nftables":
        # сбрасываем старую таблицу, если есть
        run_privileged(["nft", "delete", "table", NFT_TABLE], check=False)
        ruleset = _nft_ruleset(tcp_ports, udp_ports, interface)
        run_privileged(["nft", "-f", "-"], input_text=ruleset)
    else:
        _iptables_setup(tcp_ports, udp_ports, interface)


def _iptables_setup(tcp_ports: str, udp_ports: str, interface: str) -> None:
    chain = "zapret"
    reply = "reply"
    table = "mangle"

    def conv(ports: str) -> str:
        return ports.replace("{", "").replace("}", "").replace("-", ":")

    ipt_tcp = conv(tcp_ports)
    ipt_udp = conv(udp_ports)
    oif = ["-o", interface] if interface and interface != "any" else []

    for cmd in ("iptables", "ip6tables"):
        run_privileged([cmd, "-t", table, "-D", "POSTROUTING", "-j", chain], check=False)
        run_privileged([cmd, "-t", table, "-F", chain], check=False)
        run_privileged([cmd, "-t", table, "-X", chain], check=False)
        run_privileged([cmd, "-t", table, "-D", "PREROUTING", "-j", reply], check=False)
        run_privileged([cmd, "-t", table, "-F", reply], check=False)
        run_privileged([cmd, "-t", table, "-X", reply], check=False)

        run_privileged([cmd, "-t", table, "-N", chain])
        run_privileged([cmd, "-t", table, "-A", "POSTROUTING", "-j", chain])
        if ipt_tcp:
            run_privileged([cmd, "-t", table, "-A", chain] + oif +
                           ["-p", "tcp", "-m", "multiport", "--dports", ipt_tcp,
                            "-m", "connbytes", "--connbytes-dir=original",
                            "--connbytes-mode=packets", "--connbytes", "1:6",
                            "-m", "mark", "!", "--mark", NFT_MARK,
                            "-j", "NFQUEUE", "--queue-num", str(NFT_QUEUE_NUM),
                            "--queue-bypass"])
        if ipt_udp:
            run_privileged([cmd, "-t", table, "-A", chain] + oif +
                           ["-p", "udp", "-m", "multiport", "--dports", ipt_udp,
                            "-m", "connbytes", "--connbytes-dir=original",
                            "--connbytes-mode=packets", "--connbytes", "1:6",
                            "-m", "mark", "!", "--mark", NFT_MARK,
                            "-j", "NFQUEUE", "--queue-num", str(NFT_QUEUE_NUM),
                            "--queue-bypass"])
        if ipt_tcp:
            run_privileged([cmd, "-t", table, "-N", reply])
            run_privileged([cmd, "-t", table, "-A", "PREROUTING", "-j", reply])
            run_privileged([cmd, "-t", table, "-A", reply] + oif +
                           ["-p", "tcp", "-m", "multiport", "--sports", ipt_tcp,
                            "-m", "connbytes", "--connbytes-dir=reply",
                            "--connbytes-mode=packets", "--connbytes", "1:3",
                            "-m", "mark", "!", "--mark", NFT_MARK,
                            "-j", "NFQUEUE", "--queue-num", str(NFT_QUEUE_NUM),
                            "--queue-bypass"])


def firewall_clear() -> None:
    if which("nft"):
        run_privileged(["nft", "delete", "table", NFT_TABLE], check=False)
    for cmd in ("iptables", "ip6tables"):
        if which(cmd):
            run_privileged([cmd, "-t", "mangle", "-D", "POSTROUTING", "-j", "zapret"], check=False)
            run_privileged([cmd, "-t", "mangle", "-F", "zapret"], check=False)
            run_privileged([cmd, "-t", "mangle", "-X", "zapret"], check=False)
            run_privileged([cmd, "-t", "mangle", "-D", "PREROUTING", "-j", "reply"], check=False)
            run_privileged([cmd, "-t", "mangle", "-F", "reply"], check=False)
            run_privileged([cmd, "-t", "mangle", "-X", "reply"], check=False)


# ----------------------------------------------------------------------------
# nfqws
# ----------------------------------------------------------------------------

def stop_nfqws() -> None:
    """Останавливает только сам процесс nfqws (по имени, а не по строке запуска)."""
    run_privileged(["pkill", "-x", "nfqws"], check=False)


def start_nfqws(blocks: list) -> None:
    if not nfqws_path().exists():
        raise RuntimeError("nfqws не найден. Скачайте зависимости.")
    stop_nfqws()
    flat = []
    for b in blocks:
        flat.extend(shlex.split(b))
    cmd = [str(nfqws_path()), "--daemon",
           f"--dpi-desync-fwmark={NFT_MARK}", f"--qnum={NFT_QUEUE_NUM}"] + flat
    run_privileged(cmd, cwd=str(strategies_dir()))


def nfqws_running() -> bool:
    """Проверяет, запущен ли nfqws.

    Важно: сравниваем именно имя процесса. Поиск по командной строке (`pgrep -f`)
    ловил бы посторонние процессы, в аргументах которых встречается «nfqws».
    """
    try:
        r = subprocess.run(["pgrep", "-x", "nfqws"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return r.returncode == 0
    except OSError:
        pass
    proc = Path("/proc")
    if not proc.exists():
        return False
    try:
        for entry in proc.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                name = (entry / "comm").read_text(encoding="utf-8", errors="replace").strip()
            except OSError:
                continue
            if name == "nfqws":
                return True
    except OSError:
        return False
    return False


def firewall_active() -> bool:
    if not which("nft"):
        return False
    for args in ([*elevation_prefix(), "nft", "list", "table", NFT_TABLE],
                 ["nft", "list", "table", NFT_TABLE]):
        try:
            r = subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return r.returncode == 0
        except OSError:
            continue
    return False


# ----------------------------------------------------------------------------
# Запуск / остановка
# ----------------------------------------------------------------------------

def run_zapret(cfg: dict = None) -> None:
    cfg = cfg or config_mod.load()
    if not deps_ready():
        raise RuntimeError("Зависимости не скачаны. Нажмите «Скачать зависимости».")
    tcp, udp, blocks = build_effective_blocks(cfg)

    stop_nfqws()
    firewall_clear()
    time.sleep(0.5)

    backend = detect_backend(cfg.get("firewall_backend", "auto"))
    firewall_setup(tcp, udp, cfg.get("interface", "any"), backend)
    start_nfqws(blocks)


def stop_zapret() -> None:
    stop_nfqws()
    firewall_clear()


def status() -> dict:
    from .integration import (
        permissions_ready,
        service_active,
        shortcut_installed,
    )
    return {
        "running": nfqws_running(),
        "firewall": firewall_active(),
        "deps_ready": deps_ready(),
        "service_installed": service_installed(),
        "service_active": service_active(),
        "shortcut_installed": shortcut_installed(),
        "sudo_ok": permissions_ready(),
        "backend": detect_backend("auto") if (which("nft") or which("iptables")) else None,
        "app_dir": str(app_dir()),
    }


def service_installed() -> bool:
    from .integration import SERVICE_FILE
    return Path(SERVICE_FILE).exists()


# ----------------------------------------------------------------------------
# Демон (для systemd)
# ----------------------------------------------------------------------------

def daemon() -> None:
    cfg = config_mod.load()
    try:
        run_zapret(cfg)
    except Exception as exc:  # noqa: BLE001
        print(f"[{APP_NAME}] Ошибка запуска: {exc}", flush=True)
        raise SystemExit(1) from exc
    print(f"[{APP_NAME}] zapret запущен в режиме демона", flush=True)

    stop_flag = {}

    def _handler(signum, frame):
        stop_flag["stop"] = True

    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)

    try:
        while not stop_flag.get("stop"):
            time.sleep(1)
    finally:
        print(f"[{APP_NAME}] Остановка…", flush=True)
        stop_zapret()
