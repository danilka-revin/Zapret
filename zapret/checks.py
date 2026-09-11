"""
Проверки для главного экрана: доступность сервисов, задержки и трафик.

Здесь нет ничего, что требует прав root: обычные HTTP-запросы и счётчики
сетевых интерфейсов из /proc. Используется и автопилотом, и интерфейсом.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from urllib import error as urlerror
from urllib import request as urlrequest

CURL = shutil.which("curl")
USER_AGENT = "Mozilla/5.0 (X11; Linux x86_64) ZapretControl/1.0"

# Порог «медленно»: выше него сервис считаем замедленным
SLOW_MS = 900.0


@dataclass(frozen=True)
class Service:
    key: str
    title: str
    icon: str
    endpoints: tuple[str, ...]
    hint: str


SERVICES: tuple[Service, ...] = (
    Service(
        "youtube", "YouTube", "play",
        (
            "https://www.youtube.com/generate_204",
            "https://i.ytimg.com/generate_204",
            "https://www.youtube.com/",
        ),
        "видео и превью",
    ),
    Service(
        "discord", "Discord", "gamepad",
        (
            "https://discord.com/api/v9/gateway",
            "https://cdn.discordapp.com/",
            "https://discord.com/",
        ),
        "чат и голос",
    ),
    Service(
        "telegram", "Telegram", "send",
        (
            "https://web.telegram.org/",
            "https://telegram.org/",
            "https://t.me/",
        ),
        "MTProto и веб",
    ),
)

SERVICES_BY_KEY = {s.key: s for s in SERVICES}

# ---------------------------------------------------------------------------
# Готовые группы сайтов для подбора стратегии
# ---------------------------------------------------------------------------

SITE_GROUPS: list[tuple[str, str, str, tuple[str, ...]]] = [
    ("youtube", "YouTube", "play",
     ("youtube.com", "youtu.be", "www.youtube.com", "i.ytimg.com", "googlevideo.com")),
    ("discord", "Discord", "gamepad",
     ("discord.com", "discordapp.com", "cdn.discordapp.com", "gateway.discord.gg")),
    ("telegram", "Telegram", "send",
     ("web.telegram.org", "telegram.org", "t.me", "cdn-telegram.org")),
    ("ai", "Нейросети", "sparkles",
     ("chatgpt.com", "claude.ai", "gemini.google.com", "perplexity.ai")),
    ("social", "Соцсети", "globe",
     ("instagram.com", "x.com", "www.facebook.com", "reddit.com")),
    ("media", "Медиа", "play",
     ("spotify.com", "twitch.tv", "soundcloud.com", "netflix.com")),
]

SITE_GROUPS_BY_KEY = {key: (title, icon, hosts) for key, title, icon, hosts in SITE_GROUPS}


def clean_host(value: str) -> str:
    """Превращает любой ввод («https://site.ru/page?x=1», «site.ru») в домен."""
    text = (value or "").strip().lower()
    if not text:
        return ""
    text = re.sub(r"^[a-z][a-z0-9+.-]*://", "", text)   # схема
    text = text.split("/")[0].split("?")[0].split("#")[0]
    text = text.split("@")[-1]                          # логин:пароль@
    text = text.split(":")[0]                           # порт
    text = text.strip(".").strip()
    if not text or " " in text:
        return ""
    # Простейшая проверка на правдоподобность домена: есть точка или localhost
    if "." not in text and text != "localhost":
        return ""
    if not re.match(r"^[a-z0-9._-]+$", text):
        return ""
    return text


def parse_targets(value: str) -> list[str]:
    """Разбирает строку с сайтами: пробелы, запятые, группы (youtube, ai…)."""
    if not value:
        return []
    result: list[str] = []
    for token in re.split(r"[\s,;]+", value.strip()):
        if not token:
            continue
        if token in SITE_GROUPS_BY_KEY:
            result.extend(SITE_GROUPS_BY_KEY[token][2])
            continue
        host = clean_host(token)
        if host and host not in result:
            result.append(host)
    return result


# Порог «медленно» для конкретного сайта: выше — считаем, что обход не помог
TARGET_SLOW_MS = 1200.0
TARGET_TLS_SLOW_MS = 700.0

# Коды, означающие «сервер ответил» (401/403 — авторизация, а не блокировка)
REACHABLE_CODES = {200, 204, 206, 301, 302, 303, 307, 308, 401, 403, 405, 429}


# ---------------------------------------------------------------------------
# Низкоуровневые проверки
# ---------------------------------------------------------------------------

def _probe_curl(url: str, timeout: float) -> tuple[int, float]:
    cmd = [
        CURL, "-s", "-o", "/dev/null", "--max-time", f"{timeout:.1f}",
        "-A", USER_AGENT, "-w", "%{http_code} %{time_total}", url,
    ]
    started = time.monotonic()
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                              text=True, timeout=timeout + 2)
    except (OSError, subprocess.TimeoutExpired):
        return 0, (time.monotonic() - started) * 1000
    parts = (proc.stdout or "").split()
    if len(parts) < 2:
        return 0, (time.monotonic() - started) * 1000
    try:
        code = int(parts[0])
        seconds = float(parts[1])
    except ValueError:
        return 0, (time.monotonic() - started) * 1000
    if seconds <= 0:
        seconds = time.monotonic() - started
    return code, seconds * 1000


def _probe_urllib(url: str, timeout: float) -> tuple[int, float]:
    started = time.monotonic()
    req = urlrequest.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urlrequest.urlopen(req, timeout=timeout) as resp:
            return int(getattr(resp, "status", 200)), (time.monotonic() - started) * 1000
    except urlerror.HTTPError as exc:
        return int(exc.code), (time.monotonic() - started) * 1000
    except Exception:  # noqa: BLE001 — таймауты, DNS, сброс соединения
        return 0, (time.monotonic() - started) * 1000


def probe_url(url: str, timeout: float = 6.0) -> tuple[int, float]:
    if CURL:
        return _probe_curl(url, timeout)
    return _probe_urllib(url, timeout)


def probe_service(service: Service, timeout: float = 6.0,
                  endpoints: int = 2) -> dict:
    """Опроса сервиса по нескольким адресам — берём лучший результат."""
    best_code, best_ms, worst = 0, float("inf"), 0.0
    for url in service.endpoints[:endpoints]:
        code, ms = probe_url(url, timeout)
        worst = max(worst, ms)
        if code in REACHABLE_CODES:
            if ms < best_ms:
                best_code, best_ms = code, ms
            if ms < 400:
                break
    if best_code:
        state = "warn" if best_ms > SLOW_MS else "ok"
        latency = best_ms
    else:
        state = "bad"
        latency = worst if worst != float("inf") else float(timeout * 1000)

    if state == "ok":
        detail = f"{latency:.0f} мс · работает"
    elif state == "warn":
        detail = f"{latency:.0f} мс · медленно, обход не помогает"
    else:
        detail = "нет ответа · похоже на блокировку"

    return {
        "key": service.key,
        "title": service.title,
        "icon": service.icon,
        "state": state,
        "latency_ms": round(latency),
        "code": best_code,
        "detail": detail,
        "hint": service.hint,
    }


def probe_all(timeout: float = 6.0, keys: list[str] | None = None) -> list[dict]:
    wanted = SERVICES if not keys else tuple(SERVICES_BY_KEY[k] for k in keys
                                             if k in SERVICES_BY_KEY)
    return [probe_service(s, timeout) for s in wanted]


def probe_host(host: str, timeout: float = 6.0, attempts: int = 2) -> dict:
    """Проверяет конкретный сайт: отвечает ли и насколько быстро.

    Меряет полное время ответа и время TLS-рукопожатия: у замедленных DPI сайтов
    рукопожатие «залипает» задолго до загрузки страницы.
    """
    started = time.monotonic()
    best: dict | None = None
    for scheme in ("https", "http"):
        url = f"{scheme}://{host}/"
        for _ in range(max(1, attempts)):
            code, total_ms, connect_ms = _probe_full(url, timeout)
            if code and (best is None or total_ms < best["latency_ms"]):
                best = {"host": host, "url": url, "code": code,
                        "latency_ms": round(total_ms), "tls_ms": round(connect_ms)}
            if best is not None and best["latency_ms"] < 400:
                break
        if best is not None:
            break

    if best is None:
        return {"host": host, "url": f"https://{host}/", "state": "bad", "code": 0,
                "latency_ms": round((time.monotonic() - started) * 1000), "tls_ms": 0,
                "title": host, "detail": "нет ответа · заблокирован или недоступен"}

    slow = (best["latency_ms"] > TARGET_SLOW_MS
            or (best["tls_ms"] and best["tls_ms"] > TARGET_TLS_SLOW_MS))
    state = "warn" if slow else "ok"
    if state == "ok":
        detail = f"{best['latency_ms']} мс · открывается"
    else:
        reason = "медленное TLS-рукопожатие" if (best["tls_ms"] or 0) > TARGET_TLS_SLOW_MS             else "долгий ответ"
        detail = f"{best['latency_ms']} мс · {reason}"
    best.update({"state": state, "detail": detail, "title": host})
    return best


def _probe_full(url: str, timeout: float) -> tuple[int, float, float]:
    """Возвращает (http-код, общее время мс, время TLS/подключения мс)."""
    if CURL:
        cmd = [CURL, "-s", "-o", "/dev/null", "--max-time", f"{timeout:.1f}",
               "-A", USER_AGENT, "-w", "%{http_code} %{time_total} %{time_appconnect}",
               url]
        started = time.monotonic()
        try:
            proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                  text=True, timeout=timeout + 2)
        except (OSError, subprocess.TimeoutExpired):
            return 0, (time.monotonic() - started) * 1000, 0.0
        parts = (proc.stdout or "").split()
        if len(parts) < 3:
            return 0, (time.monotonic() - started) * 1000, 0.0
        try:
            return int(parts[0]), max(1.0, float(parts[1]) * 1000), float(parts[2]) * 1000
        except ValueError:
            return 0, (time.monotonic() - started) * 1000, 0.0

    code, ms = _probe_urllib(url, timeout)
    return code, ms, 0.0


def probe_hosts(hosts: list[str], timeout: float = 6.0) -> list[dict]:
    return [probe_host(host, timeout) for host in hosts]


def internet_available(timeout: float = 4.0) -> bool:
    """Проверяет, есть ли сеть вообще (не путать с блокировкой сервиса)."""
    for url in ("https://1.1.1.1/", "https://dns.google/resolve?name=example.com"):
        code, _ms = probe_url(url, timeout)
        if code:
            return True
    return False


def score(results: list[dict]) -> tuple[int, float]:
    """Оценка качества обхода: (сколько сервисов работает, средняя задержка)."""
    ok = [r for r in results if r["state"] == "ok"]
    if not results:
        return 0, 0.0
    latencies = [r["latency_ms"] for r in results if r["state"] != "bad"]
    avg = sum(latencies) / len(latencies) if latencies else SLOW_MS
    return len(ok), avg


# ---------------------------------------------------------------------------
# Трафик
# ---------------------------------------------------------------------------

def read_iface_counters() -> dict[str, tuple[int, int]]:
    """Счётчики (rx, tx) в байтах по каждому интерфейсу из /proc/net/dev."""
    out: dict[str, tuple[int, int]] = {}
    path = Path("/proc/net/dev")
    if not path.exists():
        return out
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()[2:]
    except OSError:
        return out
    for line in lines:
        if ":" not in line:
            continue
        name, rest = line.split(":", 1)
        parts = rest.split()
        if len(parts) < 9:
            continue
        try:
            out[name.strip()] = (int(parts[0]), int(parts[8]))
        except ValueError:
            continue
    return out


def list_interfaces() -> list[str]:
    net = Path("/sys/class/net")
    if not net.exists():
        return []
    try:
        return sorted(p.name for p in net.iterdir() if p.is_dir())
    except OSError:
        return []


def default_route_iface() -> str | None:
    """Интерфейс маршрута по умолчанию — нужен для подсчёта трафика."""
    if shutil.which("ip"):
        try:
            proc = subprocess.run(["ip", "route", "get", "1.1.1.1"],
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                  text=True, timeout=3)
            parts = (proc.stdout or "").split()
            if "dev" in parts:
                return parts[parts.index("dev") + 1]
        except (OSError, subprocess.TimeoutExpired):
            pass
    counters = read_iface_counters()
    for name in ("wlan0", "wlp2s0", "wlp3s0", "enp0s3", "eth0", "eno1"):
        if name in counters:
            return name
    return next(iter(counters), None)


class TrafficMonitor:
    """Считает скорость по дельте счётчиков; хранит скользящий максимум."""

    def __init__(self, iface: str | None = None):
        self.iface = iface
        self._last: tuple[float, int, int] | None = None
        self.peak_rx = 0.0
        self.peak_tx = 0.0
        self.total_rx = 0
        self.total_tx = 0

    def sample(self) -> dict:
        counters = read_iface_counters()
        iface = self.iface if self.iface in counters else None
        if iface is None:
            iface = default_route_iface()
            if iface is None or iface not in counters:
                iface = next(iter(counters), None)
            self.iface = iface
        if iface is None:
            return {"iface": None, "rx_mbps": 0.0, "tx_mbps": 0.0,
                    "peak_rx": self.peak_rx, "peak_tx": self.peak_tx,
                    "total_rx": self.total_rx, "total_tx": self.total_tx}
        rx, tx = counters[iface]
        now = time.monotonic()
        rx_mbps = tx_mbps = 0.0
        if self._last is not None:
            prev_t, prev_rx, prev_tx = self._last
            dt = max(0.2, now - prev_t)
            drx = rx - prev_rx
            dtx = tx - prev_tx
            # защита от перезапуска счётчиков
            if drx < 0:
                drx = 0
            if dtx < 0:
                dtx = 0
            self.total_rx += drx
            self.total_tx += dtx
            rx_mbps = drx * 8 / dt / 1_000_000
            tx_mbps = dtx * 8 / dt / 1_000_000
        self._last = (now, rx, tx)
        self.peak_rx = max(self.peak_rx, rx_mbps)
        self.peak_tx = max(self.peak_tx, tx_mbps)
        return {
            "iface": iface,
            "rx_mbps": rx_mbps,
            "tx_mbps": tx_mbps,
            "peak_rx": self.peak_rx,
            "peak_tx": self.peak_tx,
            "total_rx": self.total_rx,
            "total_tx": self.total_tx,
        }
