"""Настоящий журнал событий робота — то, что показывается на бортовом экране.

Раньше панель «Логи» на экране киоска рисовала выдуманный поток строк
(«SLAM: скан лидара 360° — 412 точек», «маршрут: склад → зона выгрузки») и
дорисовывала его случайными сообщениями каждые полторы секунды. Это и была
главная ложь экрана: демо-строки выглядели как живые события робота.

Теперь журнал один и он настоящий: кольцевой буфер в памяти процесса, куда
попадает всё, что **реально произошло** в этом процессе:

* записи логирования ``robot_control.*`` — подключение и потеря лидара,
  выбор источника данных, ошибки портов, открытие/переоткрытие экрана
  в браузере (их ловит :class:`EventLogHandler`);
* явные события маршрутов — проверка PIN, команды руления и тяги, нажатия
  «Задать 0°», недоступность управления (их пишет :func:`record`).

Журнал честен и в другую сторону: если событий не было — так и написано
(``entries`` пустой), а не подставлены демо-строки ради красоты. Демо-потока
в системе нет.

Ничего секретного в журнал не попадает: значения PIN/паролей/токенов
маскируются (:func:`sanitize`), строки обрезаются, переводы строк и
управляющие символы вычищаются — одна запись всегда одна строка.
"""

from __future__ import annotations

import logging
import os
import re
import threading
import time
import uuid
from collections import deque
from typing import Any, Iterable, Mapping

__all__ = [
    "DEFAULT_LIMIT",
    "MAX_LIMIT",
    "ROOT_LOGGER",
    "EventLog",
    "EventLogHandler",
    "GROUPS",
    "event_group",
    "clear",
    "get_event_log",
    "install",
    "payload",
    "record",
    "reset",
    "sanitize",
]

#: Сколько записей держим в памяти (``RC_KIOSK_LOG_LIMIT``).
DEFAULT_LIMIT = 400
#: Границы буфера: меньше 20 — экран пустой, больше 5000 — копим мусор.
MIN_LIMIT = 20
MAX_LIMIT = 5000
#: Столько записей отдаём экрану за один запрос по умолчанию.
DEFAULT_QUERY_LIMIT = 120
#: Длина сообщения в буфере: длиннее — обрезаем с многоточием.
MAX_MESSAGE = 300
#: Повтор того же сообщения внутри окна — не новая строка, а счётчик «×N».
COLLAPSE_WINDOW_S = 5.0
#: Дерево логирования робота: и родитель, и все дочерние журналы.
ROOT_LOGGER = "robot_control"

_LEVELS = ("debug", "info", "ok", "warn", "err")

# logging-уровни → уровни экрана. ``ok`` ставится только явными событиями.
_LOGGING_LEVELS = {
    logging.DEBUG: "debug",
    logging.INFO: "info",
    logging.WARNING: "warn",
    logging.ERROR: "err",
    logging.CRITICAL: "err",
}

#: ``pin=1234``, ``password: hunter2`` … — значения в журнал не попадают.
_SECRET_RE = re.compile(
    r"(?i)\b(pin|пароль|password|passwd|secret|token|apikey|api_key)\b"
    r"(\s*[:=]\s*)(\S+)"
)
#: ``PIN 1234`` — код без знака равенства маскируется только если это цифры:
#: иначе пострадала бы обычная фраза «PIN принят».
_SECRET_WORD_RE = re.compile(
    r"(?i)\b(pin|пароль|password|passwd|secret|token)\b(\s+)(\d{4,8})\b"
)
_WHITESPACE_RE = re.compile(r"\s+")
_CONTROL_RE = re.compile(r"[\x00-\x08\x0b-\x1f\x7f]")


def sanitize(message: Any) -> str:
    """Одна чистая строка без секретов, переводов строк и управляющих символов."""
    text = _CONTROL_RE.sub("", str(message))
    text = _WHITESPACE_RE.sub(" ", text).strip()
    text = _SECRET_RE.sub(lambda m: f"{m.group(1)}{m.group(2)}***", text)
    text = _SECRET_WORD_RE.sub(lambda m: f"{m.group(1)}{m.group(2)}***", text)
    if len(text) > MAX_MESSAGE:
        text = text[: MAX_MESSAGE - 1].rstrip() + "…"
    return text


def normalize_level(level: Any) -> str:
    """Уровень записи: только ``debug/info/ok/warn/err`` (иначе ``info``)."""
    if isinstance(level, int):
        return _LOGGING_LEVELS.get(level, "info")
    name = str(level or "").strip().lower()
    if name in {"warn", "warning"}:
        return "warn"
    if name in {"err", "error", "critical", "fatal", "exception"}:
        return "err"
    if name in {"ok", "success"}:
        return "ok"
    if name == "debug":
        return "debug"
    return "info"


def _env_limit(default: int = DEFAULT_LIMIT) -> int:
    raw = (os.environ.get("RC_KIOSK_LOG_LIMIT") or "").strip()
    if not raw:
        return default
    try:
        value = int(raw)
    except ValueError:
        return default
    return max(MIN_LIMIT, min(MAX_LIMIT, value))


def clamp_limit(value: Any, *, default: int = DEFAULT_QUERY_LIMIT,
                maximum: int = MAX_LIMIT) -> int:
    """Сколько записей отдать экрану: число из запроса в разумных границах."""
    match = re.search(r"\d+", str(value if value is not None else ""))
    if not match:
        return max(1, min(maximum, default))
    return max(1, min(maximum, int(match.group())))


GROUPS = {
    "wheels": "Колёса и тяга", "battery": "АКБ и питание", "lidar": "Лидар",
    "cameras": "Камеры", "navigation": "Карта и навигация", "system": "Система",
}


def event_group(source: str, message: str = "") -> str:
    """Stable subsystem IDs; retain the original source for diagnostics."""
    name = source.lower().removeprefix("robot_control.").split(".")[0]
    for group, prefixes in (
        ("lidar", ("lidar", "ros_lidar", "лидар")),
        ("wheels", ("wheel", "master", "мастер", "колёс", "колес", "руление", "тяга", "управление")),
        ("battery", ("battery", "bms", "акб", "питание")),
        ("cameras", ("camera", "камер")),
        ("navigation", ("navigation", "slam_map", "карта", "навигац")),
    ):
        if name.startswith(prefixes):
            return group
    if message.lower().startswith(("лидар:", "лидар на связи", "скан лидара")):
        return "lidar"
    return "system"


# Only routine lidar counters are variable summaries. Never generalize errors,
# angles, port numbers or other numeric diagnostics.
_POINT_COUNT = re.compile(r"(?i)(?:\d+\s*(?:точ(?:ек|ки|ка)|points?)\b|(?:точек|points?)\s*[:=]\s*\d+)")


def _summary_key(level: str, text: str, source: str) -> str | None:
    if level not in {"info", "ok", "debug"} or event_group(source, text) != "lidar":
        return None
    if not re.search(r"(?i)скан|считан|прочитан|получен|scan|read|received", text):
        return None
    return _POINT_COUNT.sub("<points>", text) if _POINT_COUNT.search(text) else None


class EventLog:
    """Кольцевой буфер настоящих событий процесса (потокобезопасный)."""

    #: Сколько последних записей просматриваем в поиске повтора.
    COLLAPSE_SCAN = 12

    def __init__(self, limit: int | None = None, *, clock=time.time,
                 collapse_window: float = COLLAPSE_WINDOW_S) -> None:
        self.limit = max(MIN_LIMIT, min(MAX_LIMIT, int(limit or _env_limit())))
        self.collapse_window = collapse_window
        self.clock = clock
        self._entries: deque[dict[str, Any]] = deque(maxlen=self.limit)
        self._lock = threading.Lock()
        self._seq = 0
        self._journal_id = uuid.uuid4().hex
        self.dropped = 0

    # ------------------------------------------------------------- запись --- #
    def record(self, level: Any, message: Any, *, source: str = "") -> dict[str, Any] | None:
        """Добавить настоящее событие. Пустое сообщение игнорируется."""
        text = sanitize(message)
        if not text:
            return None
        lvl = normalize_level(level)
        src = sanitize(source) if source else ""
        now = self.clock()
        with self._lock:
            repeated = self._find_repeat(lvl, text, src, now)
            if repeated is not None:
                # Тот же повтор внутри окна (лидар не отвечает, тягу держат…):
                # не плодим строки, а показываем счётчик «×N».
                repeated["msg"] = text
                repeated["repeat"] += 1
                repeated["ts"] = now
                repeated["time"] = self._clock_label(now)
                self._entries.remove(repeated)
                self._entries.append(repeated)   # обновлённая запись — в конец
                return repeated
            self._seq += 1
            entry = {
                "seq": self._seq,
                "ts": now,
                "firstTs": now,
                "time": self._clock_label(now),
                "level": lvl,
                "source": src,
                "group": event_group(src, text),
                "msg": text,
                "repeat": 1,
            }
            if len(self._entries) == self.limit:
                self.dropped += 1
            self._entries.append(entry)
            return entry

    def extend(self, events: Iterable[Mapping[str, Any]]) -> int:
        """Записать пачку событий (например, разобранный журнал из файла)."""
        added = 0
        for event in events:
            if self.record(event.get("level", "info"), event.get("msg", ""),
                           source=event.get("source", "")):
                added += 1
        return added

    # ------------------------------------------------------------- чтение --- #
    def entries(self, limit: int | None = None, *, oldest_first: bool = True) -> list[dict[str, Any]]:
        """Записи журнала: по умолчанию от старых к новым, как читают лог."""
        with self._lock:
            items = [dict(item) for item in self._entries]
        if limit is not None and limit > 0:
            items = items[-int(limit):]
        if not oldest_first:
            items = items[::-1]
        return [dict(item) for item in items]

    def totals(self) -> dict[str, int]:
        """Сколько записей каждого уровня (для подписи под журналом)."""
        totals = {level: 0 for level in _LEVELS}
        with self._lock:
            for entry in self._entries:
                totals[entry["level"]] = totals.get(entry["level"], 0) + 1
        return totals

    def clear(self) -> int:
        """Очистить буфер (только в памяти процесса). Возвращает число записей."""
        with self._lock:
            count = len(self._entries)
            self._entries.clear()
            self.dropped = 0
            self._seq = 0
            self._journal_id = uuid.uuid4().hex
        return count

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._entries)

    @property
    def last_seq(self) -> int:
        with self._lock:
            return self._seq

    # -------------------------------------------------------------- выдача --- #
    def payload(self, limit: int | None = None, *, uptime_sec: int | None = None,
                source: str | None = None) -> dict[str, Any]:
        """Ответ API для экрана: записи + честные счётчики.

        ``demo`` всегда ``False``: этот журнал не выдуман (демо-режима нет).
        """
        # One coherent snapshot, including counts while logging continues.
        with self._lock:
            items = [dict(entry) for entry in self._entries]
            dropped, journal_id = self.dropped, self._journal_id
        shown = items[-limit:] if limit and limit > 0 else items
        levels = {level: 0 for level in _LEVELS}
        groups = {key: {"label": label, "count": 0, "events": 0, "errors": 0, "warnings": 0}
                  for key, label in GROUPS.items()}
        for entry in items:
            levels[entry["level"]] += 1
            group = groups[entry["group"]]
            group["count"] += 1
            group["events"] += entry["repeat"]
            group["errors"] += int(entry["level"] == "err")
            group["warnings"] += int(entry["level"] == "warn")
        return {
            "entries": shown,
            "count": len(items),
            "shown": len(shown),
            "dropped": dropped,
            "levels": levels,
            "groups": groups,
            "journalId": journal_id,
            "limit": self.limit,
            "uptimeSec": uptime_sec,
            "source": source,
            "demo": False,
            "journal": "robot",
        }

    # ------------------------------------------------------------ служебное -- #
    def _find_repeat(self, level: str, text: str, source: str,
                     now: float) -> dict[str, Any] | None:
        """Последняя такая же запись внутри окна повторов (или None).

        Смотрим не только на конец журнала: между повторами успевают прошлые
        настоящие события (лидар написал строку — тяга продолжает удерживаться),
        и без этого одна и та же ситуация распадалась бы на несколько записей.
        """
        summary = _summary_key(level, text, source)
        # Scan the bounded buffer for routine summaries even if many other
        # subsystems logged in between; keep the last real message and time.
        deadline = now - (max(30.0, self.collapse_window) if summary else self.collapse_window)
        candidates = list(self._entries) if summary else list(self._entries)[-self.COLLAPSE_SCAN:]
        for item in reversed(candidates):
            if item["ts"] < deadline:
                return None
            if (item["level"] == level and item["source"] == source
                    and (item["msg"] == text or (summary is not None
                         and _summary_key(level, item["msg"], source) == summary))):
                return item
        return None

    @staticmethod
    def _clock_label(ts: float) -> str:
        return time.strftime("%H:%M:%S", time.localtime(ts))


class EventLogHandler(logging.Handler):
    """Пускает настоящие записи ``robot_control.*`` в журнал экрана.

    Формат не нужен: сообщение берётся через ``getMessage()`` (подстановки
    ``%s`` уже раскрыты), а имя журнала — короткое, без ``robot_control.``.
    Свои же сообщения модуля журнала не берутся: иначе запись о записи
    порождала бы саму себя.
    """

    def __init__(self, target: EventLog, level: int = logging.INFO,
                 ignore: tuple[str, ...] = ("robot_control.eventlog",)) -> None:
        super().__init__(level=level)
        self.target = target
        self.ignore = tuple(ignore)

    def emit(self, record: logging.LogRecord) -> None:  # noqa: D102 - базовый класс
        try:
            if record.name.startswith(self.ignore):
                return
            source = record.name
            if source.startswith(ROOT_LOGGER + "."):
                source = source[len(ROOT_LOGGER) + 1:]
            elif source == ROOT_LOGGER:
                source = ""
            self.target.record(record.levelno, record.getMessage(), source=source)
        except Exception:  # noqa: BLE001 — журнал не имеет права ломать работу робота
            self.handleError(record)


# --------------------------------------------------------------- синглтон --- #
_log: EventLog | None = None
_handlers: dict[str, EventLogHandler] = {}
_lock = threading.Lock()


def get_event_log() -> EventLog:
    """Журнал процесса: один на всё приложение (пульт и киоск — свои процессы)."""
    global _log
    with _lock:
        if _log is None:
            _log = EventLog()
        return _log


def install(logger_name: str = ROOT_LOGGER, *, level: int = logging.INFO,
            target: EventLog | None = None) -> EventLogHandler:
    """Подключить журнал к логированию (повторный вызов ничего не дублирует)."""
    journal = target or get_event_log()
    with _lock:
        existing = _handlers.get(logger_name)
        logger = logging.getLogger(logger_name)
        if existing is not None and existing in logger.handlers and existing.target is journal:
            return existing
        handler = EventLogHandler(journal, level=level)
        logger.addHandler(handler)
        _handlers[logger_name] = handler
        return handler


def reset() -> None:
    """Отключить журнал (тесты и повторная инициализация приложения)."""
    global _log
    with _lock:
        for name, handler in list(_handlers.items()):
            logging.getLogger(name).removeHandler(handler)
        _handlers.clear()
        _log = None


def record(level: Any, message: Any, *, source: str = "") -> dict[str, Any] | None:
    """Записать настоящее событие в журнал процесса."""
    return get_event_log().record(level, message, source=source)


def clear() -> int:
    """Очистить журнал процесса."""
    return get_event_log().clear()


def payload(limit: int | None = None, *, uptime_sec: int | None = None,
            source: str | None = None) -> dict[str, Any]:
    """Ответ API для экрана из журнала процесса."""
    return get_event_log().payload(limit, uptime_sec=uptime_sec, source=source)
