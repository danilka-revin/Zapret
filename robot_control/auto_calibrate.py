"""Автоматическая калибровка при запуске по ярлыку.

Ярлык «Запустить робот» поднимает бортовой киоск (``kiosk_app``), который
владеет USB-линками четырёх колёс. Как только все четыре платы дали свежие
кадры, киоск сам один раз за жизнь процесса:

1. калибрует руль по оптодатчику — команда ``k`` у всех четырёх колёс
   (поиск флажка в пределах ±125°, прошивка ``CALIB_TIMEOUT_MS`` = 120 с);
2. включает режим потенциометра (``pm 1``) и калибрует потенциометры —
   команда ``q`` (ноль 0°, −60°, +60°, наклон сохраняется в EEPROM).

Порядок жёсткий: потенциометру нужен ноль руля (``zero_set``), который даёт
только калибровка по датчику, поэтому сначала колёса, потом потенциометры —
и только при свежих кадрах от всех четырёх модулей.

Калибровка руля физически вращает колесо, поэтому:

* запускается только на бортовом процессе с настоящим источником (master),
  никогда в тестах (``config.testing``) и не для источников без калибровки;
* взведённый E-STOP отменяет всю последовательность (и до старта, и в ходе
  ожиданий) — ни одна команда движения не отправляется;
* тяга/движение отклоняются самими командами бэкенда (``module_busy``):
  автокалибровка повторяет попытку и сдаётся с записью в журнал;
* включена по умолчанию — именно это и просили для ярлыка. Отключить:
  ``RC_AUTO_CALIBRATE=0`` (или ``false``/``no``/``off``) в окружении.

Один запуск ярлыка — одна последовательность. При следующем старте киоска
она выполнится снова.
"""
from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any, Callable

from . import eventlog

log = logging.getLogger(__name__)

MODULE_IDS = ("FL", "FR", "RL", "RR")

POLL_SEC = 0.5
#: Пока модули не подключились, напоминаем о себе не чаще раза в минуту.
WAIT_REMIND_SEC = 60.0
#: Старт калибровки: пока мастер разнесёт команду по четырём платам.
WHEEL_START_TIMEOUT_SEC = 20.0
#: Прошивка: CALIB_TIMEOUT_MS = 120 с; запас на связь и смену сторон.
WHEEL_DONE_TIMEOUT_SEC = 160.0
POT_START_TIMEOUT_SEC = 10.0
POT_MODE_TIMEOUT_SEC = 15.0
#: Прошивка: POT_CAL_TIMEOUT_MS = 30 с; запас двойной.
POT_DONE_TIMEOUT_SEC = 60.0
QUEUE_ATTEMPTS = 4
QUEUE_RETRY_SEC = 5.0

_DISABLED_VALUES = {"0", "false", "no", "off"}
_SOURCE_TAG = "автокалибровка"


def autostart_enabled(env: Any = None) -> bool:
    """Включена ли автокалибровка по умолчанию (``RC_AUTO_CALIBRATE``)."""
    env = os.environ if env is None else env
    return (env.get("RC_AUTO_CALIBRATE") or "1").strip().lower() not in _DISABLED_VALUES


def _fresh_frames(source, now: float) -> dict[str, dict]:
    """Свежие кадры известных модулей: ``{module: frame}``.

    Источник не читается силой: если кадров нет или они протухли — модуля
    просто нет в ответе. Неожиданные ошибки источника трактуем как «ещё не
    подключено», а не как повод отправлять команды.
    """
    try:
        stale = float(getattr(source, "STALE_S", 1.0))
        lock = getattr(source, "lock", None)
        if lock is not None:
            with lock:
                frames = dict(getattr(source, "frames", None) or {})
        else:
            frames = dict(getattr(source, "frames", None) or {})
    except Exception:  # noqa: BLE001 — «не читается» = «не подключено»
        return {}
    fresh: dict[str, dict] = {}
    for mid in MODULE_IDS:
        entry = frames.get(mid)
        if not entry:
            continue
        try:
            frame, ts = entry
            age = now - float(ts)
        except (TypeError, ValueError):
            continue
        if age < stale and isinstance(frame, dict):
            fresh[mid] = frame
    return fresh


def _all_fresh(source, now: float) -> dict[str, dict] | None:
    fresh = _fresh_frames(source, now)
    return fresh if len(fresh) == len(MODULE_IDS) else None


def _wheels_started(source, now: float) -> bool:
    return any(f.get("calib") or f.get("moving") for f in _fresh_frames(source, now).values())


def _wheels_done(source, now: float) -> bool:
    fresh = _all_fresh(source, now)
    return fresh is not None and all(not f.get("calib") and not f.get("moving")
                                     for f in fresh.values())


def _wheels_homed(source, now: float) -> list[str]:
    """Список модулей, у которых ноль НЕ задан (пусто — всё хорошо)."""
    fresh = _all_fresh(source, now)
    if fresh is None:
        return list(MODULE_IDS)
    return sorted(mid for mid, frame in fresh.items() if not frame.get("cal"))


def _pot_done(source, now: float) -> bool:
    fresh = _all_fresh(source, now)
    return fresh is not None and all(not f.get("calib") and not f.get("moving")
                                     for f in fresh.values())


def _pot_ready(source, now: float) -> list[str]:
    """Список модулей, у которых потенциометр НЕ подтверждён после q."""
    fresh = _all_fresh(source, now)
    if fresh is None:
        return list(MODULE_IDS)
    return sorted(mid for mid, frame in fresh.items() if not frame.get("pot"))


def _queue_with_retry(call: Callable[[], dict], latched, note, sleep, label: str) -> dict:
    reason = "unknown"
    for attempt in range(1, QUEUE_ATTEMPTS + 1):
        if latched() is not None:
            return {"ok": False, "reason": "estop"}
        try:
            result = call()
        except Exception as exc:  # noqa: BLE001 — источник не должен ронять поток
            log.exception("автокалибровка: %s — исключение при постановке в очередь", label)
            result = {"ok": False, "reason": str(exc)}
        if not isinstance(result, dict):
            result = {"ok": False, "reason": "bad_reply"}
        if result.get("ok"):
            return result
        reason = str(result.get("reason") or "unknown")
        if reason == "estop":
            return result
        if attempt < QUEUE_ATTEMPTS:
            note("warn", "%s не принята (%s), повтор %d/%d через %d с" %
                 (label, reason, attempt + 1, QUEUE_ATTEMPTS, int(QUEUE_RETRY_SEC)))
            sleep(QUEUE_RETRY_SEC)
    return {"ok": False, "reason": reason}


def _wait_for(predicate: Callable[[], bool], *, timeout: float, sleep, monotonic,
              latched) -> str:
    """``ok`` — условие выполнено; ``timeout`` — вышло время; ``estop`` — взведён E-STOP."""
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if latched() is not None:
            return "estop"
        if predicate():
            return "ok"
        sleep(POLL_SEC)
    return "timeout"


def run(source, latched, *, sleep=time.sleep, monotonic=time.monotonic,
        now=time.time, record=eventlog.record) -> dict:
    """Выполнить всю последовательность. Возвращает итог для журнала/тестов."""

    def note(level: str, message: str) -> None:
        try:
            record(level, message, source=_SOURCE_TAG)
        except Exception:  # noqa: BLE001 — журнал не должен ронять последовательность
            log.exception("автокалибровка: не удалось записать в журнал: %s", message)
        if level == "err":
            log.error("автокалибровка: %s", message)
        elif level == "warn":
            log.warning("автокалибровка: %s", message)
        else:
            log.info("автокалибровка: %s", message)

    # --- 0. ждём подключения всех четырёх ---------------------------------- #
    if latched() is not None:
        note("warn", "E-STOP взведён до старта — автокалибровка отменена")
        return {"ok": False, "reason": "estop", "stage": "wait"}
    note("info", "жду подключения всех четырёх колёс…")
    last_reminder = monotonic()
    while _all_fresh(source, now()) is None:
        if latched() is not None:
            note("warn", "E-STOP взведён в ожидании модулей — автокалибровка отменена")
            return {"ok": False, "reason": "estop", "stage": "wait"}
        if monotonic() - last_reminder >= WAIT_REMIND_SEC:
            last_reminder = monotonic()
            missing = sorted(set(MODULE_IDS) - set(_fresh_frames(source, now())))
            note("info", "всё ещё жду колёса, нет кадров: %s" % (", ".join(missing) or "все"))
        sleep(POLL_SEC)
    note("ok", "все четыре модуля подключены — калибрую руль (оптодатчик)")

    # --- 1. калибровка руля по оптодатчику --------------------------------- #
    result = _queue_with_retry(lambda: source.calibrate_module("ALL"), latched, note,
                               sleep, "калибровка руля")
    if not result.get("ok"):
        note("err", "калибровка руля не запущена: %s" % (result.get("reason") or "unknown"))
        return {"ok": False, "reason": result.get("reason"), "stage": "wheel"}
    # Сначала обязательно дождаться старта: иначе свежее «cal=…, calib=0»
    # выглядит как мгновенное завершение и мы бы ушли на потенциометры
    # прямо во время разворота колёс.
    state = _wait_for(lambda: _wheels_started(source, now()),
                      timeout=WHEEL_START_TIMEOUT_SEC,
                      sleep=sleep, monotonic=monotonic, latched=latched)
    if state == "estop":
        note("warn", "E-STOP взведён до старта калибровки руля — последовательность прервана")
        return {"ok": False, "reason": "estop", "stage": "wheel"}
    if state != "ok":
        note("err", "калибровка руля не стартовала: платы не отреагировали на k (связь?) — потенциометры пропускаю")
        return {"ok": False, "reason": "not_started", "stage": "wheel"}
    state = _wait_for(lambda: _wheels_done(source, now()), timeout=WHEEL_DONE_TIMEOUT_SEC,
                      sleep=sleep, monotonic=monotonic, latched=latched)
    if state == "estop":
        note("warn", "E-STOP взведён во время калибровки руля — последовательность прервана")
        return {"ok": False, "reason": "estop", "stage": "wheel"}
    if state != "ok":
        note("err", "калибровка руля не завершилась за %d с — потенциометры пропускаю" %
             int(WHEEL_DONE_TIMEOUT_SEC))
        return {"ok": False, "reason": "timeout", "stage": "wheel"}
    missing = _wheels_homed(source, now())
    if missing:
        note("err", "калибровка руля завершилась, но ноль не задан у: %s — потенциометры пропускаю"
             % ", ".join(missing))
        return {"ok": False, "reason": "not_homed", "stage": "wheel", "modules": missing}
    note("ok", "калибровка руля завершена: ноль по датчику у всех четырёх")

    # --- 2. режим потенциометра --------------------------------------------- #
    result = _queue_with_retry(lambda: source.set_pot_mode("on"), latched, note,
                               sleep, "включение потенциометра")
    if not result.get("ok"):
        note("err", "режим потенциометра не включён: %s" % (result.get("reason") or "unknown"))
        return {"ok": False, "reason": result.get("reason"), "stage": "pot_mode"}
    def _pot_mode_confirmed() -> bool:
        fresh = _all_fresh(source, now())
        return fresh is not None and all(f.get("potMode") is True for f in fresh.values())

    confirmed = _wait_for(_pot_mode_confirmed,
                          timeout=POT_MODE_TIMEOUT_SEC, sleep=sleep, monotonic=monotonic, latched=latched)
    if confirmed == "estop":
        note("warn", "E-STOP взведён перед калибровкой потенциометров — последовательность прервана")
        return {"ok": False, "reason": "estop", "stage": "pot_mode"}
    if confirmed != "ok":
        # pm 1 ставится в очередь всем четырём разом; молчание платы видно
        # по кадрам ниже — q всё равно попробуем, отказ вернёт причина.
        note("warn", "платы не подтвердили pm 1 в кадрах — продолжаю с калибровкой потенциометра")

    # --- 3. калибровка потенциометров ---------------------------------------- #
    note("info", "калибрую потенциометры (ноль 0°, −60°, +60°)…")
    result = _queue_with_retry(lambda: source.set_pot_mode("calibrate"), latched, note,
                               sleep, "калибровка потенциометра")
    if not result.get("ok"):
        note("err", "калибровка потенциометра не запущена: %s" % (result.get("reason") or "unknown"))
        return {"ok": False, "reason": result.get("reason"), "stage": "pot"}
    state = _wait_for(lambda: any(f.get("calib") or f.get("moving") for f in _fresh_frames(source, now()).values()),
                      timeout=POT_START_TIMEOUT_SEC, sleep=sleep, monotonic=monotonic, latched=latched)
    if state == "estop":
        note("warn", "E-STOP взведён до старта калибровки потенциометров — последовательность прервана")
        return {"ok": False, "reason": "estop", "stage": "pot"}
    if state != "ok":
        note("err", "калибровка потенциометра не стартовала: платы не отреагировали на q (связь?)")
        return {"ok": False, "reason": "not_started", "stage": "pot"}
    state = _wait_for(lambda: _pot_done(source, now()), timeout=POT_DONE_TIMEOUT_SEC,
                      sleep=sleep, monotonic=monotonic, latched=latched)
    if state == "estop":
        note("warn", "E-STOP взведён во время калибровки потенциометров — последовательность прервана")
        return {"ok": False, "reason": "estop", "stage": "pot"}
    if state != "ok":
        note("err", "калибровка потенциометра не завершилась за %d с" % int(POT_DONE_TIMEOUT_SEC))
        return {"ok": False, "reason": "timeout", "stage": "pot"}
    missing = _pot_ready(source, now())
    if missing:
        note("err", "потенциометр не подтверждён у: %s — проверьте датчик A0 и провод" %
             ", ".join(missing))
        return {"ok": False, "reason": "not_calibrated", "stage": "pot", "modules": missing}
    note("ok", "калибровка потенциометров завершена у всех четырёх; режим потенциометра включён")
    return {"ok": True, "stage": "done"}


def _run_quietly(source, latched) -> None:
    try:
        outcome = run(source, latched)
    except Exception:  # noqa: BLE001 — поток-демон не должен ронять киоск
        log.exception("автокалибровка: неожиданная ошибка")
        try:
            eventlog.record("err", "автокалибровка: неожиданная ошибка, см. лог процесса",
                            source=_SOURCE_TAG)
        except Exception:  # noqa: BLE001
            pass
        return
    if not outcome.get("ok"):
        log.warning("автокалибровка: остановлена на этапе %s (%s)",
                    outcome.get("stage"), outcome.get("reason"))


def start(state, *, latched: Callable[[], object] | None = None, env: Any = None) -> bool:
    """Запустить поток автокалибровки. ``False`` — если она неприменима/выключена."""
    if not autostart_enabled(env):
        log.info("автокалибровка: выключена переменной RC_AUTO_CALIBRATE")
        return False
    source = getattr(state, "source", None)
    if source is None:
        log.info("автокалибровка: источник данных не готов — пропускаю")
        return False
    if not getattr(source, "supports_calibration", False):
        log.info("автокалибровка: источник %s не калибрует — пропускаю",
                 getattr(source, "name", "unknown"))
        return False
    for capability in ("calibrate_module", "set_pot_mode"):
        if not callable(getattr(source, capability, None)):
            log.info("автокалибровка: источник не поддерживает %s — пропускаю", capability)
            return False
    if getattr(source, "frames", None) is None:
        log.info("автокалибровка: источник не публикует кадры — пропускаю")
        return False
    if latched is None:
        from . import estop
        latched = estop.latched
    thread = threading.Thread(target=_run_quietly, args=(source, latched),
                              name="auto-calibrate", daemon=True)
    thread.start()
    log.info("автокалибровка: поток запущен — жду все четыре колеса "
             "(отключить: RC_AUTO_CALIBRATE=0)")
    return True
