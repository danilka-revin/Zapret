"""Режим «Скан»: сначала строим карту, потом по ней ездим.

Отдельный режим для оператора, который объезжает помещение и собирает карту
лидаром. Отличие от обычного руления и от автономного обследования:

* управление **включается явно** кнопкой и выключается кнопкой же, клавишей
  Esc, потерей связи или молчанием страницы (`DEADMAN_S`). Пока управление
  выключено, команды тяги и руления из этого режима отклоняются: «выключено»
  значит «не поедет», а не «поедет, если нажмут».
* выключение всегда сначала тормозит: STOP уходит на шасси до смены состояния,
  даже если браузер уже закрылся.
* тяга ограничена отдельным потолком `RC_SCAN_MAX_PERCENT` (по умолчанию 20 %
  шкалы Servo): карту снимают медленно, а лидар теряет позу на резких манёврах.
  Ноль разрешён всегда — это остановка.

Сервис ничего не знает про Flask и про то, как именно устроен источник: ему
передают функции отправки тяги и руления, поэтому он одинаково работает с
настоящим шасси и в тестах.
"""
from __future__ import annotations

import os
import secrets
import threading
import time
from typing import Any, Callable, Optional

#: Потолок тяги режима «Скан», % шкалы Servo.
DEFAULT_MAX_PERCENT = 20

#: Сколько секунд страница может молчать, прежде чем управление отключится.
#: Прошивка сама гасит колесо через 650 мс без heartbeat, поэтому запас
#: намеренно небольшой: это «человек ушёл от экрана», а не «сеть тормозит».
DEADMAN_S = 5.0

#: Переменная окружения с потолком тяги.
ENV_MAX_PERCENT = "RC_SCAN_MAX_PERCENT"


def env_max_percent() -> int:
    """Потолок тяги режима «Скан» из ``RC_SCAN_MAX_PERCENT`` (1…100)."""
    try:
        limit = int(os.environ.get(ENV_MAX_PERCENT, DEFAULT_MAX_PERCENT))
    except (TypeError, ValueError):
        limit = DEFAULT_MAX_PERCENT
    return max(1, min(100, limit))


class ScanModeError(Exception):
    """Отказ режима «Скан» с причиной для экрана и кодом HTTP."""

    def __init__(self, message: str, reason: str, status: int = 409) -> None:
        super().__init__(message)
        self.message = message
        self.reason = reason
        self.status = status

    def payload(self) -> dict[str, Any]:
        return {"ok": False, "reason": self.reason, "error": self.message}


class ScanModeService:
    """Включаемое вручную управление для съёмки карты.

    ``drive(source, body)`` и ``steer(source, body)`` — те же функции отправки,
    что использует обычное руление (``slam.dispatch_drive_command`` и
    ``slam.dispatch_steering_command``). Источник берётся заново на каждую
    команду: шасси могло переподключиться, а закешированный объект — устареть.
    """

    def __init__(self,
                 source: Callable[[], Any],
                 drive: Callable[[Any, dict], tuple],
                 steer: Callable[[Any, dict], tuple],
                 estop: Optional[Callable[[], Any]] = None,
                 clock: Callable[[], float] = time.monotonic,
                 max_percent: Optional[int] = None,
                 deadman_s: float = DEADMAN_S) -> None:
        self._source = source
        self._drive = drive
        self._steer = steer
        self._estop = estop
        self._clock = clock or time.monotonic
        self._lock = threading.RLock()
        self.max_percent = env_max_percent() if max_percent is None else max(1, min(100, int(max_percent)))
        self.deadman_s = float(deadman_s)
        self._enabled = False
        self._token: Optional[str] = None
        self._last_heard = 0.0
        self._enabled_at = 0.0
        self._reason = ""
        self._commands = 0

    # -- состояние --------------------------------------------------------- #
    def status(self) -> dict[str, Any]:
        with self._lock:
            if self._enabled:
                self._expire_locked()
            now = self._clock()
            return {
                "ok": True,
                "enabled": self._enabled,
                "token": self._token,
                "maxPercent": self.max_percent,
                "deadmanS": self.deadman_s,
                "silentS": (None if not self._enabled
                            else round(max(0.0, now - self._last_heard), 3)),
                "enabledAt": self._enabled_at or None,
                "reason": self._reason,
                "commands": self._commands,
                "estop": self._estop_latched(),
            }

    def _estop_latched(self) -> bool:
        if self._estop is None:
            return False
        try:
            return bool(self._estop())
        except Exception:  # noqa: BLE001 — защёлка не должна ронять режим
            return True

    def _expire_locked(self) -> None:
        """Молчание страницы дольше deadman — выключаем и тормозим."""
        if not self._enabled:
            return
        if self._clock() - self._last_heard <= self.deadman_s:
            return
        self._disable_locked("deadman")

    # -- включение и выключение -------------------------------------------- #
    def enable(self) -> dict[str, Any]:
        """Включить управление. Перед этим — STOP: включаем только на стоящем."""
        if self._estop_latched():
            raise ScanModeError("E-STOP взведён: снимите его на роботе.", "estop", 409)
        result, status = self._send_drive({"percent": 0})
        if not result.get("ok"):
            raise ScanModeError(
                "Шасси не подтвердило остановку: %s" % _reason_text(result.get("reason")),
                "brake_failed", 503 if status >= 500 else 409)
        with self._lock:
            self._enabled = True
            self._token = secrets.token_urlsafe(18)
            self._last_heard = self._clock()
            self._enabled_at = self._last_heard
            self._reason = "enabled"
            self._commands = 0
            return {"ok": True, "enabled": True, "token": self._token,
                    "maxPercent": self.max_percent, "deadmanS": self.deadman_s}

    def disable(self, reason: str = "disabled") -> dict[str, Any]:
        """Выключить управление: сначала STOP, потом смена состояния."""
        with self._lock:
            return self._disable_locked(reason)

    def _disable_locked(self, reason: str) -> dict[str, Any]:
        had_motion = self._enabled
        self._enabled = False
        self._token = None
        self._reason = reason
        result, _status = self._send_drive({"percent": 0})
        return {"ok": True, "enabled": False, "braked": bool(result.get("ok")),
                "reason": reason, "wasEnabled": had_motion,
                "maxPercent": self.max_percent}

    def heartbeat(self, token: Optional[str] = None) -> dict[str, Any]:
        """Страница жива: продлить окно и вернуть состояние."""
        with self._lock:
            if not self._enabled:
                return {"ok": True, "enabled": False, "token": None,
                        "maxPercent": self.max_percent, "deadmanS": self.deadman_s}
            self._check_token_locked(token)
            self._last_heard = self._clock()
            return {"ok": True, "enabled": True, "token": self._token,
                    "maxPercent": self.max_percent, "deadmanS": self.deadman_s}

    def _check_token_locked(self, token: Optional[str]) -> None:
        if token is None:
            return
        if token != self._token:
            raise ScanModeError("Управление перехвачено другой вкладкой: включите заново.",
                                "scan_token", 409)

    # -- команды ------------------------------------------------------------ #
    def drive(self, body: dict, token: Optional[str] = None) -> tuple[dict, int]:
        percent = _percent(body)
        if percent == 0:
            # Остановка разрешена всегда, в том числе при выключенном управлении.
            result, status = self._send_drive({"percent": 0})
            return result, status
        self._require_enabled(token)
        if abs(percent) > self.max_percent:
            raise ScanModeError("Предел тяги режима «Скан» — %d %%." % self.max_percent,
                                "drive_limit", 400)
        payload = {"percent": percent}
        module = body.get("module") if isinstance(body, dict) else None
        if module is not None:
            payload["module"] = module
        result, status = self._send_drive(payload)
        if result.get("ok"):
            self._count_locked()
        return result, status

    def steer(self, body: dict, token: Optional[str] = None) -> tuple[dict, int]:
        self._require_enabled(token)
        result, status = self._send_steer(body)
        if result.get("ok"):
            self._count_locked()
        return result, status

    def _count_locked(self) -> None:
        with self._lock:
            self._commands += 1
            self._last_heard = self._clock()

    def _require_enabled(self, token: Optional[str]) -> None:
        with self._lock:
            if not self._enabled:
                raise ScanModeError("Управление в режиме «Скан» выключено.", "scan_disabled", 409)
            if self._clock() - self._last_heard > self.deadman_s:
                self._disable_locked("deadman")
                raise ScanModeError("Страница долго молчала — управление снято.",
                                    "scan_timeout", 409)
            self._check_token_locked(token)
            self._last_heard = self._clock()

    # -- отправка ----------------------------------------------------------- #
    def _send_drive(self, body: dict) -> tuple[dict, int]:
        try:
            source = self._source()
        except Exception:  # noqa: BLE001 — нет источника — это отказ, а не падение
            return {"ok": False, "reason": "serial_unavailable"}, 503
        if source is None:
            return {"ok": False, "reason": "serial_unavailable"}, 503
        try:
            return self._drive(source, body)
        except Exception as exc:  # noqa: BLE001 — режим не должен ронять сервер
            return {"ok": False, "reason": "command_failed", "error": str(exc)}, 503

    def _send_steer(self, body: dict) -> tuple[dict, int]:
        try:
            source = self._source()
        except Exception:  # noqa: BLE001
            return {"ok": False, "reason": "serial_unavailable"}, 503
        if source is None:
            return {"ok": False, "reason": "serial_unavailable"}, 503
        try:
            return self._steer(source, body)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "reason": "command_failed", "error": str(exc)}, 503


def _percent(body: Any) -> float:
    """Процент тяги из тела команды. Ошибка формата — отказ, а не ноль."""
    if not isinstance(body, dict):
        raise ScanModeError("Ожидается JSON-объект.", "payload", 400)
    raw = body.get("percent")
    if isinstance(raw, bool):
        raise ScanModeError("percent должен быть числом.", "invalid_drive", 400)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        raise ScanModeError("percent должен быть числом.", "invalid_drive", 400) from None
    if value != value or value in (float("inf"), float("-inf")):
        raise ScanModeError("percent должен быть конечным числом.", "invalid_drive", 400)
    if abs(value) > 100:
        raise ScanModeError("percent вне 0…100.", "invalid_drive", 400)
    return value


_REASONS = {
    "estop": "E-STOP",
    "serial_unavailable": "нет связи с шасси",
    "control_not_supported": "источник не умеет тягу",
    "module_offline": "нет телеметрии колёс",
    "not_homed": "не откалиброван руль",
    "neutral_locked": "включён нейтральный режим",
}


def _reason_text(reason: Any) -> str:
    return _REASONS.get(str(reason or ""), str(reason or "неизвестная причина"))
