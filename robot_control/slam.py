"""Веб-интерфейс RUS SLAM внутри пульта оператора.

Интерфейс взят из https://github.com/danilka-revin/rus_slam (каталог ``gui/``)
и лежит в репозитории в ``slam_gui/`` — робот работает без интернета, поэтому
подтягивать его откуда-то при запуске нельзя.

Здесь мы:
  * импортируем ``slam_gui/backend.py`` как модуль (он написан только на
    стандартной библиотеке, pyserial и ROS подключаются по желанию);
  * выбираем источник данных: ROS 2, UART-модули, мастер по одному UART или симуляция;
  * отдаём страницы и JSON API через Flask, под общей авторизацией пульта.

Страницы:  ``/`` и ``/main.html`` — основной экран робота,
           ``/console`` и ``/index.html`` — инженерный пульт.
JS обращается к API относительными путями (``fetch('api/state')``), поэтому
консоль живёт строго на ``/console`` без завершающего слэша.
"""

from __future__ import annotations

import glob
import importlib.util
import logging
import os
import math
import re
import threading
import time
from pathlib import Path

from flask import Blueprint, Response, jsonify, request, send_from_directory, session

from robot_control import estop as estop_latch
from robot_control import eventlog
from robot_control.camera import CameraManager
from robot_control.lidar import (
    LidarManager,
    lidar_holds_port,
    lidar_reserved_ports,
    set_self_port,
    startup_summary,
    tag_module_port,
    try_lock_port,
    unlock_port,
    untag_module_port,
)

log = logging.getLogger("robot_control.slam")

GUI_DIR = Path(__file__).resolve().parent.parent / "slam_gui"
BACKEND_PY = GUI_DIR / "backend.py"

MAIN_PAGE = "main.html"
CONSOLE_PAGE = "index.html"
KIOSK_PAGE = "robot.html"
#: Путь бортового экрана: его же открывает браузер на дисплее робота.
KIOSK_URL_PATH = "/kiosk"

#: Ввод PIN открывает короткую сессию для руления и удерживаемой тяги.
STEERING_SESSION_KEY = "kiosk_steering_expires"
STEERING_SESSION_TTL_S = 5 * 60
#: Предел абсолютного угла руля. Совпадает с ``STEER_LIMIT_DEG`` в
#: ``slam_gui/backend.py`` и в прошивке; если источник задаёт свой предел
#: (``MAX_STEER_DEG``), проверка и ответ берут его — иначе API обещает одно,
#: а плата режет другое, и руление выглядит «кривым».
STEERING_ANGLE_LIMIT_DEG = 100.0
STEERING_COMMAND_HEADER = "X-Robot-Control"
STEERING_COMMAND_HEADER_VALUE = "steering"
DRIVE_COMMAND_HEADER_VALUE = "drive"
#: Заголовок кнопки «Остановить всё» на бортовом экране: у киоска нет входа
#: оператора, поэтому запрос на остановку помечается так же, как команды
#: движения — сторонний POST из браузера такой заголовок поставить не может.
STOP_COMMAND_HEADER_VALUE = "stop"
#: Заводской предел тяги. Плата может прислать свой (поле dmax= прошивки с
#: командой m): экран и API берут предел из телеметрии, но не выше потолка.
MAX_DRIVE_PERCENT = 20
DRIVE_ABS_MAX_PERCENT = 100


def grant_steering_access(now=None):
    """Разрешить ручной поворот модулей на короткое время после проверки PIN."""
    session[STEERING_SESSION_KEY] = (time.time() if now is None else now) + STEERING_SESSION_TTL_S


def revoke_steering_access():
    session.pop(STEERING_SESSION_KEY, None)


def steering_access_valid(now=None):
    """Проверить и очистить истёкший доступ к рулению и тяге."""
    now = time.time() if now is None else now
    expires = session.get(STEERING_SESSION_KEY, 0)
    if not isinstance(expires, (int, float)) or expires <= now:
        # Истёкшая сессия — настоящее событие: на экране видно, почему руление
        # снова требует PIN. Сессии, которой не было, в журнал не попадает:
        # иначе запись появлялась бы на каждом опросе состояния.
        if expires:
            eventlog.record(
                "warn",
                "PIN-сессия руления истекла (%.0f мин) — команды снова требуют PIN"
                % (STEERING_SESSION_TTL_S / 60.0),
                source="руление",
            )
        revoke_steering_access()
        return False
    return True


#: Русский текст причины отказа проверки PIN (сам код в журнал не пишется).
PIN_REASON_TEXT = {
    "wrong": "неверный PIN",
    "format": "PIN должен быть 4…8 цифр",
    "blocked": "ввод заблокирован после серии ошибок",
    "ok": "PIN принят",
}


def note_pin_result(result, *, where="киоск"):
    """Записать в журнал настоящий итог проверки PIN (подробности без цифр кода)."""
    reason = str((result or {}).get("reason", ""))
    text = PIN_REASON_TEXT.get(reason, reason or "проверка не пройдена")
    if (result or {}).get("ok"):
        eventlog.record(
            "ok",
            "PIN принят (%s): руление и тяга разрешены на %.0f мин"
            % (where, STEERING_SESSION_TTL_S / 60.0),
            source="pin",
        )
    else:
        eventlog.record("warn", "PIN не принят (%s): %s" % (where, text), source="pin")


#: Последняя записанная ступень тяги: heartbeat ручки не должен забивать журнал.
_last_drive_percent: float | None = None
_last_drive_lock = threading.Lock()


def note_drive_command(percent: float, module: str = "") -> None:
    """Записать ступень тяги (0 — тормоз), а не каждый heartbeat удержания.

    Кнопку держат, и команда уходит 5 раз в секунду; в журнале отражается
    смена ступени, то есть реальное действие оператора. Повторы внутри
    ``eventlog`` сворачиваются в счётчик ``×N``. У одиночной тяги ступень
    своя на каждое колесо — иначе FL вперёд и FR назад выглядели бы одним
    «0», и в журнале не осталось бы, что именно крутили.
    """
    global _last_drive_percent
    step = round(float(percent))
    module = str(module or "").upper()
    key = (module, step)
    with _last_drive_lock:
        if _last_drive_percent == key:
            return
        _last_drive_percent = key
    # В журнале видно, что именно крутили: «колесо FL» или «BLDC» (все четыре).
    label = ("колесо " + module) if module else "BLDC"
    if step == 0:
        eventlog.record("info", "тяга %s: 0%% — тормоз" % label, source="тяга")
    else:
        eventlog.record(
            "ok",
            "тяга %s %+d%% (%s)" % (label, step, "вперёд" if step > 0 else "назад"),
            source="тяга",
        )


#: Русский текст причины отказа команды — для журнала экрана.
COMMAND_REASON_TEXT = {
    "payload": "неверный состав полей",
    "invalid_module": "неизвестный модуль",
    "invalid_angle": "неверный угол",
    "angle_limit": "угол вне допуска руля",
    "invalid_drive": "неверная величина тяги",
    "drive_limit": "тяга вне допуска (см. предел в настройке привода)",
    "invalid_action": "неизвестное действие настройки привода",
    "invalid_value": "недопустимое значение настройки привода",
    "firmware_update_required": "прошивка плат не умеет настройку тяги",
    "estop": "E-STOP: движение заблокировано, снимите защёлку с PIN",
    "control_not_supported": "источник данных не умеет адресное управление",
    "drive_firmware_required": "прошивка плат не умеет тягу (нет поля drive=)",
    "module_offline": "модуль не на связи",
    "serial_unavailable": "нет связи по UART",
    "command_queue_full": "очередь команд занята",
    "command_failed": "команда не выполнена",
    "pin_required": "нужен PIN руления",
    "request_header_required": "запрос без заголовка X-Robot-Control",
    "module_busy": "модуль занят (поворачивается, калибруется или включена тяга)",
    "not_homed": "у модуля нет нуля — откалибруйте по датчику или задайте 0°",
}


def note_pin_required(action: str) -> None:
    """Команда без PIN-сессии: на экране видно, чего именно не хватает."""
    note_command_rejected(action, "pin_required")


def note_command_rejected(action: str, reason: str) -> None:
    """Отказ команды — тоже настоящее событие: на экране видно, почему её нет."""
    text = COMMAND_REASON_TEXT.get(reason, reason or "причина неизвестна")
    eventlog.record("warn", "%s отклонено: %s" % (action, text), source="управление")


def note_steering_command(module: str, angle: float) -> None:
    """Записать настоящую команду поворота одного модуля."""
    eventlog.record("ok", "руление %s → %+.1f°" % (module, float(angle)), source="руление")


def note_steering_zero(module: str) -> None:
    """Записать ручной программный ноль: без движения, только отсчёт от колеса."""
    eventlog.record(
        "info", "программный ноль %s выставлен вручную (без движения)" % module,
        source="руление",
    )


def note_steering_calibration(module: str) -> None:
    """Записать запуск калибровки по оптодатчику D18 (колесо будет двигаться)."""
    target = "всех колёс" if module == "ALL" else module
    eventlog.record(
        "ok", "калибровка руля %s по оптодатчику запущена" % target, source="руление",
    )


#: Слова в сообщении прошивки, после которых запись в журнале — предупреждение.
_MASTER_WARN_WORDS = ("ОШИБКА", "нет связи", "не доставлена", "нет команды", "отклонена",
                      "заблокирован", "Неизв.")


def note_master_message(text: str) -> None:
    """Сообщение прошивки мастера («# FR: Калибровка ЗАВЕРШЕНА!») — в журнал экрана."""
    text = str(text or "").strip()
    if not text:
        return
    level = "warn" if any(word in text for word in _MASTER_WARN_WORDS) else "info"
    eventlog.record(level, "прошивка " + text, source="мастер")


def note_master_status(level: str, text: str) -> None:
    """Поиск порта мастера и его обрыв — в журнал экрана, а не только в stdout."""
    text = str(text or "").strip()
    if not text:
        return
    eventlog.record(level if level in {"ok", "warn", "err"} else "info", text,
                    source="мастер")


def reset_journal_notes() -> None:
    """Забыть, о чём журнал уже писал: повторная инициализация и тесты.

    Без этого после очистки журнала тот же статус лидара считался бы «уже
    записанным» и на экране осталась бы пустота.
    """
    global _lidar_note, _last_drive_percent
    with _lidar_note_lock:
        _lidar_note = None
    with _last_drive_lock:
        _last_drive_percent = None


#: Повторяющийся «лидар:» внутри причины статуса — префикс нужен один.
_LIDAR_PREFIX_RE = re.compile(r"(?i)(?:^|(?<=[\s(]))лидар:\s*")

#: Последнее состояние лидара, о котором уже написано в журнал.
_lidar_note: tuple | None = None
_lidar_note_lock = threading.Lock()


def note_lidar_status(status) -> None:
    """Записать в журнал смену состояния лидара — и только смену.

    Статус пересобирается на каждом запросе экрана, поэтому в журнал попадает
    переход («на связи» → «скан не идёт»), а не одна и та же строка подряд.
    """
    global _lidar_note
    if not isinstance(status, dict):
        return
    connected = bool(status.get("connected"))
    reason = str(status.get("reason", ""))
    # Scan counters and growing age describe telemetry, not a new event.
    reason_key = "" if connected else re.sub(
        r"(?i)(скан устарел)\s*\([^)]*с без точек\)", r"\1", reason)
    key = (
        connected,
        bool(status.get("demo")),
        reason_key,
        str(status.get("source", "")),
    )
    with _lidar_note_lock:
        if key == _lidar_note:
            return
        _lidar_note = key
    connected, demo, _reason_key, source = key
    # Причина приходит из статуса со своим «лидар:» (иногда дважды) — на экране
    # это читалось бы как «лидар: … лидар: …», поэтому префикс оставляем один.
    detail = _LIDAR_PREFIX_RE.sub("", reason).strip()
    if connected:
        points = status.get("points_count")
        where = "%s, %s точек" % (source or "источник", points) if points else (source or "")
        eventlog.record("ok", "лидар на связи (" + where + ")" if where else "лидар на связи",
                        source="лидар")
    elif demo:
        eventlog.record("warn", "лидар: " + (detail or "реального лидара нет, показан демо-скан"),
                        source="лидар")
    else:
        eventlog.record("warn", "лидар: скана нет — " + (detail or "причина неизвестна"),
                        source="лидар")


def steering_angle_limit_deg(source=None):
    """Действующий предел угла руля в градусах.

    Источник (``MasterSource``) знает свой предел — берём его, иначе экран
    получил бы одно число в ``limitDeg``, а команда была бы отклонена по
    другому. Так предел один: прошивка → источник → API → ползунки экрана.
    """
    limit = getattr(source, "MAX_STEER_DEG", None)
    try:
        limit = float(limit)
    except (TypeError, ValueError):
        return STEERING_ANGLE_LIMIT_DEG
    if not math.isfinite(limit) or limit <= 0:
        return STEERING_ANGLE_LIMIT_DEG
    return limit


def steering_control_info(source):
    """Публичная возможность управления: мастер, адресные команды, ноль руля.

    Ноль — калибровкой по оптодатчику D18 (прошивка v2, команда ``k``), ручной
    «Задать 0°» остался запасным вариантом (например, датчик снят).
    """
    calibration = bool(getattr(source, "supports_calibration", False))
    return {
        "available": bool(getattr(source, "supports_individual_steer", False)),
        "source": getattr(source, "name", "unknown"),
        # Тяга каждого колеса отдельно: если источника нет, экран не рисует
        # кнопки «вперёд/назад» в карточках колёс.
        "individualDrive": bool(getattr(source, "supports_individual_drive", False)),
        "zeroMode": "opto" if calibration else "manual",
        "optoUsed": calibration,
        "calibration": calibration,
        "manualZero": True,
        "limitDeg": steering_angle_limit_deg(source),
    }


def dispatch_steering_command(source, body):
    """Проверить payload и отправить адресную команду одному рулевому модулю.

    Ответ не меняется, но отказ теперь виден в журнале экрана: иначе на борту
    «ничего не произошло» без объяснения — оператор не знает, ждать ему или
    искать причину.
    """
    result, status = _steering_command(source, body)
    if not result.get("ok"):
        note_command_rejected("руление", str(result.get("reason", "")))
    return result, status


def _steering_command(source, body):
    if estop_latch.latched() is not None:
        return {"ok": False, "reason": "estop"}, 409
    if not isinstance(body, dict) or set(body) != {"module", "angle"}:
        return {"ok": False, "reason": "payload"}, 400
    module = body.get("module")
    if not isinstance(module, str) or module.upper() not in {"FL", "FR", "RL", "RR"}:
        return {"ok": False, "reason": "invalid_module"}, 400
    module = module.upper()
    raw_angle = body.get("angle")
    if isinstance(raw_angle, bool):
        return {"ok": False, "reason": "invalid_angle"}, 400
    try:
        angle = float(raw_angle)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "invalid_angle"}, 400
    import math
    if not math.isfinite(angle):
        return {"ok": False, "reason": "invalid_angle"}, 400
    limit_deg = steering_angle_limit_deg(source)
    if abs(angle) > limit_deg:
        return {"ok": False, "reason": "angle_limit", "limit": limit_deg}, 400

    if not getattr(source, "supports_individual_steer", False):
        return {"ok": False, "reason": "control_not_supported",
                "source": getattr(source, "name", "unknown")}, 409
    command = getattr(source, "steer_module", None)
    if not callable(command):
        return {"ok": False, "reason": "control_not_supported"}, 409
    result = command(module, angle)
    if result.get("ok"):
        note_steering_command(module, result.get("angle", round(angle, 1)))
        return {"ok": True, "queued": True, "module": module,
                "angle": result.get("angle", round(angle, 1))}, 200

    reason = result.get("reason", "command_failed")
    status = 409
    if reason in {"serial_unavailable", "module_offline"}:
        status = 503
    elif reason == "command_queue_full":
        status = 429
    elif reason in {"invalid_module", "invalid_angle", "angle_limit"}:
        status = 400
    failure = {"ok": False, "reason": reason}
    if reason == "angle_limit":
        # Предел — из ответа источника: экран покажет настоящее число.
        failure["limit"] = result.get("limit", limit_deg)
    return failure, status


def dispatch_steering_zero(source, body):
    """Проверить payload и поставить в очередь программный ноль одного модуля."""
    result, status = _steering_zero(source, body)
    if not result.get("ok"):
        note_command_rejected("программный ноль", str(result.get("reason", "")))
    return result, status


def _steering_zero(source, body):
    if estop_latch.latched() is not None:
        return {"ok": False, "reason": "estop"}, 409
    if not isinstance(body, dict) or set(body) != {"module"}:
        return {"ok": False, "reason": "payload"}, 400
    module = body.get("module")
    if not isinstance(module, str) or module.upper() not in {"FL", "FR", "RL", "RR"}:
        return {"ok": False, "reason": "invalid_module"}, 400
    module = module.upper()
    if not getattr(source, "supports_individual_steer", False):
        return {"ok": False, "reason": "control_not_supported"}, 409
    command = getattr(source, "zero_module", None)
    if not callable(command):
        return {"ok": False, "reason": "control_not_supported"}, 409
    result = command(module)
    if result.get("ok"):
        note_steering_zero(module)
        return {"ok": True, "queued": True, "module": module,
                "zeroMode": "manual"}, 200
    reason = result.get("reason", "command_failed")
    status = 409
    if reason in {"serial_unavailable", "module_offline"}:
        status = 503
    elif reason == "command_queue_full":
        status = 429
    elif reason == "invalid_module":
        status = 400
    return {"ok": False, "reason": reason}, status


def _drive_limit(source) -> int:
    """Предел тяги источника: свой у платы, но не выше потолка сервера."""
    getter = getattr(source, "drive_limit_percent", None)
    limit = None
    if callable(getter):
        try:
            limit = int(getter())
        except Exception:                     # noqa: BLE001 — источник не должен ронять пульт
            limit = None
    if limit is None:
        limit = int(getattr(source, "MAX_DRIVE_PERCENT", MAX_DRIVE_PERCENT) or MAX_DRIVE_PERCENT)
    return max(1, min(limit, DRIVE_ABS_MAX_PERCENT))


def dispatch_drive_config(source, body):
    """Настройка привода тяги: выход ШИМ/0–5 В, тахометр, тормоз, предел, мВ."""
    action = body.get("action") if isinstance(body, dict) else None
    result, status = _drive_config(source, body)
    if not result.get("ok"):
        note_command_rejected("настройка привода (%s)" % (action or "?"),
                              str(result.get("reason", "")))
    else:
        eventlog.record("ok", "настройка привода тяги: %s" % (action or ""), source="тяга")
    return result, status


def _drive_config(source, body):
    if estop_latch.latched() is not None:
        return {"ok": False, "reason": "estop"}, 409
    if not isinstance(body, dict) or set(body) - {"action", "value"}:
        return {"ok": False, "reason": "payload"}, 400
    action = body.get("action")
    if not isinstance(action, str):
        return {"ok": False, "reason": "payload"}, 400
    command = getattr(source, "set_drive_config", None)
    if not callable(command):
        return {"ok": False, "reason": "control_not_supported"}, 409
    result = command(action, body.get("value"))
    if result.get("ok"):
        return result, 200
    reason = result.get("reason", "command_failed")
    status = 409
    if reason in {"serial_unavailable", "module_offline"}:
        status = 503
    elif reason == "command_queue_full":
        status = 429
    elif reason in {"payload", "invalid_action", "invalid_value"}:
        status = 400
    return dict(result, ok=False, reason=reason), status


def dispatch_pot_mode(source, body):
    if estop_latch.latched() is not None:
        return {"ok": False, "reason": "estop"}, 409
    if not isinstance(body, dict) or set(body) != {"action"} or not isinstance(body["action"], str) or body["action"] not in {"on", "off", "calibrate"}:
        return {"ok": False, "reason": "payload"}, 400
    command = getattr(source, "set_pot_mode", None)
    if not callable(command):
        return {"ok": False, "reason": "control_not_supported"}, 409
    result = command(body["action"])
    return result, 200 if result.get("ok") else 409


def dispatch_steering_calibration(source, body):
    """Проверить payload и запустить калибровку по оптодатчику (модуль или ALL)."""
    result, status = _steering_calibration(source, body)
    if not result.get("ok"):
        note_command_rejected("калибровка руля", str(result.get("reason", "")))
    return result, status


def _steering_calibration(source, body):
    if estop_latch.latched() is not None:
        return {"ok": False, "reason": "estop"}, 409
    if not isinstance(body, dict) or set(body) != {"module"}:
        return {"ok": False, "reason": "payload"}, 400
    module = body.get("module")
    if not isinstance(module, str) or module.upper() not in {"FL", "FR", "RL", "RR", "ALL"}:
        return {"ok": False, "reason": "invalid_module"}, 400
    module = module.upper()
    command = getattr(source, "calibrate_module", None)
    if not getattr(source, "supports_calibration", False) or not callable(command):
        return {"ok": False, "reason": "control_not_supported",
                "source": getattr(source, "name", "unknown")}, 409
    result = command(module)
    if result.get("ok"):
        note_steering_calibration(module)
        return {"ok": True, "queued": True, "module": module,
                "offline": list(result.get("offline") or [])}, 200
    reason = result.get("reason", "command_failed")
    status = 409
    if reason in {"serial_unavailable", "module_offline"}:
        status = 503
    elif reason == "command_queue_full":
        status = 429
    elif reason == "invalid_module":
        status = 400
    return {"ok": False, "reason": reason}, status


def engage_estop(source):
    """Нажатие E-STOP: взвести защёлку и сразу послать нулевую тягу в источник.

    Защёлка работает даже если источник не ответил: тогда все команды тяги и
    руления отклоняются, а нулевая тяга повторяется при следующем нажатии.
    """
    info = estop_latch.latch("E-STOP")
    drive = getattr(source, "drive", None)
    stopped = False
    if getattr(source, "supports_drive", False) and callable(drive):
        try:
            stopped = bool((drive(0) or {}).get("ok"))
        except Exception as exc:  # источник не должен ронять веб-пульт
            log.error("E-STOP: тяга не остановлена: %s", exc)
    eventlog.record("error", "E-STOP: защёлка взведена, тяга остановлена" if stopped
                    else "E-STOP: защёлка взведена, источник тяги не подтвердил останов",
                    source="руление")
    return {"ok": True, "estop": True, "driveStopped": stopped, "since": info.get("since")}


def release_estop():
    """Снять защёлку. Вызывать только после проверки PIN-сессии руления."""
    if estop_latch.latched() is None:
        return {"ok": True, "estop": False}
    estop_latch.release()
    eventlog.record("warn", "E-STOP снят оператором с PIN", source="руление")
    return {"ok": True, "estop": False}


def dispatch_drive_command(source, body):
    """Проверить ограниченную команду тяги (отрицательная величина — назад).

    Одна и та же ручка работает для всех колёс (``{"percent": 8}``) и для
    одного колеса (``{"module": "FL", "percent": 8}``): второй вариант —
    адресная команда, как ``/api/kiosk_steer`` для руля.
    """
    result, status = _drive_command(source, body)
    if not result.get("ok"):
        note_command_rejected("тяга", str(result.get("reason", "")))
    return result, status


def _drive_command(source, body):
    if not isinstance(body, dict):
        return {"ok": False, "reason": "payload"}, 400
    keys = set(body)
    module = None
    if keys == {"module", "percent"}:
        raw_module = body.get("module")
        if not isinstance(raw_module, str) or raw_module.upper() not in {"FL", "FR", "RL", "RR"}:
            return {"ok": False, "reason": "invalid_module"}, 400
        module = raw_module.upper()
    elif keys != {"percent"}:
        return {"ok": False, "reason": "payload"}, 400
    raw_percent = body.get("percent")
    if isinstance(raw_percent, bool):
        return {"ok": False, "reason": "invalid_drive"}, 400
    try:
        percent = float(raw_percent)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "invalid_drive"}, 400
    if not math.isfinite(percent):
        return {"ok": False, "reason": "invalid_drive"}, 400
    if percent != 0 and estop_latch.latched() is not None:
        return {"ok": False, "reason": "estop"}, 409
    limit = _drive_limit(source)
    if abs(percent) > limit:
        return {"ok": False, "reason": "drive_limit", "limit": limit}, 400
    if module is not None:
        # Одиночная тяга — только там, где каждое колесо на своём порту.
        if not getattr(source, "supports_individual_drive", False):
            return {"ok": False, "reason": "control_not_supported"}, 409
        command = getattr(source, "drive_module", None)
    else:
        if not getattr(source, "supports_drive", False):
            return {"ok": False, "reason": "control_not_supported"}, 409
        command = getattr(source, "drive", None)
    if not callable(command):
        return {"ok": False, "reason": "control_not_supported"}, 409
    result = command(module, percent) if module is not None else command(percent)
    if result.get("ok"):
        applied = result.get("percent", round(percent))
        note_drive_command(applied, module or "")
        answer = {"ok": True, "queued": True, "percent": applied,
                  "maxPercent": result.get("maxPercent", limit)}
        if module is not None:
            answer["module"] = module
        return answer, 200
    reason = result.get("reason", "command_failed")
    status = 409
    if reason in {"serial_unavailable", "module_offline"}:
        status = 503
    elif reason == "command_queue_full":
        status = 429
    elif reason in {"invalid_drive", "drive_limit", "payload", "invalid_module"}:
        status = 400
    failure = {"ok": False, "reason": reason}
    if reason == "drive_limit":
        failure["limit"] = result.get("limit", limit)
    return failure, status


#: Статика интерфейса: имя файла → endpoint. Отдаётся только этот список.
ASSETS = {
    "steer_dial.js": "steer_dial_js",
    "steer_dial.css": "steer_dial_css",
    "connections.css": "connections_css",
    "connections.js": "connections_js",
    "main.css": "main_css",
    "main.js": "main_js",
    # карта лидара в стиле RViz (зум, сетка, Decay Time) — общая для пульта и бортового экрана
    "lidar_view.js": "lidar_view_js",
    "robot_model.js": "robot_model_js",
    "map.css": "map_css",
    "map_view.js": "map_view_js",
    "viewport.js": "viewport_js",
    "camera_view.js": "camera_view_js",
    "styles.css": "styles_css",
    "app.js": "app_js",
    "console.js": "console_js",
    "console-core.js": "console_core_js",
    "vision.js": "vision_js",
    # вкладка «Сводка» основного экрана (рейсы, энергия, замок — с сервера)
    "summary.js": "summary_js",
    "summary.css": "summary_css",
    # наш скрипт: пробрасывает токен сессии в подзапросы (нужен в iframe без cookie)
    "slam_auth.js": "slam_auth_js",
    # страница киоска на борту робота (без авторизации)
    "robot.css": "robot_css",
    "robot.js": "robot_js",
    "battery.js": "battery_js",
    "steering.css": "steering_css",
    "steering.js": "steering_js",
    "wheel_settings.js": "wheel_settings_js",
    "web_controls.js": "web_controls_js",
    # общие пресеты руления: одни и те же углы на киоске и веб-пульте
    "steer_presets.js": "steer_presets_js",
    "pot_mode.js": "pot_mode_js",
    "update.css": "update_css",
    "update.js": "update_js",
    # кнопка «Остановить всё»: пульт, бортовой экран, лидар и окно киоска
    "power.css": "power_css",
    "power.js": "power_js",
    # панель настроек лидара (кнопка «Лидар» на пульте и на бортовом экране)
    "lidar_settings.css": "lidar_settings_css",
    "lidar_settings.js": "lidar_settings_js",
}

#: Порты, которые проверяем в режиме ``auto``, если список не задан явно.
DEFAULT_PORTS = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]

_backend = None


def load_backend():
    """Загружает vendored ``backend.py`` (один раз на процесс)."""
    global _backend
    if _backend is not None:
        return _backend
    if not BACKEND_PY.is_file():
        raise RuntimeError(f"не найден {BACKEND_PY} — интерфейс RUS SLAM не установлен")
    spec = importlib.util.spec_from_file_location("rus_slam_backend", str(BACKEND_PY))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _backend = module
    return module


def _have(module_name: str) -> bool:
    """Есть ли модуль в этом интерпретаторе.

    Для ``rclpy`` отдельная ветка: ROS 2 ставится системно в ``/opt/ros``, а
    приложение работает из ``.venv`` и без подготовки путей системных пакетов не
    видит (см. :mod:`robot_control.ros_env`). Раньше ``auto`` из-за этого
    выбирал UART-модули, хотя ROS 2 на роботе стоял.
    """
    if module_name == "rclpy":
        from . import ros_env
        return ros_env.rclpy_available()
    try:
        __import__(module_name)
    except ImportError:
        return False
    return True


def _present_ports(ports):
    return [p for p in ports if os.path.exists(p)]


def chassis_ports(ports):
    """Порты модулей шасси без тех, что отданы лидару.

    USB-порт можно читать одному: если источник данных шасси в режиме ``auto``
    сам найдёт все ``/dev/ttyUSB*`` и откроет порт лидара, лидар останется без
    данных — именно так «скана нет» в веб-интерфейсе при живом железе (скрипт
    ``scripts/lidar_check.py`` при этом работает: его запускают, когда пульт
    остановлен). Порт из ``RC_LIDAR_PORT`` сравнивается по realpath, потому что
    на роботе это обычно ``/dev/serial/by-id/…`` → ``/dev/ttyUSB1``.
    """
    from .battery import reserved_ports
    reserved = lidar_reserved_ports() | reserved_ports()
    if not reserved:
        return list(ports)
    kept, skipped = [], []
    for port in ports:
        try:
            real = os.path.realpath(port)
        except OSError:
            real = port
        if port in reserved or real in reserved:
            skipped.append(port)
        else:
            kept.append(port)
    if skipped:
        log.info(
            "шасси: порт(ы) %s отданы лидару/BMS (RC_LIDAR_PORT/RC_BMS_PORT) — модули их не открывают",
            ", ".join(skipped),
        )
    return kept


def _autodetect_ports():
    """Реально существующие UART-адаптеры робота (кроме порта лидара)."""
    found = chassis_ports(_present_ports(DEFAULT_PORTS))
    if found:
        return found
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        found = chassis_ports(sorted(glob.glob(pattern)))
        if found:
            return found
    return []


def chassis_source(mod, ports):
    """Источник данных шасси на этих портах — под тем же замком, что у лидара.

    Замок порта (``try_lock_port``/``unlock_port``) общий на всё приложение:
    пока порт не доказал, что он — модуль шасси, его читает только один процесс
    робота, а лидар (он берёт порт монопольно, ``exclusive`` в pyserial) всегда
    получает его свободным. Без этого пульт и бортовой экран читали один USB-порт
    вдвоём — «device reports readiness to read but returned no data (… multiple
    access on port?)» — и лидар не мог найти себя ни на одном порту, хотя порты
    держало это же приложение, а на экране появлялся совет «проверь кабель».

    Замка мало: он говорит только «занят / свободен» и не различает, кто держит
    порт. Поэтому здесь же два механизма из ``robot_control.lidar``:

      * ``yield_to_lidar`` (:func:`lidar_holds_port`) — пока лидар ищет свой порт
        (окно ``RC_LIDAR_PRIORITY_S``, 20 с) или уже читает этот порт, модули не
        занимают его своей проверкой. Иначе на роботе с двумя адаптерами оба
        перебора сталкивались, и лидар сообщал «все 2 порт(ов) заняты другой
        программой» про порты, которые держало это же приложение;
      * ``tag_module``/``untag_module`` (:func:`tag_module_port`) — отметка в
        ``data/`` «на этом порту подтвердились модули». Её видит лидар и не
        перебирает такой порт: он всё равно занят, а совет «проверь кабель»
        уводит в сторону.
    """
    from .battery import reserved_ports
    bms_ports = reserved_ports()
    ports = [p for p in ports if p not in bms_ports and os.path.realpath(p) not in bms_ports]
    return mod.SerialSource(
        ports,
        claim=try_lock_port,
        release=unlock_port,
        yield_to_lidar=lidar_holds_port,
        tag_module=tag_module_port,
        untag_module=untag_module_port,
    )


def peer_kiosk_url() -> str:
    """Адрес бортового экрана, откуда пульт берёт телеметрию мастера."""
    explicit = (os.environ.get("RC_SLAM_PEER_URL") or "").strip()
    if explicit:
        return explicit
    port = (os.environ.get("RC_KIOSK_PORT") or "8090").strip() or "8090"
    return f"http://127.0.0.1:{port}/api/kiosk_state"


def make_source(mode: str = "auto", ports=None, master_role: str = "owner"):
    """Источник данных робота.

    ``master``: четыре колеса по одному USB каждое (см. firmware/motor_wheel).
    Порты FL/FR/RL/RR задаются только вручную (RC_WHEEL_*_PORT).
    Порты открывает только владелец (``master_role="owner"`` — бортовой экран);
    пульт (``"peer"``) читает телеметрию с экрана и порты не трогает.
    ``auto``: ROS 2, если есть rclpy; иначе UART-модули, если есть pyserial и
    живой порт; иначе пустой источник (без выдуманных данных).
    """
    mod = load_backend()
    mode = (mode or "auto").strip().lower()

    if mode == "none":
        # Робот не подключён: значения пустые, демо-физики нет.
        return mod.NoSource()
    if mode == "serial":
        # Явно заданные RC_SLAM_PORTS — воля оператора: их лидар и так не
        # трогает (они попадают в excluded_ports()). Если список пуст, ищем
        # сами — но порт лидара не занимаем (см. chassis_ports).
        chosen = list(ports) if ports else (_autodetect_ports() or chassis_ports(DEFAULT_PORTS))
        if not chosen:
            log.info("шасси: живых UART-портов нет (порт лидара не в счёт) — значений не будет")
        return chassis_source(mod, chosen)
    if mode == "ros":
        return mod.RosSource()
    if mode == "master" and master_role == "peer":
        return mod.MasterPeerSource(peer_kiosk_url())
    if mode == "master":
        from .master_port import reserved_ports
        from .wheel_settings import current_settings

        assignments = {m: p for m, p in current_settings().items() if p}
        blocked = reserved_ports()
        for module, path in list(assignments.items()):
            if path in blocked or os.path.realpath(path) in blocked:
                log.error("Колесо %s: порт %s занят лидаром/BMS — отключено", module, path)
                assignments.pop(module)
        if not assignments:
            log.warning("Порты колёс не назначены: выберите их на бортовом экране → Тяга и руление")
        return mod.MasterSource(
            module_ports=assignments,
            on_message=note_master_message,
            on_status=note_master_status,
            claim=try_lock_port,
            release=unlock_port,
            yield_to_lidar=lidar_holds_port,
            tag_module=tag_module_port,
            untag_module=untag_module_port,
        )

    if mode != "auto":
        log.warning("неизвестный RC_SLAM_SOURCE=%r — использую auto", mode)

    if _have("rclpy"):
        log.info("источник данных: ROS 2")
        return mod.RosSource()

    live = _present_ports(list(ports or [])) or _autodetect_ports()
    if _have("serial") and live:
        log.info("источник данных: UART-модули %s", ", ".join(live))
        return chassis_source(mod, live)

    log.info("источник данных: робот не подключён — значения пустые (демо-режима нет)")
    return mod.NoSource()


def build_state(mode="auto", ports=None, lock_file=None, pin=None,
                max_attempts=None, lock_sec=None, master_role="owner"):
    """Собирает ``App`` из backend.py: источник данных + PIN-хранилище."""
    mod = load_backend()
    source = make_source(mode, ports, master_role=master_role)
    path = str(lock_file) if lock_file else str(GUI_DIR / "state" / "lock.json")
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    vault = mod.Vault(
        path,
        pin or mod.DEFAULT_PIN,
        max_attempts=max_attempts or mod.MAX_ATTEMPTS,
        lock_ms=int((lock_sec if lock_sec is not None else mod.LOCK_MS / 1000.0) * 1000),
    )
    return mod.App(source, vault)


def _send(name: str):
    response = send_from_directory(GUI_DIR, name)
    response.headers["Cache-Control"] = "no-store"
    return response


def lidar_for_screen(manager, snap):
    """Точки лидара для экрана. Демо-скана нет: только реальные данные менеджера.

    Точки приходят из USB-порта (RPLIDAR, LD06/LD19, LDS-01/LDS-02), ROS 2
    ``/scan``, HTTP ``POST /api/lidar`` или соседнего процесса робота. Без них
    возвращается ``None`` — экран покажет «нет данных».
    Второе значение (признак демо) всегда ``False``; оно сохранено для формата ответа.
    """
    del snap  # раньше сюда подставлялся выдуманный скан; теперь не используется
    scan = manager.get_scan()
    if scan:
        return scan, False
    return None, False


def lidar_status_payload(manager, points, demo):
    """Единый статус лидара для ``/api/lidar``, ``/api/state`` и киоска.

    Одинаковый во всех ответах: соседний процесс робота берёт по нему скан, а
    экран — причину «скана нет». Флаг ``demo`` всегда ``False``: выдуманного
    скана в системе нет.
    """
    status = manager.get_status()
    status["demo"] = bool(demo)
    if demo and not status["connected"]:
        status["reason"] = "демо-скан источника данных (реального лидара нет): " + status["reason"]
    status["scan"] = points
    status["count"] = len(points) if points else 0
    # Экранный журнал: смена состояния лидара — настоящее событие робота.
    note_lidar_status(status)
    return status


def kiosk_log_payload(limit=None, *, state=None) -> dict:
    """Настоящий журнал событий для экрана: записи, счётчики, время работы."""
    source = None
    if state is not None:
        source = getattr(getattr(state, "source", None), "name", None)
    uptime = None
    started = getattr(state, "started", None)
    if isinstance(started, (int, float)):
        uptime = max(0, int(time.time() - started))
    return eventlog.payload(limit, uptime_sec=uptime, source=source)


def start_lidar(*, testing: bool = False, port: int | None = None, force: bool = False):
    """Запускает чтение лидара при старте веб-приложения.

    Без этого менеджер создавался только по первому запросу к API: до открытия
    страницы USB-порт мог занять источник данных шасси, а пульт и бортовой
    экран начинали читать лидар в разное время. ``.env`` к этому моменту уже
    прочитан (``Config.from_env``), поэтому настройки лидара берутся из него,
    а в журнал пишется, каким интерпретатором запущен процесс и что он видит.

    В тестах не запускается (``PYTEST_CURRENT_TEST``/``testing``): поток
    перебора портов там не нужен. ``RC_LIDAR_AUTOSTART=0`` отключает ранний
    старт и на роботе.
    """
    if (os.environ.get("RC_LIDAR_AUTOSTART") or "1").strip().lower() in {
        "0", "false", "no", "off", "нет",
    }:
        return None
    if not force and (testing or os.environ.get("PYTEST_CURRENT_TEST")):
        return None
    if port is not None:
        set_self_port(port)
    manager = LidarManager.get_instance()
    for line in startup_summary():
        log.info("Лидар: %s", line)
    return manager


def create_blueprint(state, guard=None, *, csrf_header_ok=None,
                     csrf_token_provider=None) -> Blueprint:
    """Маршруты интерфейса RUS SLAM.

    ``guard`` — декоратор авторизации пульта; без него маршруты открыты
    (используется в тестах самого интерфейса). Команды навигации в веб-пульте
    дополнительно проверяют CSRF-токен, прежде чем уйти на бортовой экран.
    """
    bp = Blueprint("slam", __name__)
    guard = guard or (lambda view: view)

    # ------------------------------ страницы ----------------------------- #
    @bp.get("/", endpoint="main")
    @guard
    def main_screen():
        """Основной экран робота."""
        return _send(MAIN_PAGE)

    bp.add_url_rule("/main.html", endpoint="main_page", view_func=guard(main_screen))

    def remote_screen(view):
        # Reuse the onboard controls, including hold-to-drive and loss-of-focus
        # braking. No second copy of motor limits, presets or USB ownership.
        html = (GUI_DIR / KIOSK_PAGE).read_text(encoding="utf-8")
        title = 'АКБ · веб-пульт' if view == 'battery' else 'Руление и тяга · веб-пульт'
        html = html.replace('<title>Экран робота</title>', f'<title>{title}</title>')
        html = html.replace('<body class="kiosk-body">',
                            f'<body class="kiosk-body" data-remote-view="{view}">')
        html = html.replace('<header class="swipe-head">',
            '<header class="swipe-head"><a class="swipe-close" href="/">Пульт</a>'
            '<a class="swipe-close" href="/controls">Руление и тяга</a>'
            '<a class="swipe-close" href="/battery" id="remote-battery-link">АКБ</a>')
        # Remote battery view is strictly read-only. The onboard process owns
        # the UART and its connection settings.
        html = html.replace('id="bms-setup-open"', 'id="bms-setup-open" hidden')
        html = html.replace('<h3>Аккумулятор · JBD BMS</h3>',
            '<h3>АКБ · полная информация JBD BMS</h3>'
            '<p>Данные с борта · только чтение. Подключение BMS настраивается на бортовом экране.</p>')
        html = html.replace('<h3>Тяга и руление</h3>',
            '<h3>Руление и тяга · веб-пульт</h3><p><a href="/">Пульт</a> · '
            '<a href="/console">Сервис</a> · <a href="/map">Карта</a> · '
            '<a href="/exit">Выход</a></p>'
            '<p><button id="remote-estop" class="traction-stop" type="button">E-STOP</button> '
            '<button id="remote-estop-release" type="button">Снять E-STOP по PIN</button></p>'
            '<p id="remote-control-message" role="status"></p>')
        html = html.replace('<script src="robot.js"></script>',
            '<script src="/slam_auth.js"></script><script src="/web_controls.js"></script>'
            '<script src="robot.js"></script>')
        return Response(html, mimetype="text/html", headers={"Cache-Control": "no-store"})

    @bp.get("/controls")
    @guard
    def remote_controls_screen():
        return remote_screen("motors")

    @bp.get("/connections")
    @guard
    def connections_screen():
        return _send("connections.html")

    @bp.get("/api/connections")
    @guard
    def connections_snapshot():
        from .map_peer import proxy_connections
        return proxy_connections(peer_kiosk_url().removesuffix("/api/kiosk_state"))

    @bp.get("/battery")
    @guard
    def remote_battery_screen():
        return remote_screen("battery")

    @bp.get("/api/control/battery/report.csv")
    @guard
    def remote_battery_report():
        from .map_peer import proxy_battery_report
        return proxy_battery_report(peer_kiosk_url().removesuffix("/api/kiosk_state"))

    @bp.get("/api/control/session")
    @guard
    def remote_control_session():
        return jsonify(ok=True, csrfToken=csrf_token_provider() if csrf_token_provider else "")

    @bp.route("/api/control/<path:operation>", methods=["GET", "POST"])
    @guard
    def remote_control_command(operation):
        from .map_peer import CONTROL_ROUTES, proxy_control
        route = CONTROL_ROUTES.get(operation)
        if route is None:
            return jsonify(ok=False, error="Неизвестная команда управления"), 404
        if request.method not in route[1]:
            return jsonify(ok=False, error="Метод не поддерживается"), 405
        if request.method == "POST":
            if request.headers.get(STEERING_COMMAND_HEADER) != route[2]:
                return jsonify(ok=False, reason="request_header_required"), 400
            if csrf_header_ok is not None and not csrf_header_ok():
                return jsonify(ok=False, reason="csrf_required"), 400
        return proxy_control(peer_kiosk_url().removesuffix("/api/kiosk_state"), operation)

    @bp.get("/console", endpoint="console")
    @guard
    def console():
        """Инженерный пульт."""
        return _send(CONSOLE_PAGE)

    bp.add_url_rule("/index.html", endpoint="console_page", view_func=guard(console))

    @bp.get("/map")
    @guard
    def map_screen():
        return _send("map.html")

    @bp.get("/route")
    @guard
    def route_screen():
        return _send("route.html")

    # The onboard process owns the map. The web process must never build a
    # second grid or race it when saving the same files.
    from .map_peer import proxy_map, proxy_navigation

    for suffix, methods in (("", ["GET"]), ("/settings", ["GET", "POST"]),
                            ("/reset", ["POST"]), ("/save", ["POST"])):
        def map_view(suffix=suffix):
            return proxy_map(peer_kiosk_url().removesuffix("/api/kiosk_state"), suffix)
        bp.add_url_rule("/api/kiosk_map" + suffix,
                        endpoint="map_peer" + suffix.replace("/", "_"),
                        view_func=guard(map_view), methods=methods)

    def navigation_proxy(suffix):
        return proxy_navigation(peer_kiosk_url().removesuffix("/api/kiosk_state"), suffix)

    @bp.get("/api/navigation/status")
    @guard
    def api_navigation_status():
        response = navigation_proxy("/status")
        if isinstance(response, tuple):
            response_obj, status_code = response[0], response[1]
        else:
            response_obj, status_code = response, response.status_code
        if csrf_token_provider and getattr(response_obj, "is_json", False):
            payload = response_obj.get_json(silent=True)
            if isinstance(payload, dict):
                payload["csrfToken"] = csrf_token_provider()
                return jsonify(payload), status_code, {"Cache-Control": "no-store"}
        return response

    @bp.post("/api/navigation/plan")
    @guard
    def api_navigation_plan():
        return navigation_proxy("/plan")

    def navigation_control_allowed():
        if request.headers.get("X-Robot-Control") != "navigation":
            return False
        return csrf_header_ok is not None and bool(csrf_header_ok())

    @bp.post("/api/navigation/start")
    @guard
    def api_navigation_start():
        if not navigation_control_allowed():
            return jsonify(ok=False, error="Недействительный CSRF-токен или заголовок управления."), 400
        return navigation_proxy("/start")

    @bp.post("/api/navigation/cancel")
    @guard
    def api_navigation_cancel():
        if not navigation_control_allowed():
            return jsonify(ok=False, error="Недействительный CSRF-токен или заголовок управления."), 400
        return navigation_proxy("/cancel")

    @bp.post("/api/navigation/estop")
    @guard
    def api_navigation_estop():
        if not navigation_control_allowed():
            return jsonify(ok=False, error="Недействительный CSRF-токен или заголовок управления."), 400
        return navigation_proxy("/estop")

    # --------- экран киоска на борту робота: без авторизации -------- #
    # Открывается локально в киоск-браузере на роботе; телеметрия публична,
    # а руление/тяга открываются отдельной короткой PIN-сессией.
    @bp.get(KIOSK_URL_PATH, endpoint="kiosk")
    def kiosk_screen():
        """Экран, который показывается на дисплее самого робота."""
        return _send(KIOSK_PAGE)

    bp.add_url_rule("/robot.html", endpoint="kiosk_page", view_func=kiosk_screen)

    @bp.get("/api/kiosk_state")
    def api_kiosk_state():
        """Публичная телеметрия и признак, поддерживает ли источник адресный поворот."""
        manager = LidarManager.get_instance()
        snap = state.snapshot()
        points, demo = lidar_for_screen(manager, snap)
        public = {
            "battery": snap.get("battery"),
            "motors": snap.get("motors"),
            "lidar": points,
            # Демо (стенд без железа) помечено: экран пишет «демо», а не
            # «в реальном времени».
            "lidarDemo": demo,
            # Причина «скана нет» нужна и на этом экране: robot.js берёт её
            # отсюда, а раньше в ответе её не было вовсе.
            "lidarStatus": lidar_status_payload(manager, points, demo),
            "speedMps": snap.get("speedMps"),
            "powerKw": snap.get("powerKw"),
            "mode": snap.get("mode"),
            "linkOk": snap.get("linkOk"),
            "ts": snap.get("ts"),
            "source": snap.get("source"),
            "steeringControl": steering_control_info(state.source),
            "drive": snap.get("drive"),
            "masterPort": snap.get("masterPort"),
            "masterMessages": snap.get("masterMessages") or [],
            "estop": (snap.get("estop") if getattr(state.source, "name", "") == "master-peer"
                      else estop_latch.latched()),
        }
        return jsonify({"ok": True, "data": public})

    @bp.get("/api/battery")
    def api_battery():
        from .battery import kiosk_snapshot
        snap = kiosk_snapshot()
        # Порт BMS держит бортовой экран: настраивать подключение можно только там.
        snap["setupAvailable"] = False
        return jsonify({"ok": True, "battery": snap}), 200, {"Cache-Control": "no-store"}

    @bp.get("/api/battery/setup")
    def api_battery_setup():
        return jsonify({"ok": True, "available": False, "ports": [],
                        "reason": "Подключение BMS настраивается на бортовом экране (порт 8090): "
                                  "страница «АКБ» → «Подключение BMS»."}), 200, {"Cache-Control": "no-store"}

    @bp.get("/api/battery/report.csv")
    def api_battery_report():
        from .battery import kiosk_snapshot, report_response
        return report_response(kiosk_snapshot())

    # PIN-оверлей киоска: телеметрия публична, команды руления и тяги доступны
    # только после проверки PIN и в короткоживущей серверной сессии.
    @bp.get("/api/kiosk_lock")
    def api_kiosk_lock():
        return jsonify({"ok": True, "lock": state.vault.state(state.lock_open),
                        "steeringAuthorized": steering_access_valid()})

    @bp.post("/api/kiosk_lock/verify")
    def api_kiosk_lock_verify():
        body = request.get_json(silent=True) or {}
        result = state.vault.verify(str(body.get("pin", "")))
        note_pin_result(result, where="бортовой экран")
        if result.get("ok"):
            grant_steering_access()
        else:
            revoke_steering_access()
        result["lock"] = state.vault.state(state.lock_open)
        result["steeringAuthorized"] = steering_access_valid() if result.get("ok") else False
        return jsonify(result), 200 if result.get("ok") else 403

    @bp.post("/api/kiosk_lock/session/end")
    def api_kiosk_steering_logout():
        if steering_access_valid():
            eventlog.record("info", "PIN-сессия руления закрыта оператором", source="руление")
        revoke_steering_access()
        return jsonify({"ok": True, "steeringAuthorized": False})

    @bp.post("/api/kiosk_lock/open")
    def api_kiosk_lock_open():
        body = request.get_json(silent=True) or {}
        result = state.open_lock(str(body.get("pin", "")))
        return jsonify(result), 200 if result.get("ok") else 403

    @bp.post("/api/kiosk_lock/close")
    def api_kiosk_lock_close():
        return jsonify(state.close_lock())

    @bp.post("/api/kiosk_steer")
    def api_kiosk_steer():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            note_pin_required("руление")
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_steering_command(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @bp.post("/api/kiosk_zero")
    def api_kiosk_zero():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            note_pin_required("программный ноль")
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_steering_zero(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @bp.post("/api/kiosk_pot_mode")
    def api_kiosk_pot_mode():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_pot_mode(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @bp.post("/api/kiosk_calibrate")
    def api_kiosk_calibrate():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            note_pin_required("калибровка руля")
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_steering_calibration(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @bp.post("/api/estop")
    @guard
    def api_estop():
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if csrf_header_ok is not None and not csrf_header_ok():
            return jsonify({"ok": False, "reason": "csrf_required"}), 400
        if getattr(state.source, "name", "") == "master-peer":
            from .map_peer import proxy_estop
            return proxy_estop(peer_kiosk_url().removesuffix("/api/kiosk_state"))
        return jsonify(engage_estop(state.source))

    @bp.post("/api/estop/release")
    @guard
    def api_estop_release():
        """Снять E-STOP: PIN в теле запроса — та же проверка, что у замка."""
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if csrf_header_ok is not None and not csrf_header_ok():
            return jsonify({"ok": False, "reason": "csrf_required"}), 400
        if getattr(state.source, "name", "") == "master-peer":
            from .map_peer import proxy_estop
            return proxy_estop(peer_kiosk_url().removesuffix("/api/kiosk_state"), release=True)
        body = request.get_json(silent=True) or {}
        result = state.vault.verify(str(body.get("pin", "")))
        note_pin_result(result, where="снятие E-STOP")
        if not result.get("ok"):
            return jsonify({"ok": False, "reason": result.get("reason", "pin")}), 403
        return jsonify(release_estop())

    @bp.post("/api/kiosk_drive")
    def api_kiosk_drive():
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            note_pin_required("тяга")
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_drive_command(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @bp.post("/api/kiosk_drive_config")
    def api_kiosk_drive_config():
        """Настройка привода тяги: выход (ШИМ / 0–5 В), тахометр, тормоз, предел."""
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            note_pin_required("настройка привода")
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_drive_config(state.source, request.get_json(silent=True))
        return jsonify(result), status

    # -------------------------------- API -------------------------------- #
    @bp.get("/api/state")
    @guard
    def api_state():
        snap = state.snapshot()
        manager = LidarManager.get_instance()
        points, demo = lidar_for_screen(manager, snap)
        snap["lidar"] = points
        snap["lidarDemo"] = demo
        # Причина, если скана нет: экран пишет её под картой (порт лидара не
        # найден, не та скорость, порт занят вторым процессом робота и т. д.).
        snap["lidarStatus"] = lidar_status_payload(manager, points, demo)
        snap["cameras"] = CameraManager.get_instance().list_cameras()
        if getattr(state.source, "name", "") != "master-peer":
            snap["estop"] = estop_latch.latched()
        else:
            snap.setdefault("estop", None)  # read from the onboard kiosk peer
        if csrf_token_provider:
            snap["csrfToken"] = csrf_token_provider()
        return jsonify({"ok": True, "data": snap})

    @bp.post("/api/drive")
    @guard
    def api_drive():
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_drive_command(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @bp.post("/api/drive_config")
    @guard
    def api_drive_config():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_drive_config(state.source, request.get_json(silent=True))
        return jsonify(result), status

    # ----------------------------- ЛИДАР --------------------------------- #
    @bp.get("/api/lidar")
    @bp.get("/api/scan")
    def api_lidar_get():
        """Скан и статус лидара.

        Ответ открыт без авторизации намеренно: по нему второй процесс робота
        (пульт 8080 ↔ бортовой экран 8090) берёт скан, когда USB-порт занят
        соседом. Демо-точек в системе нет, флаг ``demo`` всегда ``False``.
        """
        mgr = LidarManager.get_instance()
        scan = mgr.get_scan()
        demo = False
        if scan is None and hasattr(state, "source"):
            try:
                src_data = getattr(state.source, "read", lambda: {})()
                scan = src_data.get("lidar")  # реальный скан источника; демо-режима нет
            except Exception:  # noqa: BLE001 — источник может не уметь read()
                scan = None
        return jsonify(lidar_status_payload(mgr, scan, demo))

    @bp.post("/api/lidar")
    @bp.post("/api/scan")
    def api_lidar_post():
        body = request.get_json(silent=True) or {}
        mgr = LidarManager.get_instance()
        if "points" in body or "scan" in body:
            pts = body.get("points") or body.get("scan") or []
            count = mgr.feed_points(pts, source_label="api")
            return jsonify({"ok": True, "count": count})
        elif "ranges" in body:
            count = mgr.feed_scan_msg(
                ranges=body.get("ranges", []),
                angle_min=float(body.get("angle_min", -math.pi)),
                angle_max=float(body.get("angle_max", math.pi)),
                angle_increment=float(body.get("angle_increment", 0.0174)),
                range_min=float(body.get("range_min", 0.05)),
                range_max=float(body.get("range_max", 12.0)),
                source_label="api_ranges",
            )
            return jsonify({"ok": True, "count": count})
        return jsonify({"ok": False, "error": "expected 'points' or 'ranges'"}), 400

    # ---------------------------- КАМЕРЫ --------------------------------- #
    @bp.get("/api/cameras")
    def api_cameras():
        mgr = CameraManager.get_instance()
        return jsonify({"ok": True, "cameras": mgr.list_cameras()})

    @bp.get("/api/camera/<int:cam_id>/stream")
    def api_camera_stream(cam_id: int):
        mgr = CameraManager.get_instance()
        return Response(
            mgr.generate_mjpeg_stream(cam_id),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @bp.get("/api/camera/front/stream")
    def api_camera_front_stream():
        return api_camera_stream(0)

    @bp.get("/api/camera/rear/stream")
    def api_camera_rear_stream():
        return api_camera_stream(1)

    @bp.get("/api/camera/<int:cam_id>/frame")
    def api_camera_frame(cam_id: int):
        mgr = CameraManager.get_instance()
        frame = mgr.get_frame(cam_id)
        if frame:
            return Response(frame, mimetype="image/jpeg")
        return Response(status=404)

    @bp.get("/api/camera/front/frame")
    def api_camera_front_frame():
        return api_camera_frame(0)

    @bp.get("/api/camera/rear/frame")
    def api_camera_rear_frame():
        return api_camera_frame(1)

    @bp.post("/api/camera/<int:cam_id>/frame")
    def api_camera_feed(cam_id: int):
        mgr = CameraManager.get_instance()
        data = request.get_data()
        if data:
            mgr.feed_frame(cam_id, data)
            return jsonify({"ok": True})
        return jsonify({"ok": False, "error": "empty data"}), 400

    @bp.get("/api/wheels/settings")
    @guard
    def api_wheel_settings():
        from .wheel_settings import settings_payload
        result = settings_payload()
        result["canEdit"] = False
        return jsonify(result)

    @bp.get("/api/camera/settings")
    @guard
    def api_camera_settings():
        from .camera_settings import settings_payload as cam_settings_payload

        result = cam_settings_payload()
        # На пульте (8080) настройки видны, но менять порт камер лучше на борту (8090)
        result["canEdit"] = True
        # Совместимость: фронтенд проверяет requiresPin для старых клиентов
        result["requiresPin"] = False
        result["hint"] = "Порт камер меняется на бортовом экране (8090): свайп «Камеры» → «Настройка камер». Здесь — просмотр и резервное сохранение."
        return jsonify(result)

    @bp.post("/api/camera/settings")
    @guard
    def api_camera_settings_save():
        from .camera_settings import apply_camera_settings

        body = request.get_json(silent=True) or {}
        try:
            result = apply_camera_settings(body.get("settings") or body)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        except OSError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500
        eventlog.record("ok", "камеры (пульт): передняя %s, задняя %s — сохранено" % (
            result["settings"].get("front") or "авто",
            result["settings"].get("rear") or "авто"), source="камеры")
        return jsonify(result)

    @bp.post("/api/camera/rescan")
    @guard
    def api_camera_rescan():
        from .camera_settings import describe_camera_ports
        from .camera import CameraManager

        mgr = CameraManager.get_instance()
        ports = describe_camera_ports()
        return jsonify({"ok": True, "ports": ports, "cameras": mgr.list_cameras()})

    @bp.get("/api/kiosk_logs")
    @guard
    def api_kiosk_logs():
        """Настоящий журнал событий робота — то, что показывает панель «Логи».

        Тот же журнал, что и на бортовом экране (8090): записи логирования
        ``robot_control.*`` и явные события маршрутов. На пульте он закрыт
        входом оператора — в отличие от киоска, где панель публична, как и
        вся телеметрия борта.
        """
        limit = eventlog.clamp_limit(request.args.get("limit"))
        return jsonify({"ok": True, "data": kiosk_log_payload(limit, state=state)})

    @bp.get("/api/audit")
    @guard
    def api_audit():
        raw = request.args.get("limit", "20") or ""
        match = re.search(r"\d+", raw)
        limit = max(1, min(200, int(match.group()))) if match else 20
        return jsonify(
            {
                "ok": True,
                "audit": state.vault.audit(limit),
                "integrity": state.vault.verify_audit(),
            }
        )

    @bp.get("/api/health")
    @guard
    def api_health():
        return jsonify(
            {
                "ok": True,
                "source": getattr(state.source, "name", "?"),
                "uptimeSec": int(time.time() - state.started),
                "pinDefault": state.vault.default_pin,
            }
        )

    @bp.post("/api/lock/open")
    @guard
    def api_lock_open():
        body = request.get_json(silent=True) or {}
        result = state.open_lock(str(body.get("pin", "")))
        return jsonify(result), 200 if result.get("ok") else 403

    @bp.post("/api/lock/close")
    @guard
    def api_lock_close():
        return jsonify(state.close_lock())

    @bp.post("/api/lock/pin")
    @guard
    def api_lock_pin():
        body = request.get_json(silent=True) or {}
        result = state.vault.set_pin(str(body.get("current", "")), str(body.get("new", "")))
        if result.get("ok"):
            state.close_lock()
        return jsonify(result), 200 if result.get("ok") else 403

    # ------------------------------- статика ------------------------------ #
    # Без авторизации намеренно: в средах без cookie (iframe-предпросмотр)
    # подзапросы не могут передать сессию, и страница приезжала сломанной.
    # Секретов в CSS/JS нет; данные и страницы по-прежнему закрыты.
    for filename, endpoint in ASSETS.items():
        bp.add_url_rule(
            f"/{filename}",
            endpoint=endpoint,
            view_func=lambda name=filename: _send(name),
        )

    return bp
