"""Onboard USB-UART receipt printer, with no automatic probing or raw-data API.

The USB adapter and its socket are not proof that the connected device is a
printer. The operator explicitly identifies it; in master mode the wheel owner
must first passively check the same unchanged UART. No such hint relaxes the
wheel discovery interlock. Actual printing takes the shared USB lock and is
performed in a single bounded background job; only application-generated text
and the explicitly selected harmless ESC/POS initialisation are sent.
"""
from __future__ import annotations

import copy
import glob
import json
import logging
import os
import re
import stat
import tempfile
import threading
import time
from pathlib import Path

from .battery_receipt import CHARSETS, LANGUAGES, LEVELS, encode_receipt, render_receipt
from .config import PROJECT_ROOT
from .device_registry import SERIAL_RE, UsbTopology, _find_port, _identity, _resolve_identity

log = logging.getLogger(__name__)
DEFAULTS = {"port": "", "identity": None, "baudRate": 9600, "framing": "8N1", "width": 32,
            "profile": "plain", "language": "en", "charset": "cp866", "lineDelayMs": 800}
BAUDS = (9600, 19200, 38400, 57600, 115200)
FRAMINGS = ("8N1", "8E1", "7E1")
PROFILES = ("plain", "escpos")
REQUEST_ID = re.compile(r"^[a-zA-Z0-9_-]{16,80}$")


class PrinterError(ValueError):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


def config_path():
    return Path(os.environ.get("RC_PRINTER_CONFIG") or PROJECT_ROOT / "data" / "printer.json")


def _read_saved(path=None):
    path = Path(path or config_path())
    try:
        if not path.is_file():
            return None
        if path.stat().st_size > 16384:
            raise ValueError("слишком большой файл")
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict) or value.get("version") != 1 or not isinstance(value.get("settings"), dict):
            raise ValueError("неизвестный формат")
        return value["settings"]
    except (OSError, ValueError, TypeError, UnicodeError) as exc:
        log.warning("Принтер: настройки не прочитаны: %s", exc)
        return None


def printer_reserved_ports(path=None):
    """Exclude a configured printer from BMS/lidar probes, never authorize it.

    The saved name is conservatively reserved even when it has been reused;
    an identity match also protects a renumbered adapter. A changed socket or
    cloned adapter cannot be proven; the operator must physically re-check it.
    """
    settings = _read_saved(path)
    if not settings:
        return set()
    port = settings.get("port")
    if not isinstance(port, str) or not SERIAL_RE.fullmatch(port):
        return set()
    result = {port, os.path.realpath(port)}
    try:
        topology = UsbTopology().collect()
        device, node, _ = _resolve_identity(topology, settings.get("identity"))
        if device and node and node.get("kind") == "serial":
            current = node.get("path")
            if isinstance(current, str) and SERIAL_RE.fullmatch(current):
                result.update((current, os.path.realpath(current)))
                for alias in node.get("aliases") or []:
                    if isinstance(alias, str) and SERIAL_RE.fullmatch(alias):
                        result.update((alias, os.path.realpath(alias)))
    except Exception:  # noqa: BLE001 — never use a bad hint as permission
        log.exception("Принтер: не удалось сверить сохранённый USB-адаптер")
    return result


def _fingerprint(path):
    try:
        st = os.stat(path)
        if not stat.S_ISCHR(st.st_mode):
            return None
        return (st.st_dev, st.st_ino, st.st_rdev, st.st_ctime_ns)
    except OSError:
        return None


def _save(path, settings):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".printer-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as out:
            json.dump({"version": 1, "settings": settings}, out, ensure_ascii=False, sort_keys=True)
            out.write("\n")
            out.flush()
            os.fsync(out.fileno())
        os.chmod(tmp, 0o600)
        os.replace(tmp, path)
    finally:
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass


def _valid_settings(settings):
    if not isinstance(settings, dict) or set(settings) != set(DEFAULTS):
        return False
    port, identity = settings["port"], settings["identity"]
    return (isinstance(port, str) and (not port or bool(SERIAL_RE.fullmatch(port))) and
            (identity is None if not port else isinstance(identity, dict)) and
            type(settings["baudRate"]) is int and settings["baudRate"] in BAUDS and
            settings["framing"] in FRAMINGS and type(settings["width"]) is int and 24 <= settings["width"] <= 48 and
            settings["profile"] in PROFILES and settings["language"] in LANGUAGES and settings["charset"] in CHARSETS and
            type(settings["lineDelayMs"]) is int and 0 <= settings["lineDelayMs"] <= 2000)


class PrinterService:
    def __init__(self, registry, battery, source_provider, *, path=None, serial_factory=None, clock=time.time):
        self.registry, self.battery, self.source_provider = registry, battery, source_provider
        self.path = Path(path or config_path())
        self.serial_factory = serial_factory
        self.clock = clock
        self._lock = threading.RLock()
        self._job_lock = threading.Lock()
        self._cancel = threading.Event()
        self._confirmed = None  # never persisted across server restart / device replacement
        self._jobs = {}
        self._job = None
        self._settings = copy.deepcopy(DEFAULTS)
        saved = _read_saved(self.path)
        if saved and _valid_settings(saved):
            self._settings = saved
        elif saved:
            log.warning("Принтер: сохранённые настройки неверны, печать отключена")

    def settings(self):
        with self._lock:
            return copy.deepcopy(self._settings)

    def _resolve(self, settings, *, refresh=True):
        if not settings.get("port"):
            raise PrinterError("Выберите отдельный USB-UART принтера в настройках.")
        topology = self.registry.usb(refresh=refresh)
        if not topology.get("available"):
            raise PrinterError("USB-топология недоступна; порт принтера не подтверждён.")
        device, node, confidence = _resolve_identity(topology, settings.get("identity"))
        if not device or not node or node.get("kind") != "serial":
            raise PrinterError("Сохранённый USB-адаптер не найден или изменился. Проверьте кабель и выберите порт заново.")
        port = node.get("path")
        if not isinstance(port, str) or not SERIAL_RE.fullmatch(port):
            raise PrinterError("Не найден разрешённый последовательный порт USB-адаптера.")
        fingerprint = _fingerprint(port)
        if fingerprint is None:
            raise PrinterError("USB-UART недоступен или не является символьным устройством.")
        return port, fingerprint, device, confidence

    def _blocked(self, port):
        from . import bms_setup
        from .battery import reserved_ports as bms_ports
        from .lidar import excluded_ports, lidar_active_port
        real = os.path.realpath(port)
        blocked = set(bms_ports()) | bms_setup.master_ports() | bms_setup._lidar_ports() | bms_setup._chassis_ports()
        blocked |= excluded_ports()
        if getattr(self.battery, "port", ""):
            blocked.add(self.battery.port)
        active = lidar_active_port()
        if active:
            blocked.add(active)
        # Explicit lidar settings are exclusions, never permission to print.
        for part in (os.environ.get("RC_LIDAR_PORT") or "").replace(";", ",").split(","):
            part = part.strip()
            if part:
                blocked.update(glob.glob(part) or [part])
        return real in {os.path.realpath(str(p)) for p in blocked if p}

    def _assert_safe(self, port, fingerprint, source, *, passive=True):
        if _fingerprint(port) != fingerprint:
            raise PrinterError("USB-порт изменился; печать остановлена.")
        if self._blocked(port):
            raise PrinterError("Порт занят или зарезервирован BMS, колесом либо лидаром.")
        if source is None or source is not self.source_provider() or not getattr(source, "auto_discovery", False):
            raise PrinterError("Нужна проверка USB-портов бортовым владельцем колёс (режим master).")
        if passive:
            try:
                allowed = source.manual_printer_port_allowed(port)
            except Exception:  # noqa: BLE001 — no owner check means no printing
                allowed = False
            if not allowed:
                # Явное назначение порта принтера оператором на странице
                # «Устройства» — это его подтверждение физического подключения.
                try:
                    allowed = source.operator_assigned_port_allowed(port, "printer")
                except Exception:  # noqa: BLE001
                    allowed = False
            if not allowed:
                raise PrinterError("Порт ещё проверяется или опознан как колесо, BMS либо лидар. Дождитесь проверки USB.")

    def configure(self, body):
        expected = {"port", "baudRate", "framing", "width", "profile", "language", "charset", "lineDelayMs", "confirmed"}
        if not isinstance(body, dict) or set(body) != expected or type(body.get("confirmed")) is not bool:
            raise PrinterError("Неверные настройки принтера.", 400)
        if self._job_lock.locked():
            raise PrinterError("Дождитесь окончания печати перед изменением настроек.")
        settings = {key: body[key] for key in DEFAULTS if key != "identity"}
        settings["identity"] = None
        port = settings.get("port")
        if not isinstance(port, str) or (port and not SERIAL_RE.fullmatch(port)):
            raise PrinterError("Выберите USB-UART из списка: /dev/ttyUSB… или /dev/serial/by-id/…", 400)
        if port:
            if body["confirmed"] is not True:
                raise PrinterError("Физически проверьте и подтвердите USB-UART принтера.", 400)
            topology = self.registry.usb(refresh=True)
            device, node = _find_port(topology, port)
            if not topology.get("available") or not device or not node or node.get("kind") != "serial":
                raise PrinterError("Выбранный порт не найден среди текущих USB-UART адаптеров.")
            settings["identity"] = _identity(device, node)
        if not _valid_settings(settings):
            raise PrinterError("Проверьте скорость, кадр, ширину, протокол, язык и паузу.", 400)
        source = self.source_provider()
        confirmed = None
        lock_fd = None
        if port:
            selected, fingerprint, _, _ = self._resolve(settings)
            self._assert_safe(selected, fingerprint, source)
            from .lidar import try_lock_port
            lock_fd = try_lock_port(selected)
            if lock_fd in (None, -1):
                raise PrinterError("Порт занят или недоступен общий USB-замок. Попробуйте после проверки устройств.")
            try:
                self._assert_safe(selected, fingerprint, source)
                confirmed = (os.path.realpath(selected), fingerprint)
            except Exception:
                from .lidar import unlock_port
                unlock_port(lock_fd)
                raise
        try:
            with self._lock:
                if self._job_lock.locked():
                    raise PrinterError("Принтер уже печатает.")
                _save(self.path, settings)
                self._settings = settings
                self._confirmed = confirmed
            # This is only an inventory hint; it never grants permission.
            try:
                self.registry.set_hint("printer", port)
            except (OSError, ValueError, TypeError):
                log.exception("Принтер: не удалось сохранить подсказку в реестре")
        finally:
            if lock_fd not in (None, -1):
                from .lidar import unlock_port
                unlock_port(lock_fd)
        return self.status(include_ports=False)

    def status(self, *, include_ports=True):
        settings = self.settings()
        with self._lock:
            confirmed, job = self._confirmed, copy.deepcopy(self._job)
        ready, detail, current, adapter, confidence = False, "Принтер не настроен.", None, None, None
        if settings["port"]:
            try:
                port, fingerprint, device, confidence = self._resolve(settings, refresh=False)
                current = port
                adapter = {k: device.get(k) for k in ("product", "manufacturer", "vidPid", "serial", "topology", "connection")}
                if confirmed != (os.path.realpath(port), fingerprint):
                    detail = "Подтвердите физический порт принтера после запуска или переподключения."
                else:
                    self._assert_safe(port, fingerprint, self.source_provider())
                    ready = True
                    detail = "USB-адаптер совпал; модель, наличие бумаги и печать не подтверждены."
            except PrinterError as exc:
                detail = str(exc)
        result = {"ok": True, "settings": settings, "configured": bool(settings["port"]), "ready": ready,
                  "port": current, "adapter": adapter, "confidence": confidence, "detail": detail,
                  "printing": self._job_lock.locked(), "job": job}
        if include_ports:
            from .bms_setup import describe_ports
            result["ports"] = describe_ports()
        return result

    def _snapshot(self):
        snapshot = self.battery.snapshot()
        sample = snapshot.get("data") if isinstance(snapshot, dict) else None
        try:
            age = self.clock() - sample["updatedAt"]
            fresh = isinstance(age, (int, float)) and -2 <= age <= 5
        except (KeyError, TypeError, ValueError, OverflowError):
            fresh = False
        if not isinstance(snapshot, dict) or not snapshot.get("connected") or not isinstance(sample, dict) or not fresh:
            raise PrinterError("Нет свежих данных BMS. Подключите аккумулятор и дождитесь показаний.", 503)
        return snapshot

    def preview(self, level):
        settings = self.settings()
        if level not in LEVELS:
            raise PrinterError("Выберите короткий, средний или полный отчёт.", 400)
        lines = render_receipt(self._snapshot(), level, width=settings["width"], language=settings["language"])
        return {"ok": True, "level": level, "language": settings["language"], "width": settings["width"],
                "lines": lines, "lineCount": len(lines), "feedLines": 4}

    def _set_job(self, job_id, **values):
        with self._lock:
            job = self._jobs.get(job_id)
            if job is not None:
                job.update(values)
                if self._job and self._job.get("id") == job_id:
                    self._job = copy.deepcopy(job)

    def start(self, body, *, test=False):
        if not isinstance(body, dict) or set(body) != ({"requestId"} if test else {"requestId", "level"}):
            raise PrinterError("Неверный запрос печати.", 400)
        job_id = body.get("requestId")
        if not isinstance(job_id, str) or not REQUEST_ID.fullmatch(job_id):
            raise PrinterError("Неверный идентификатор печати.", 400)
        with self._lock:
            if job_id in self._jobs:
                return {"ok": True, "job": copy.deepcopy(self._jobs[job_id]), "replayed": True}
        level = body.get("level")
        if not test and level not in LEVELS:
            raise PrinterError("Выберите короткий, средний или полный отчёт.", 400)
        if not self._job_lock.acquire(blocking=False):
            raise PrinterError("Принтер уже занят. Подождите окончания печати.")
        try:
            status = self.status(include_ports=False)
            # status.printing is true because this request owns the job lock.
            if not status["ready"]:
                raise PrinterError(status["detail"])
            settings = self.settings()
            snapshot = None if test else self._snapshot()
            if test:
                lines = ["PRINTER TEST", f"UART {settings['baudRate']} {settings['framing']}",
                         "Hello 123 ABC abc +-*/", "-" * settings["width"]]
                if settings["language"] == "ru":
                    lines.append("Русский: АБВ абв 123")
            else:
                lines = render_receipt(snapshot, level, width=settings["width"], language=settings["language"])
            init, chunks = encode_receipt(lines, profile=settings["profile"],
                                          language=settings["language"], charset=settings["charset"])
            job = {"id": job_id, "kind": "test" if test else "battery", "level": level,
                   "state": "queued", "sentLines": 0, "totalLines": len(chunks),
                   "message": "Задание принято; проверяем порт.", "startedAt": self.clock(), "finishedAt": None}
            with self._lock:
                # The lock covers the check and insertion, including duplicate concurrent requests.
                if job_id in self._jobs:
                    # Another request with this id may have completed between
                    # the first lookup and our non-blocking lock acquisition.
                    replay = {"ok": True, "job": copy.deepcopy(self._jobs[job_id]), "replayed": True}
                    self._job_lock.release()
                    return replay
                self._cancel.clear()
                self._job = copy.deepcopy(job)
                self._jobs[job_id] = copy.deepcopy(job)
                while len(self._jobs) > 64:
                    self._jobs.pop(next(iter(self._jobs)))
            thread = threading.Thread(target=self._work, args=(job_id, settings, snapshot, init, chunks),
                                      name="receipt-printer", daemon=True)
            try:
                thread.start()
            except Exception:
                self._set_job(job_id, state="error", message="Не удалось запустить задание печати.",
                              finishedAt=self.clock())
                raise
            return {"ok": True, "job": copy.deepcopy(job), "replayed": False}
        except Exception:
            self._job_lock.release()
            raise

    def cancel(self):
        if self._job_lock.locked():
            self._cancel.set()
            return {"ok": True, "message": "Останавливаем отправку; уже переданные строки отозвать нельзя."}
        return {"ok": True, "message": "Активной печати нет."}

    def _open(self, port, settings):
        import serial
        factory = self.serial_factory or serial.Serial
        framing = settings["framing"]
        device = factory(port=None, baudrate=settings["baudRate"], bytesize=int(framing[0]),
                         parity=framing[1], stopbits=int(framing[2]), timeout=0.1, write_timeout=0.8,
                         xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
        device.dtr = False
        device.rts = False
        device.port = port
        device.open()
        return device

    def _wait_for_output(self, device, port, fingerprint, source, settings, byte_count):
        """Wait, cancellably, for the OS transmit queue to empty before close.

        ``Serial.flush()`` uses a potentially unbounded tcdrain on POSIX, so it
        cannot safely implement the UI's cancel operation on a stalled UART.
        An empty driver queue is still not confirmation that paper was printed.
        """
        deadline = time.monotonic() + min(90, max(3, byte_count * 12 / settings["baudRate"] + 5))
        while True:
            if self._cancel.is_set():
                raise PrinterError("Отменено оператором. Ранее отправленные строки могли напечататься.")
            self._assert_safe(port, fingerprint, source, passive=False)
            try:
                pending = device.out_waiting
            except Exception as exc:  # noqa: BLE001 — no unsafe blind success
                raise PrinterError("Не удалось проверить очередь USB-UART; результат печати неизвестен.") from exc
            if not isinstance(pending, int) or pending < 0:
                raise PrinterError("Неизвестное состояние очереди USB-UART.")
            if pending == 0:
                return
            if time.monotonic() >= deadline:
                raise PrinterError("Буфер USB-UART не опустел вовремя; результат печати неизвестен.")
            self._cancel.wait(0.05)

    def _work(self, job_id, settings, snapshot, init, chunks):
        from .lidar import try_lock_port, unlock_port
        port_lock, device, sent = None, None, 0
        write_attempted = False
        try:
            if self._cancel.is_set():
                raise PrinterError("Отменено до отправки.")
            port, fingerprint, _, _ = self._resolve(settings)
            source = self.source_provider()
            if self._confirmed != (os.path.realpath(port), fingerprint):
                raise PrinterError("USB-порт изменился. Подтвердите принтер заново.")
            self._assert_safe(port, fingerprint, source)
            port_lock = try_lock_port(port)
            if port_lock in (None, -1):
                raise PrinterError("Порт занят или недоступен общий USB-замок.")
            self._assert_safe(port, fingerprint, source)
            if snapshot is not None:
                # Don't start a queued job with an old measurement.
                age = self.clock() - snapshot["data"]["updatedAt"]
                if not -2 <= age <= 5:
                    raise PrinterError("Показания BMS устарели до начала печати.")
            device = self._open(port, settings)
            opened = os.fstat(device.fileno())
            if (fingerprint != (opened.st_dev, opened.st_ino, opened.st_rdev, opened.st_ctime_ns) or
                    _fingerprint(port) != fingerprint):
                raise PrinterError("USB-порт изменился во время открытия; байты не отправлены.")
            self._assert_safe(port, fingerprint, source)
            self._set_job(job_id, state="printing", message="Отправляем чек на принтер…")
            for chunk in ([init] if init else []) + chunks:
                if self._cancel.is_set():
                    raise PrinterError("Отменено оператором. Ранее отправленные строки могли напечататься.")
                # The lock prevents other compliant owners from opening this
                # device; check its identity and reservations again per line.
                self._assert_safe(port, fingerprint, source, passive=False)
                view = memoryview(chunk)
                while view:
                    # A timeout can occur after the driver accepted part of a
                    # chunk without reporting how many bytes reached the UART.
                    write_attempted = True
                    written = device.write(view)
                    if not isinstance(written, int) or written <= 0 or written > len(view):
                        raise OSError("неполная запись в USB-UART")
                    view = view[written:]
                if chunk is not init:
                    sent += 1
                    self._set_job(job_id, sentLines=sent)
                    if self._cancel.wait(settings["lineDelayMs"] / 1000):
                        raise PrinterError("Отменено оператором. Ранее отправленные строки могли напечататься.")
                elif self._cancel.wait(0.05):
                    raise PrinterError("Отменено оператором.")
            self._set_job(job_id, message="Ожидаем опустошения очереди USB-UART…")
            self._wait_for_output(device, port, fingerprint, source, settings,
                                  len(init) + sum(len(chunk) for chunk in chunks))
            self._set_job(job_id, state="sent", message="Данные переданы в драйвер USB-UART. Саму печать и бумагу нужно проверить на принтере.",
                          finishedAt=self.clock())
        except Exception as exc:  # noqa: BLE001 — never replay a partially sent receipt
            message = str(exc) or type(exc).__name__
            if write_attempted:
                message += " Часть чека могла напечататься; автоматического повтора нет."
            self._set_job(job_id, state="cancelled" if self._cancel.is_set() else "error",
                          message=message, finishedAt=self.clock())
            log.warning("Принтер: %s", message)
        finally:
            if device is not None:
                try:
                    device.close()
                except Exception:  # noqa: BLE001
                    log.exception("Принтер: не удалось закрыть USB-UART")
            unlock_port(port_lock)
            self._job_lock.release()
