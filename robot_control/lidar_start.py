"""One-shot recovery for an LDS-01 which is silent until its start command.

This is deliberately not auto-discovery. The operator must physically trace the
cable and confirm the model and current port every time. The board owner must
first have passively checked that unchanged UART for wheels; an operator hint,
USB VID/PID, saved path, silence or a ROS topic is never permission to write.
Only a checksummed LDS stream after the one-byte start command is published as
an identity. This cannot prove wiring or physical safety.
"""
from __future__ import annotations

import os
import secrets
import stat
import threading
import time

from .device_registry import SERIAL_RE, _find_port, _identity
from .lidar import LDS01_BAUD, LDS01_CMD_START, Lds01Stream


class LidarStartError(ValueError):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


def fingerprint(port):
    try:
        st = os.stat(port)
        if not stat.S_ISCHR(st.st_mode):
            return None
        return (st.st_dev, st.st_ino, st.st_rdev, st.st_ctime_ns)
    except OSError:
        return None


class LidarStartService:
    CONFIRM_SECONDS = 45.0
    READ_SECONDS = 2.8

    def __init__(self, registry, battery, source_provider, *, clock=time.monotonic):
        self.registry = registry
        self.battery = battery
        self.source_provider = source_provider
        self.clock = clock
        self._lock = threading.Lock()
        self._start_lock = threading.Lock()
        self._challenges = {}

    def _blocked(self, port):
        from .battery import reserved_ports as bms_ports
        from .lidar import (chassis_module_ports, checked_lidar_ports, excluded_ports,
                            lidar_active_port)
        from .printer import printer_reserved_ports
        paths = (set(bms_ports()) | set(chassis_module_ports()) | set(checked_lidar_ports()) |
                 set(excluded_ports()) | set(printer_reserved_ports()))
        paths.update((getattr(self.battery, "port", ""), lidar_active_port()))
        return os.path.realpath(port) in {os.path.realpath(str(p)) for p in paths if p}

    def _assert_safe(self, port, real, fp, source, identity, *, reserved=False):
        if os.environ.get("RC_USB_DISCOVERY_MODE") != "1" or source is None or source is not self.source_provider():
            raise LidarStartError("Нужен активный бортовой владелец USB колёс в режиме master.")
        if not getattr(source, "auto_discovery", False) or getattr(source, "_stop", True):
            raise LidarStartError("Проверка USB колёс не работает.")
        if os.path.realpath(port) != real or fingerprint(port) != fp or fp is None:
            raise LidarStartError("USB-порт изменился. Проверьте кабель и начните заново.")
        topology = self.registry.usb(refresh=True)
        device, node = _find_port(topology, port)
        if (not topology.get("available") or not device or not node or node.get("kind") != "serial" or
                _identity(device, node) != identity):
            raise LidarStartError("USB-адаптер отсутствует или изменился. Проверьте кабель заново.")
        if self._blocked(port):
            raise LidarStartError("Порт занят или зарезервирован колесом, BMS, принтером или лидаром.")
        try:
            allowed = (source.manual_lidar_probe_allowed(port, fp) if reserved else
                       source.manual_printer_port_allowed(port))
        except Exception:  # noqa: BLE001 — owner must positively authorize the check
            allowed = False
        if not allowed:
            raise LidarStartError("Порт ещё проверяется или уже опознан. Дождитесь проверки USB и повторите.")

    def prepare(self, body):
        if (not isinstance(body, dict) or set(body) != {"port"} or
                not isinstance(body.get("port"), str) or not SERIAL_RE.fullmatch(body["port"])):
            raise LidarStartError("Выберите текущий USB-UART из списка устройств.", 400)
        port = body["port"]
        topology = self.registry.usb(refresh=True)
        device, node = _find_port(topology, port)
        if not topology.get("available") or not device or not node or node.get("kind") != "serial":
            raise LidarStartError("Выбранного USB-UART нет в текущей топологии.")
        real, fp, source = os.path.realpath(port), fingerprint(port), self.source_provider()
        identity = _identity(device, node)
        self._assert_safe(port, real, fp, source, identity)
        token = secrets.token_urlsafe(32)
        with self._lock:
            now = self.clock()
            self._challenges = {k: v for k, v in self._challenges.items() if v[0] >= now}
            while len(self._challenges) >= 16:
                self._challenges.pop(next(iter(self._challenges)))
            self._challenges[token] = (now + self.CONFIRM_SECONDS, port, real, fp, source, identity)
        return {"ok": True, "token": token, "port": port, "device": real,
                "confirmWithinS": self.CONFIRM_SECONDS}

    def start(self, body):
        if (not isinstance(body, dict) or set(body) != {"token", "confirmed"} or
                not isinstance(body.get("token"), str) or len(body["token"]) > 128 or
                body.get("confirmed") is not True):
            raise LidarStartError("Нужно физически проверить кабель и подтвердить LDS-01.", 400)
        # Each confirmation authorizes at most ONE attempt, including failures.
        with self._lock:
            challenge = self._challenges.pop(body["token"], None)
        if not challenge or self.clock() > challenge[0]:
            raise LidarStartError("Подтверждение устарело. Проверьте устройство и начните заново.")
        if not self._start_lock.acquire(blocking=False):
            raise LidarStartError("Проверка лидара уже выполняется.")
        _, port, real, fp, source, identity = challenge
        from .lidar import open_serial, try_lock_port, unlock_port
        lock_fd = None
        handle = None
        reserved = False
        try:
            self._assert_safe(port, real, fp, source, identity)
            lock_fd = try_lock_port(port)
            if lock_fd in (None, -1):
                raise LidarStartError("USB-порт занят или общий замок недоступен. Повторите позже.")
            self._assert_safe(port, real, fp, source, identity)
            if not source.reserve_manual_lidar_probe(port, fp):
                raise LidarStartError("USB-порт изменился или проверяется. Повторите позже.")
            reserved = True
            self._assert_safe(port, real, fp, source, identity, reserved=True)
            import serial
            handle = open_serial(serial, port, LDS01_BAUD)
            opened = os.fstat(handle.fileno())
            if (opened.st_dev, opened.st_ino, opened.st_rdev, opened.st_ctime_ns) != fp:
                raise LidarStartError("USB-устройство изменилось во время открытия.")
            # Do not classify buffered bytes left by an earlier reader as a
            # response to this operator-confirmed attempt.
            handle.reset_input_buffer()
            self._assert_safe(port, real, fp, source, identity, reserved=True)
            if handle.write(LDS01_CMD_START) != len(LDS01_CMD_START):
                raise LidarStartError("Команду запуска не удалось передать полностью.")
            parser = Lds01Stream(crc_bad_limit=1_000_000_000)
            points = 0
            deadline = self.clock() + self.READ_SECONDS
            while self.clock() < deadline:
                self._assert_safe(port, real, fp, source, identity, reserved=True)
                chunk = handle.read(1024)
                if not chunk:
                    continue
                points += sum(len(batch) for batch, _ in parser.feed(chunk))
                if parser.crc_ok >= 6 and points >= 20:
                    self._assert_safe(port, real, fp, source, identity, reserved=True)
                    if not source.confirm_manual_lidar(port, fp):
                        raise LidarStartError("USB-устройство изменилось во время проверки.")
                    return {"ok": True, "port": port, "protocol": "lds01", "frames": parser.crc_ok,
                            "message": "Кадры LDS-01 с контрольной суммой получены. Дождитесь скана драйвера."}
            raise LidarStartError("После запуска нет достаточного числа корректных кадров LDS-01. "
                                  "Порт не подтверждён; проверьте модель, питание, кабель и скорость.")
        except LidarStartError:
            raise
        except Exception as exc:  # noqa: BLE001 — no reply is never proof
            raise LidarStartError("Не удалось проверить LDS-01: %s" % exc) from exc
        finally:
            if handle is not None:
                try:
                    handle.close()
                except Exception:  # noqa: BLE001
                    pass
            try:
                if reserved:
                    source.release_manual_lidar_probe(real)
            finally:
                unlock_port(lock_fd)
                self._start_lock.release()
