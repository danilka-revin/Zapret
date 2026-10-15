"""Read-only JBD UART telemetry, following danilka-revin/ploter_a1.

See docs/battery.md for protocol provenance and wiring. No factory-mode,
settings, FET or capacity-write commands are exposed by this module.
"""
from __future__ import annotations

import atexit
import copy
import csv
import io
import logging
import os
import threading
import time

log = logging.getLogger(__name__)
MAX_AGE = 5.0
FAULTS = (
    "Перенапряжение банки", "Пониженное напряжение банки",
    "Перенапряжение АКБ", "Пониженное напряжение АКБ",
    "Перегрев при заряде", "Переохлаждение при заряде",
    "Перегрев при разряде", "Переохлаждение при разряде",
    "Перегрузка заряда", "Перегрузка разряда", "Короткое замыкание",
    "Ошибка AFE", "Программная защита",
)


class ProtocolError(ValueError):
    pass


def reserved_ports():
    port = os.environ.get("RC_BMS_PORT", "").strip()
    return {port, os.path.realpath(port)} if port else set()


def read_request(register):
    if register not in (3, 4):
        raise ProtocolError("Разрешено только чтение телеметрии 0x03/0x04")
    checksum = (-register) & 0xFFFF
    return bytes([0xDD, 0xA5, register, 0, checksum >> 8, checksum & 255, 0x77])


def extract_frames(buffer):
    """Accept fragmented frames, noise, and both JBD status checksum variants."""
    frames = []
    while buffer:
        start = buffer.find(b"\xdd")
        if start < 0:
            return frames, b""
        buffer = buffer[start:]
        if len(buffer) < 7:
            break
        size = buffer[3] + 7
        if len(buffer) < size:
            break
        frame = buffer[:size]
        checksum = int.from_bytes(frame[-3:-1], "big")
        if frame[-1] != 0x77 or checksum not in (
            (-sum(frame[2:-3])) & 0xFFFF, (-sum(frame[3:-3])) & 0xFFFF,
        ):
            buffer = buffer[1:]
            continue
        frames.append((frame[1], frame[2], frame[4:-3]))
        buffer = buffer[size:]
    return frames, buffer


def parse_snapshot(basic, cells, now=None):
    if len(basic) < 23 or len(basic) < 23 + basic[22] * 2:
        raise ProtocolError("Неполный ответ общих параметров/температур")
    if not cells or len(cells) % 2:
        raise ProtocolError("Неполный ответ напряжений банок")
    count = basic[21]
    if not 1 <= count <= 127:
        raise ProtocolError("Некорректное количество банок BMS")
    word = lambda offset: int.from_bytes(basic[offset:offset + 2], "big")
    millivolts = [int.from_bytes(cells[i:i + 2], "big") for i in range(0, len(cells), 2)]
    balance = word(12) | (word(14) << 16)
    average = sum(millivolts) / len(millivolts)
    faults = word(16)
    current = int.from_bytes(basic[2:4], "big", signed=True) / 100
    voltage = word(0) / 100
    complete = len(millivolts) == count
    return {
        "updatedAt": time.time() if now is None else now,
        "volts": voltage, "amps": current, "watts": round(voltage * current, 2),
        "soc": basic[19], "remainingAh": word(4) / 100, "fullAh": word(6) / 100,
        "cycles": word(8), "firmwareVersion": basic[18],
        "temperatures": [round((word(23 + i * 2) - 2731) / 10, 1) for i in range(basic[22])],
        "chargeOn": bool(basic[20] & 1), "dischargeOn": bool(basic[20] & 2),
        "faultRaw": faults,
        "faults": [label for bit, label in enumerate(FAULTS) if faults & (1 << bit)]
                  + ([f"Неизвестные защиты: 0x{faults & ~0x1FFF:04X}"] if faults & ~0x1FFF else []),
        "cellCount": count, "receivedCount": len(millivolts), "complete": complete,
        "minMv": min(millivolts), "maxMv": max(millivolts),
        "deltaMv": max(millivolts) - min(millivolts), "averageMv": round(average, 1),
        # Never silently truncate extra banks or omit missing ones.
        "cells": [
            {"number": i + 1, "mv": millivolts[i] if i < len(millivolts) else None,
             "deviationMv": round(millivolts[i] - average, 1) if i < len(millivolts) else None,
             "balancing": bool(balance & (1 << i))}
            for i in range(max(count, len(millivolts)))
        ],
    }


class BatteryManager:
    """One polling thread on the kiosk server; HTTP requests never touch UART."""
    def __init__(self, port="", opener=None):
        self.port = port
        self._opener = opener or self._open
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._sample = None
        self._error = "Ожидание BMS" if port else "BMS не настроена: задайте RC_BMS_PORT в .env"

    def _open(self):
        import serial
        return serial.Serial(
            self.port, baudrate=9600, bytesize=8, parity="N", stopbits=1,
            timeout=0.05, write_timeout=0.8, xonxoff=False, rtscts=False, dsrdtr=False,
        )

    def start(self):
        if self.port and not (self._thread and self._thread.is_alive()):
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="jbd-bms", daemon=True)
            self._thread.start()
            atexit.register(self.stop)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def _read(self, transport, register):
        transport.reset_input_buffer()
        transport.write(read_request(register))
        buffer = b""
        deadline = time.monotonic() + 0.8
        while not self._stop.is_set() and time.monotonic() < deadline:
            buffer += transport.read(256)
            frames, buffer = extract_frames(buffer)
            for reg, status, data in frames:
                if reg == register:
                    if status:
                        raise ProtocolError(f"BMS: статус 0x{status:02X}, регистр 0x{reg:02X}")
                    return data
        raise ProtocolError(f"BMS не ответила на регистр 0x{register:02X}")

    def poll(self, transport):
        sample = parse_snapshot(self._read(transport, 3), self._read(transport, 4))
        with self._lock:
            self._sample = sample
            self._error = ""

    def _loop(self):
        from .lidar import try_lock_port, unlock_port
        while not self._stop.is_set():
            transport = None
            fd = None
            try:
                fd = try_lock_port(self.port)
                if fd is None:
                    raise OSError("Порт BMS занят другим процессом")
                transport = self._opener()
                log.info("BMS: подключение к %s, JBD 9600 8N1", self.port)
                while not self._stop.is_set():
                    self.poll(transport)
                    self._stop.wait(1)
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                log.warning("BMS: %s", exc)
            finally:
                try:
                    if transport is not None:
                        transport.close()
                finally:
                    unlock_port(fd)
            self._stop.wait(3)

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        with self._lock:
            sample = copy.deepcopy(self._sample)
            error = self._error
        age = max(0, now - sample["updatedAt"]) if sample else None
        connected = sample is not None and age <= MAX_AGE and not error
        complete = bool(sample and sample["complete"])
        reason = error or ("Данные устарели" if not connected else (
            "Все банки прочитаны" if complete else
            f"Неполный отчёт: получено {sample['receivedCount']} из {sample['cellCount']} банок"
        ))
        return {"configured": bool(self.port), "connected": connected,
                "complete": complete, "reason": reason, "ageS": age, "data": sample}


def report_response(snapshot):
    """CSV contains every bank from a single snapshot, including missing banks."""
    from flask import Response, jsonify
    if not snapshot.get("connected") or not snapshot.get("data"):
        return jsonify({"ok": False, "reason": snapshot["reason"]}), 503
    data = snapshot["data"]
    out = io.StringIO(newline="")
    writer = csv.writer(out, delimiter=";")
    writer.writerow(["Отчёт JBD BMS", "Время Unix", data["updatedAt"]])
    writer.writerow(["Полнота", snapshot["reason"], "Получено", data["receivedCount"], "Ожидалось", data["cellCount"]])
    for label, key in (("Заряд, %", "soc"), ("Напряжение, В", "volts"), ("Ток, А", "amps"),
                       ("Мощность, Вт", "watts"), ("Остаток, Ач", "remainingAh"),
                       ("Полная ёмкость, Ач", "fullAh"), ("Циклы", "cycles"),
                       ("Минимум, мВ", "minMv"), ("Максимум, мВ", "maxMv"),
                       ("Разброс, мВ", "deltaMv")):
        writer.writerow([label, data[key]])
    writer.writerow(["Температуры, °C", *data["temperatures"]])
    writer.writerow(["Заряд разрешён", data["chargeOn"], "Разряд разрешён", data["dischargeOn"]])
    writer.writerow(["Защиты", *data["faults"]] if data["faults"] else ["Защиты", "Нет активных защит"])
    writer.writerow(["Банка", "Напряжение, мВ", "Отклонение от среднего, мВ", "Балансировка", "Данные"])
    for cell in data["cells"]:
        writer.writerow([cell["number"], cell["mv"], cell["deviationMv"],
                         "Да" if cell["balancing"] else "Нет", "Нет данных" if cell["mv"] is None else "Прочитано"])
    return Response("\ufeff" + out.getvalue(), content_type="text/csv; charset=utf-8",
                    headers={"Content-Disposition": 'attachment; filename="battery-report.csv"',
                             "Cache-Control": "no-store"})


def kiosk_snapshot():
    """The operator server relays kiosk telemetry, never opens the BMS UART."""
    import json
    import urllib.request
    try:
        port = int(os.environ.get("RC_KIOSK_PORT") or "8090")
        if not 1 <= port <= 65535:
            raise ValueError("invalid kiosk port")
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/battery", timeout=1) as response:
            return json.load(response)["battery"]
    except (OSError, ValueError, KeyError) as exc:
        return {"configured": bool(reserved_ports()), "connected": False, "complete": False,
                "reason": "Нет связи с BMS: запустите бортовой сервер (8090)", "ageS": None, "data": None}
