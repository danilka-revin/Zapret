"""Small, deterministic battery receipts, inspired by ploter_a1's three levels.

The input is one onboard JBD snapshot, never values supplied by the browser.
The result contains only printable text; printer control bytes are added by
printer.py, not copied from BMS strings. Missing/incomplete measurements are
explicit, including missing individual cells and optional protection data.
"""
from __future__ import annotations

import math
import textwrap
import unicodedata
from datetime import datetime

LEVELS = ("short", "medium", "full")
LANGUAGES = ("en", "ru")
CHARSETS = ("cp866", "cp1251", "utf8")

_CYR = dict(zip(
    "абвгдеёжзийклмнопрстуфхцчшщъыьэюя",
    ("a", "b", "v", "g", "d", "e", "yo", "zh", "z", "i", "y", "k", "l", "m", "n", "o", "p", "r", "s", "t", "u", "f", "kh", "ts", "ch", "sh", "shch", "", "y", "", "e", "yu", "ya"),
))
_SYMBOLS = {"°": " deg ", "№": "#", "−": "-", "–": "-", "—": "-", "≈": "~", "·": "/", "Δ": "Delta"}
_FAULT_EN = {
    "Перенапряжение банки": "Cell overvoltage", "Пониженное напряжение банки": "Cell undervoltage",
    "Перенапряжение АКБ": "Pack overvoltage", "Пониженное напряжение АКБ": "Pack undervoltage",
    "Перегрев при заряде": "Charge overtemperature", "Переохлаждение при заряде": "Charge undertemperature",
    "Перегрев при разряде": "Discharge overtemperature", "Переохлаждение при разряде": "Discharge undertemperature",
    "Перегрузка заряда": "Charge overcurrent", "Перегрузка разряда": "Discharge overcurrent",
    "Короткое замыкание": "Short circuit", "Ошибка AFE": "AFE error", "Программная защита": "Software protection",
}
_LABELS = {
    "en": {
        "title": "BATTERY REPORT", "report": "Report", "short": "short", "medium": "medium", "full": "full",
        "measured": "Measured", "complete": "Cells received", "incomplete": "WARNING: INCOMPLETE CELLS",
        "soc": "Charge", "voltage": "Voltage", "current": "Current", "power": "Power",
        "remaining": "Remaining / full", "cycles": "Cycles", "low": "Lowest cell", "high": "Highest cell",
        "average": "Average cell", "delta": "Spread", "max_temp": "Max temperature", "no_sensors": "No temperature sensors",
        "sensors": "TEMPERATURES", "sensor": "Sensor", "cells": "CELLS", "detail": "CELL DETAILS",
        "cell_key": "! min  ^ max  b balancing", "switches": "SWITCHES / PROTECTION",
        "charge_fet": "Charge FET", "discharge_fet": "Discharge FET", "balance": "Balancing",
        "faults": "Active protections", "none": "none", "yes": "on", "no": "off", "missing": "no data",
        "counters": "PROTECTION COUNTERS", "board": "BMS BOARD", "device": "Device", "firmware": "Firmware",
        "manufactured": "Manufactured", "thresholds": "PROTECTION THRESHOLDS", "footer": "ZAPRET / BATTERY",
        "v": "V", "a": "A", "w": "W", "ah": "Ah", "mv": "mV", "temp": "C", "cellsuf": "cells",
    },
    "ru": {
        "title": "ОТЧЁТ ПО АККУМУЛЯТОРУ", "report": "Отчёт", "short": "короткий", "medium": "средний", "full": "полный",
        "measured": "Измерено", "complete": "Получено банок", "incomplete": "ВНИМАНИЕ: НЕ ВСЕ БАНКИ",
        "soc": "Заряд", "voltage": "Напряжение", "current": "Ток", "power": "Мощность",
        "remaining": "Остаток / полная", "cycles": "Циклы", "low": "Мин. банка", "high": "Макс. банка",
        "average": "Среднее банки", "delta": "Разброс", "max_temp": "Макс. температура", "no_sensors": "Нет датчиков температуры",
        "sensors": "ТЕМПЕРАТУРЫ", "sensor": "Датчик", "cells": "БАНКИ", "detail": "ПОДРОБНО ПО БАНКАМ",
        "cell_key": "! мин  ^ макс  b баланс", "switches": "КЛЮЧИ / ЗАЩИТЫ",
        "charge_fet": "Ключ заряда", "discharge_fet": "Ключ разряда", "balance": "Балансировка",
        "faults": "Активные защиты", "none": "нет", "yes": "вкл", "no": "выкл", "missing": "нет данных",
        "counters": "СЧЁТЧИКИ ЗАЩИТ", "board": "ПЛАТА BMS", "device": "Устройство", "firmware": "Прошивка",
        "manufactured": "Дата выпуска", "thresholds": "ПОРОГИ ЗАЩИТ", "footer": "ZAPRET / АККУМУЛЯТОР",
        "v": "В", "a": "А", "w": "Вт", "ah": "Ач", "mv": "мВ", "temp": "°C", "cellsuf": "банок",
    },
}


def clean(value, language="ru", *, strip=True):
    """Strip device-originated controls (especially ESC/GS); optional ASCII transliteration."""
    text = unicodedata.normalize("NFC", str(value if value is not None else ""))
    output = []
    for char in text:
        if char in "\t\n\r" or char == "\u00a0":
            output.append(" ")
        elif not char.isprintable():
            continue
        elif language == "en" and char in _SYMBOLS:
            output.append(_SYMBOLS[char])
        elif language == "en" and char.lower() in _CYR:
            converted = _CYR[char.lower()]
            output.append(converted.upper() if char.isupper() else converted)
        elif language == "en" and not 32 <= ord(char) <= 126:
            deaccented = unicodedata.normalize("NFKD", char).encode("ascii", "ignore").decode("ascii")
            output.append(deaccented or "?")
        else:
            output.append(char)
    result = "".join(output)
    return result.strip() if strip else result


def _number(value, digits=0):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        if not math.isfinite(value):
            return None
        return f"{value:.{digits}f}"
    except (OverflowError, ValueError):
        return None


def render_receipt(snapshot, level="medium", *, width=32, language="en"):
    if level not in LEVELS or language not in LANGUAGES or type(width) is not int or not 24 <= width <= 48:
        raise ValueError("Неверный вид, язык или ширина чека")
    if not isinstance(snapshot, dict) or snapshot.get("connected") is not True or not isinstance(snapshot.get("data"), dict):
        raise ValueError("Нет свежих данных BMS")
    d = snapshot["data"]
    L = _LABELS[language]
    lines = []

    def paragraph(value, *, center=False):
        value = clean(value, language)
        parts = textwrap.wrap(value, width=width, break_long_words=True, break_on_hyphens=False) or [""]
        lines.extend(part.center(width).rstrip() if center else part for part in parts)

    def row(label, value):
        left, right = clean(label, language), clean(value, language)
        if len(left) + 1 + len(right) <= width:
            lines.append(left + " " * (width - len(left) - len(right)) + right)
        else:
            paragraph(left)
            for part in textwrap.wrap(right, width=width, break_on_hyphens=False) or [""]:
                lines.append(part.rjust(width))

    def num(key, digits, unit=""):
        val = _number(d.get(key), digits)
        return (val + (" " + unit if unit else "")) if val is not None else L["missing"]

    def cell_num(key, unit="mV"):
        val = _number(d.get(key), 0)
        return (val + " " + unit) if val is not None else L["missing"]

    def separator():
        lines.append("-" * width)

    stamp = d.get("updatedAt")
    try:
        when = datetime.fromtimestamp(stamp).strftime("%d.%m.%Y %H:%M:%S")
    except (TypeError, ValueError, OverflowError, OSError):
        when = L["missing"]

    paragraph(L["title"], center=True)
    separator()
    row(L["report"], L[level])
    row(L["measured"], when)
    received, expected = d.get("receivedCount"), d.get("cellCount")
    row(L["complete"], f"{received if type(received) is int else '?'} / {expected if type(expected) is int else '?'}")
    if snapshot.get("complete") is not True or d.get("complete") is not True:
        paragraph(L["incomplete"])
    separator()
    row(L["soc"], num("soc", 0, "%"))
    row(L["voltage"], num("volts", 2, L["v"]))
    row(L["current"], num("amps", 2, L["a"]))
    row(L["power"], num("watts", 1, L["w"]))
    remaining, full = _number(d.get("remainingAh"), 2), _number(d.get("fullAh"), 2)
    row(L["remaining"], (f"{remaining} / {full} {L['ah']}" if remaining is not None and full is not None else L["missing"]))
    if level != "short":
        row(L["cycles"], num("cycles", 0))
    separator()
    for title, key, index in (("low", "minMv", "minCell"), ("high", "maxMv", "maxCell")):
        value = cell_num(key, L["mv"])
        number = d.get(index)
        row(L[title], (f"#{number} {value}" if type(number) is int else value))
    if level != "short":
        row(L["average"], num("averageMv", 1, L["mv"]))
    row(L["delta"], cell_num("deltaMv", L["mv"]))
    cells = d.get("cells") if isinstance(d.get("cells"), list) else []
    cells = [cell for cell in cells[:127] if isinstance(cell, dict)]
    if level != "short":
        paragraph(L["cells"])
        # Two compact cells per row when they fit. Missing values are never zero.
        tiles = []
        for index, cell in enumerate(cells):
            number = cell.get("number") if type(cell.get("number")) is int else index + 1
            mv = _number(cell.get("mv"), 0)
            marks = ("!" if number == d.get("minCell") else "") + ("^" if number == d.get("maxCell") else "")
            marks += "b" if cell.get("balancing") is True else ""
            tiles.append(f"#{number:02d} {mv if mv is not None else '--'}{marks}")
        if not tiles:
            paragraph(L["missing"])
        else:
            pending = ""
            for tile in tiles:
                tile = clean(tile, language)
                if pending and len(pending) + 2 + len(tile) <= width:
                    lines.append(pending + " " * (width - len(pending) - len(tile)) + tile)
                    pending = ""
                else:
                    if pending:
                        paragraph(pending)
                    pending = tile
            if pending:
                paragraph(pending)
            paragraph(L["cell_key"])
    if level == "full":
        paragraph(L["detail"])
        for index, cell in enumerate(cells):
            number = cell.get("number") if type(cell.get("number")) is int else index + 1
            mv = _number(cell.get("mv"), 0)
            deviation = _number(cell.get("deviationMv"), 1)
            if mv is None:
                row(f"#{number:02d}", L["missing"])
            else:
                suffix = f" {float(deviation):+.1f}" if deviation is not None else ""
                suffix += " b" if cell.get("balancing") is True else ""
                row(f"#{number:02d}", f"{mv} {L['mv']}{suffix}")
    separator()
    temps = d.get("temperatures") if isinstance(d.get("temperatures"), list) else []
    valid_temps = [t for t in temps if _number(t, 1) is not None]
    if level == "short":
        row(L["max_temp"], f"{max(valid_temps):.1f} {L['temp']}" if valid_temps else L["missing"])
    else:
        paragraph(L["sensors"])
        if not temps:
            paragraph(L["no_sensors"] if d.get("ntcCount") == 0 else L["missing"])
        for index, value in enumerate(temps[:32]):
            formatted = _number(value, 1)
            row(f"{L['sensor']} {index + 1}", f"{formatted} {L['temp']}" if formatted is not None else L["missing"])
        separator()
        paragraph(L["switches"])
        for title, key in (("charge_fet", "chargeOn"), ("discharge_fet", "dischargeOn")):
            value = d.get(key)
            row(L[title], L["yes"] if value is True else L["no"] if value is False else L["missing"])
        balancing = d.get("balancingCells")
        if isinstance(balancing, list):
            row(L["balance"], ", ".join(str(n) for n in balancing[:127] if type(n) is int) or L["none"])
        else:
            row(L["balance"], L["missing"])
    faults = d.get("faults")
    if isinstance(faults, list):
        row(L["faults"], str(len(faults)) if faults else L["none"])
        for fault in faults[:2 if level == "short" else 32]:
            paragraph("! " + (_FAULT_EN.get(fault, fault) if language == "en" else str(fault)))
    else:
        row(L["faults"], L["missing"])
    if level == "full":
        separator()
        paragraph(L["counters"])
        counters = d.get("errorCounters")
        if not isinstance(counters, list) or not counters:
            paragraph(L["missing"])
        else:
            for item in counters[:32]:
                if isinstance(item, dict):
                    label = str(item.get("label") or item.get("key") or L["missing"])
                    label = _FAULT_EN.get(label, label) if language == "en" else label
                    count = _number(item.get("count"), 0)
                    row(label, count if count is not None else L["missing"])
        separator()
        paragraph(L["board"])
        for title, key in (("device", "deviceName"), ("firmware", "firmwareText"), ("manufactured", "manufactureDateText")):
            row(L[title], d.get(key) or L["missing"])
        separator()
        paragraph(L["thresholds"])
        thresholds = d.get("thresholds")
        if not isinstance(thresholds, list) or not thresholds:
            paragraph(L["missing"])
        else:
            for item in thresholds[:32]:
                if isinstance(item, dict):
                    value = _number(item.get("value"), 3)
                    row(item.get("label") or item.get("key") or L["missing"],
                        f"{value} {item.get('unit') or ''}" if value is not None else L["missing"])
    separator()
    paragraph(L["footer"], center=True)
    return lines


def encode_receipt(lines, *, profile="plain", language="en", charset="cp866"):
    """Return an init chunk and individual CRLF lines; never send a cut/heat command."""
    if profile not in ("plain", "escpos") or language not in LANGUAGES or charset not in CHARSETS:
        raise ValueError("Неверный профиль или кодировка принтера")
    codepage = {"cp866": 17, "cp1251": 46}
    init = b""
    if profile == "escpos":
        init = b"\x1b@"
        if language == "en":
            init += b"\x1bt\x00"
        elif charset in codepage:
            init += b"\x1bt" + bytes([codepage[charset]])
    encoding = "ascii" if language == "en" else ("utf-8" if charset == "utf8" else charset)
    chunks = [clean(line, language, strip=False).encode(encoding, "replace") + b"\r\n" for line in lines]
    return init, chunks + [b"\r\n"] * 4
