# -*- coding: utf-8 -*-
"""Карта лидара: сетка занятости, поза по скану и файл карты на рабочем столе.

Карта строится самим лидаром, как в ROS:

  * каждый скан (``[{a: угол, r: метры}]``) вписывается в сетку занятости
    лучами: клетки вдоль луча — свободные, конец луча — препятствие;
  * поза робота считается сопоставлением соседних сканов (scan matching) —
    одометрии от модулей на борту нет, а лидар у нас есть всегда;
  * карта живёт в сервере и отдаётся экрану одним кадром (PNG в base64),
    поэтому браузер не копит её у себя и не теряет при перезагрузке;
  * файл карты (PGM + YAML в формате ``map_server`` из ROS и PNG для
    просмотра) пишется на рабочий стол; при сбросе карты файл не удаляется,
    а перезаписывается пустым — путь и имя остаются прежними.

Настройки карты (разрешение сетки, дальность, чувствительность, забывание
старых препятствий, автосохранение) меняются с бортового экрана и хранятся
в ``~/.config/rus_slam/map.json`` — рядом с настройками лидара.

Модуль без внешних зависимостей: только стандартная библиотека.
"""

from __future__ import annotations

import base64
import json
import math
import os
import struct
import threading
import time
import zlib
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

#: Значение «клетку ещё не видели» в байтах сетки.
UNKNOWN = 255

#: Куда класть файлы карты (по умолчанию — рабочий стол).
MAP_DIR_ENV = "RC_MAP_DIR"

#: Имя файлов карты (без расширения).
MAP_STEM = "rus_slam_map"

#: Куда класть настройки карты (по умолчанию — в домашнем каталоге).
SETTINGS_PATH_ENV = "RC_MAP_SETTINGS"

#: Файл с настройками карты (в домашнем каталоге пользователя).
SETTINGS_PATH = Path.home() / ".config" / "rus_slam" / "map.json"

#: Границы серого в PGM — как у ``map_server`` из ROS.
PGM_OCCUPIED_PCT = 65      # выше — стена
PGM_FREE_PCT = 19          # ниже — свободно
PGM_OCCUPIED_VALUE = 0
PGM_FREE_VALUE = 254
PGM_UNKNOWN_VALUE = 205

#: Шаг сетки хеша для поиска ближайшей точки при сопоставлении сканов.
MATCH_CELL_M = 0.25

#: Сколько точек скана участвует в сопоставлении (быстрее и устойчивее).
MATCH_POINTS = 80

#: Сколько точек берётся на первом, грубом этапе сопоставления.
MATCH_COARSE_POINTS = 32

#: Каждый луч скана идёт в карту без пропусков, чтобы между лучами не оставалось
#: ложных UNKNOWN-клеток, мешающих выбору цели на карте.
SCAN_STRIDE = 1


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _num(value: Any, lo: float, hi: float, fallback: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback
    if not math.isfinite(number):
        return fallback
    return _clamp(number, lo, hi)


@dataclass(frozen=True)
class MapSettings:
    """Настройки карты лидара.

    resolution — сторона клетки, м;
    size_m — сторона карты, м (квадрат, робот начинает в центре);
    max_range — дальность луча, попадающего в карту, м;
    min_range — ближе этого расстояния точки в карту не идут (шум у лидара);
    hit — вероятность попадания: занятость клетки после одного попадания;
    miss — вероятность промаха: на сколько клетка «свободнеет» за промах;
    decay — период полураспада старых препятствий, с (0 — не забывать);
    matching — считать позу по скану (без неё карта «едет» вместе с роботом);
    autosave — перезаписывать файл карты на рабочем столе без запроса;
    autosave_s — как часто это делать, с.
    """

    resolution: float = 0.10
    size_m: float = 30.0
    max_range: float = 8.0
    min_range: float = 0.30
    hit: float = 0.65
    miss: float = 0.30
    decay: float = 0.0
    matching: bool = True
    autosave: bool = True
    autosave_s: float = 60.0

    @property
    def cells(self) -> int:
        """Сколько клеток по стороне при текущем разрешении."""
        return max(16, int(round(self.size_m / self.resolution)))

    def to_dict(self) -> dict:
        return {
            "resolution": self.resolution,
            "size_m": self.size_m,
            "max_range": self.max_range,
            "min_range": self.min_range,
            "hit": self.hit,
            "miss": self.miss,
            "decay": self.decay,
            "matching": self.matching,
            "autosave": self.autosave,
            "autosave_s": self.autosave_s,
        }

    @classmethod
    def from_dict(cls, raw: Any) -> "MapSettings":
        """Настройки из произвольного словаря: значения вне диапазона срезаются."""
        data = raw if isinstance(raw, dict) else {}
        default = cls()
        min_range = _num(data.get("min_range"), 0.05, 2.0, default.min_range)
        max_range = _num(data.get("max_range"), 1.0, 30.0, default.max_range)
        if max_range <= min_range:
            max_range = min_range + 0.5
        return cls(
            resolution=_num(data.get("resolution"), 0.05, 0.5, default.resolution),
            size_m=_num(data.get("size_m"), 5.0, 100.0, default.size_m),
            max_range=max_range,
            min_range=min_range,
            hit=_num(data.get("hit"), 0.05, 0.9, default.hit),
            miss=_num(data.get("miss"), 0.05, 0.9, default.miss),
            decay=_num(data.get("decay"), 0.0, 3600.0, default.decay),
            matching=bool(data.get("matching", default.matching)),
            autosave=bool(data.get("autosave", default.autosave)),
            autosave_s=_num(data.get("autosave_s"), 10.0, 3600.0, default.autosave_s),
        )


# --------------------------------------------------------------------- сетка --
class OccupancyGrid:
    """Сетка занятости: 0 — свободно, 100 — препятствие, 255 — не видели."""

    def __init__(self, settings: MapSettings):
        self.res = settings.resolution
        self.width = settings.cells
        self.height = self.width
        self.data = bytearray([UNKNOWN]) * (self.width * self.height)
        # Левый нижний угол карты в системе координат карты (робот в центре).
        self.origin = (-self.width * self.res / 2.0, -self.height * self.res / 2.0)

    # -- геометрия ---------------------------------------------------------- #
    def cell_of(self, x: float, y: float) -> Optional[tuple]:
        ix = int(math.floor((x - self.origin[0]) / self.res))
        iy = int(math.floor((y - self.origin[1]) / self.res))
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return ix, iy
        return None

    def cell_center(self, ix: int, iy: int) -> tuple:
        return (self.origin[0] + (ix + 0.5) * self.res,
                self.origin[1] + (iy + 0.5) * self.res)

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    # -- содержимое --------------------------------------------------------- #
    def reset(self) -> None:
        self.data = bytearray([UNKNOWN]) * (self.width * self.height)

    def get(self, ix: int, iy: int) -> int:
        return self.data[iy * self.width + ix]

    def set(self, ix: int, iy: int, value: int) -> None:
        self.data[iy * self.width + ix] = value

    def update_cell(self, ix: int, iy: int, hit: bool, settings: MapSettings) -> None:
        """Попадание (препятствие) или промах (свободно) в одной клетке."""
        if not self.in_bounds(ix, iy):
            return
        index = iy * self.width + ix
        current = self.data[index]
        if current == UNKNOWN:
            current = 0
        if hit:
            value = 100.0 - (100.0 - current) * (1.0 - settings.hit)
        else:
            value = current * (1.0 - settings.miss)
        self.data[index] = max(0, min(100, int(round(value))))

    def apply_decay(self, factor: float) -> None:
        """Забыть часть старых препятствий (factor — во сколько раз меньше)."""
        factor = _clamp(factor, 0.0, 1.0)
        if factor >= 1.0:
            return
        table = bytes(
            255 if value == UNKNOWN else max(0, min(100, int(round(value * factor))))
            for value in range(256)
        )
        self.data = bytearray(self.data.translate(table))

    # -- скан --------------------------------------------------------------- #
    def insert_scan(self, pose: tuple, points: Sequence[dict], settings: MapSettings) -> int:
        """Вписать скан в карту. Возвращает число затронутых клеток.

        Поза — (x, y, yaw) в системе координат карты; точки — как в телеметрии
        (``a`` — угол от «вперёд» против часовой, ``r`` — метры).
        """
        if not points:
            return 0
        px, py, yaw = pose
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        max_range = settings.max_range
        min_range = settings.min_range
        touched = 0
        origin_x, origin_y = self.origin
        res = self.res
        width = self.width
        height = self.height
        data = self.data
        hit_step = settings.hit
        miss_keep = 1.0 - settings.miss
        min_ix = width
        max_ix = -1
        min_iy = height
        max_iy = -1
        for point in points[::SCAN_STRIDE]:
            try:
                angle = float(point["a"])
                distance = float(point["r"])
            except (KeyError, TypeError, ValueError):
                continue
            if not (math.isfinite(angle) and math.isfinite(distance)):
                continue
            if distance < min_range:
                continue
            no_return = distance > max_range
            if no_return:
                distance = max_range
            # Точка в системе координат робота, затем в системе координат карты.
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            end_x = px + local_x * cos_y - local_y * sin_y
            end_y = py + local_x * sin_y + local_y * cos_y
            ix0 = int(math.floor((px - origin_x) / res))
            iy0 = int(math.floor((py - origin_y) / res))
            ix1 = int(math.floor((end_x - origin_x) / res))
            iy1 = int(math.floor((end_y - origin_y) / res))
            if not (0 <= ix0 < width and 0 <= iy0 < height):
                continue
            min_ix = min(min_ix, ix0, max(0, ix1))
            max_ix = max(max_ix, ix0, min(width - 1, ix1))
            min_iy = min(min_iy, iy0, max(0, iy1))
            max_iy = max(max_iy, iy0, min(height - 1, iy1))
            # Луч: клетки вдоль — свободные.
            steps = max(abs(ix1 - ix0), abs(iy1 - iy0))
            if steps:
                step_x = (ix1 - ix0) / steps
                step_y = (iy1 - iy0) / steps
                fx, fy = float(ix0), float(iy0)
                for index in range(1, steps):
                    fx += step_x
                    fy += step_y
                    ix = int(fx)
                    iy = int(fy)
                    if not (0 <= ix < width and 0 <= iy < height):
                        continue
                    cell = iy * width + ix
                    current = data[cell]
                    if current == UNKNOWN:
                        current = 0
                    value = current * miss_keep
                    data[cell] = max(0, min(100, int(round(value))))
                    touched += 1
            # Конец луча: препятствие (если лидар что-то увидел).
            if not no_return and 0 <= ix1 < width and 0 <= iy1 < height:
                cell = iy1 * width + ix1
                current = data[cell]
                if current == UNKNOWN:
                    current = 0
                value = 100.0 - (100.0 - current) * (1.0 - hit_step)
                data[cell] = max(0, min(100, int(round(value))))
                touched += 1
        # Заполняем одиночные UNKNOWN-пропуски между расходящимися лучами,
        # если по противоположным сторонам уже подтверждено свободное место.
        if touched and min_ix < max_ix and min_iy < max_iy:
            x_start = max(1, min_ix)
            x_end = min(width - 2, max_ix)
            y_start = max(1, min_iy)
            y_end = min(height - 2, max_iy)
            fill_indices = []
            for iy in range(y_start, y_end + 1):
                row = iy * width
                for ix in range(x_start, x_end + 1):
                    idx = row + ix
                    if data[idx] != UNKNOWN:
                        continue
                    left = data[idx - 1]
                    right = data[idx + 1]
                    up = data[idx - width]
                    down = data[idx + width]
                    if any( PG <= v < UNKNOWN for v in (left, right, up, down)
                            for PG in (PGM_OCCUPIED_PCT,) ):
                        continue
                    horiz_free = left <= PGM_FREE_PCT and right <= PGM_FREE_PCT
                    vert_free = up <= PGM_FREE_PCT and down <= PGM_FREE_PCT
                    if horiz_free or vert_free:
                        fill_indices.append(idx)
            for idx in fill_indices:
                data[idx] = 0
                touched += 1
        return touched

    # -- статистика -------------------------------------------------------- #
    def stats(self) -> dict:
        free = occupied = unknown = 0
        for value in self.data:
            if value == UNKNOWN:
                unknown += 1
            elif value >= PGM_OCCUPIED_PCT:
                occupied += 1
            elif value <= PGM_FREE_PCT:
                free += 1
            else:
                unknown += 1
        total = self.width * self.height
        known = total - unknown
        return {
            "width": self.width,
            "height": self.height,
            "cells": total,
            "free": free,
            "occupied": occupied,
            "unknown": unknown,
            "known": known,
            "coverage": round(known / total, 4) if total else 0.0,
            "sizeM": round(self.width * self.res, 2),
            "resolution": self.res,
        }

    # -- файлы и картинки --------------------------------------------------- #
    def grayscale_rows(self) -> list:
        """Строки серого изображения (как у ROS): верхняя строка — максимум Y."""
        width, height = self.width, self.height
        data = self.data
        rows = []
        for iy in range(height - 1, -1, -1):
            base = iy * width
            row = bytearray(width)
            for ix in range(width):
                value = data[base + ix]
                if value == UNKNOWN:
                    row[ix] = PGM_UNKNOWN_VALUE
                elif value >= PGM_OCCUPIED_PCT:
                    row[ix] = PGM_OCCUPIED_VALUE
                elif value <= PGM_FREE_PCT:
                    row[ix] = PGM_FREE_VALUE
                else:
                    row[ix] = PGM_UNKNOWN_VALUE
            rows.append(bytes(row))
        return rows

    def png_bytes(self) -> bytes:
        """Картинка карты в PNG (серое, как у map_server)."""
        return _png_gray(self.width, self.height, self.grayscale_rows())

    def pgm_bytes(self) -> bytes:
        """Файл карты в формате ROS map_server (P5)."""
        header = ("P5\n# RUS SLAM lidar map\n%d %d\n255\n"
                  % (self.width, self.height)).encode("ascii")
        body = b"".join(self.grayscale_rows())
        return header + body

    def yaml_text(self, image_name: str) -> str:
        """Описание карты в формате ROS (map_server читает такой yaml)."""
        return (
            "image: %s\n"
            "resolution: %.3f\n"
            "origin: [%.3f, %.3f, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n" % (
                image_name, self.res, self.origin[0], self.origin[1])
        )


def _png_gray(width: int, height: int, rows: Sequence[bytes]) -> bytes:
    """Собрать PNG из серых строк (без внешних библиотек)."""
    raw = bytearray()
    for row in rows:
        raw.append(0)          # фильтр строки: None
        raw.extend(row)
    compressed = zlib.compress(bytes(raw), 6)

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", header)
            + chunk(b"IDAT", compressed)
            + chunk(b"IEND", b""))


# ---------------------------------------------------------------------- поза --
class PoseEstimator:
    """Поза робота по скану: сопоставление соседних сканов.

    Одометрии на борту нет (мастер отдаёт только модули, без знака скорости),
    поэтому движение восстанавливается тем же лидаром: текущий скан
    сопоставляется с предыдущим, и найденное смещение добавляется к позе.
    """

    def __init__(self) -> None:
        self.pose: tuple = (0.0, 0.0, 0.0)
        self.prev: list = []
        self.quality: float = 0.0
        self.matched: int = 0
        self.moves: int = 0

    def reset(self) -> None:
        self.pose = (0.0, 0.0, 0.0)
        self.prev = []
        self.quality = 0.0
        self.matched = 0
        self.moves = 0

    # -- точки скана в системе координат робота ---------------------------- #
    @staticmethod
    def robot_points(points: Sequence[dict]) -> list:
        out = []
        for point in points:
            try:
                angle = float(point["a"])
                distance = float(point["r"])
            except (KeyError, TypeError, ValueError):
                continue
            if not (math.isfinite(angle) and math.isfinite(distance)):
                continue
            if distance < 0:
                continue
            out.append((distance * math.cos(angle), distance * math.sin(angle)))
        return out

    def update(self, points: Sequence[dict], enabled: bool = True) -> tuple:
        """Обновить позу по очередному скану. Возвращает позу (x, y, yaw).

        Сопоставление даёт сдвиг сцены в системе координат робота; движение
        робота — обратное этому сдвигу, поэтому знак меняется на противоположный.
        """
        current = self.robot_points(points)
        if not current:
            return self.pose
        if not enabled or not self.prev:
            self.prev = current
            return self.pose
        delta = self._match(self.prev, current)
        if delta is not None:
            dx, dy, dyaw = delta
            px, py, yaw = self.pose
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            self.pose = (px - (dx * cos_y - dy * sin_y),
                         py - (dx * sin_y + dy * cos_y),
                         yaw - dyaw)
            self.moves += 1
        self.prev = current
        return self.pose

    # -- сопоставление ------------------------------------------------------ #
    @staticmethod
    def _hash(points: Sequence[tuple]) -> dict:
        buckets: dict = {}
        for x, y in points:
            key = (int(math.floor(x / MATCH_CELL_M)), int(math.floor(y / MATCH_CELL_M)))
            buckets.setdefault(key, []).append((x, y))
        return buckets

    @staticmethod
    def _nearest(buckets: dict, x: float, y: float):
        base_x = int(math.floor(x / MATCH_CELL_M))
        base_y = int(math.floor(y / MATCH_CELL_M))
        best = None
        best_dist = None
        for radius in (1, 2, 3):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if radius > 1 and abs(dx) != radius and abs(dy) != radius:
                        continue
                    for px, py in buckets.get((base_x + dx, base_y + dy), ()):
                        dist = (px - x) ** 2 + (py - y) ** 2
                        if best_dist is None or dist < best_dist:
                            best_dist = dist
                            best = (px, py)
            if best is not None:
                break
        return best, best_dist

    def _cost(self, buckets: dict, source: Sequence[tuple],
              dx: float, dy: float, dyaw: float) -> tuple:
        cos_a, sin_a = math.cos(dyaw), math.sin(dyaw)
        total = 0.0
        hits = 0
        penalty = 4.0
        for x, y in source:
            tx = x * cos_a - y * sin_a + dx
            ty = x * sin_a + y * cos_a + dy
            best, dist = self._nearest(buckets, tx, ty)
            if dist is None:
                total += penalty
            else:
                total += dist
                hits += 1
        count = len(source) or 1
        return (total / count, hits)

    def _match(self, previous: Sequence[tuple], current: Sequence[tuple]):
        """Найти смещение (dx, dy, dyaw), совмещающее скан с предыдущим."""
        if len(previous) < 20 or len(current) < 20:
            return None
        buckets = self._hash(current)
        step = max(1, len(previous) // MATCH_POINTS)
        source = list(previous[::step])
        if len(source) < 20:
            source = list(previous)
        # Грубый этап: меньше точек — поиск шире и дешевле.
        coarse_step = max(1, len(source) // MATCH_COARSE_POINTS)
        coarse = list(source[::coarse_step])
        if len(coarse) < 10:
            coarse = list(source)

        best = None
        best_cost = None
        best_hits = 0
        # Крупный поиск: ±0,6 м и ±12°.
        for dx in (-0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6):
            for dy in (-0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6):
                for dyaw_deg in (-12.0, -6.0, 0.0, 6.0, 12.0):
                    dyaw = math.radians(dyaw_deg)
                    cost, hits = self._cost(buckets, coarse, dx, dy, dyaw)
                    if best_cost is None or cost < best_cost:
                        best_cost = cost
                        best_hits = hits
                        best = (dx, dy, dyaw)
        if best is None:
            return None
        # Точный поиск вокруг лучшего — уже по полному набору точек.
        base_dx, base_dy, base_dyaw = best
        for dx in (-0.1, -0.05, 0.0, 0.05, 0.1):
            for dy in (-0.1, -0.05, 0.0, 0.05, 0.1):
                for dyaw_deg in (-2.5, -1.25, 0.0, 1.25, 2.5):
                    dyaw = base_dyaw + math.radians(dyaw_deg)
                    cost, hits = self._cost(buckets, source,
                                             base_dx + dx, base_dy + dy, dyaw)
                    if cost < best_cost:
                        best_cost = cost
                        best_hits = hits
                        best = (base_dx + dx, base_dy + dy, dyaw)
        # Слишком большое смещение — так быстро робот не едет: верим только
        # разумным движениям, иначе карта уедет от мусора в скане.
        if abs(best[0]) > 1.0 or abs(best[1]) > 1.0 or abs(best[2]) > math.radians(45):
            return None
        # Мало совпавших точек — сопоставление ненадёжно, позу не двигаем.
        if best_hits < len(source) * 0.5:
            self.quality = round(best_hits / max(1, len(source)), 3)
            self.matched = best_hits
            return None
        self.quality = round(best_hits / max(1, len(source)), 3)
        self.matched = best_hits
        # Оценка качества: средняя ошибка в метрах.
        mean_err = math.sqrt(max(0.0, best_cost))
        if mean_err > 0.35:
            return None
        return best


# ------------------------------------------------------------------- файлы ---
def settings_path() -> Path:
    """Путь к файлу настроек карты (можно переопределить RC_MAP_SETTINGS)."""
    override = os.environ.get(SETTINGS_PATH_ENV)
    if override:
        return Path(override)
    return SETTINGS_PATH


def map_dir() -> Path:
    """Каталог для файлов карты: рабочий стол (или то, что задано в RC_MAP_DIR)."""
    explicit = (os.environ.get(MAP_DIR_ENV) or "").strip()
    if explicit:
        return Path(explicit).expanduser()
    home = Path.home()
    # Каталог рабочего стола из настроек XDG (у Ubuntu это ~/Desktop).
    user_dirs = home / ".config" / "user-dirs.dirs"
    try:
        if user_dirs.is_file():
            for line in user_dirs.read_text(encoding="utf-8", errors="ignore").splitlines():
                if line.startswith("XDG_DESKTOP_DIR"):
                    value = line.split("=", 1)[-1].strip().strip('"')
                    value = value.replace("$HOME", str(home))
                    candidate = Path(value).expanduser()
                    if candidate.is_dir():
                        return candidate
    except OSError:
        pass
    for name in ("Desktop", "Рабочий стол", "Робочий стол"):
        candidate = home / name
        if candidate.is_dir():
            return candidate
    # Рабочего стола нет — создадим его: карта должна лежать на виду.
    return home / "Desktop"


def map_paths(directory: Optional[Path] = None) -> dict:
    """Пути файлов карты: PGM и YAML (как в ROS) плюс PNG для просмотра."""
    base = Path(directory) if directory is not None else map_dir()
    return {
        "dir": str(base),
        "pgm": str(base / (MAP_STEM + ".pgm")),
        "yaml": str(base / (MAP_STEM + ".yaml")),
        "png": str(base / (MAP_STEM + ".png")),
    }


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "wb") as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


# ------------------------------------------------------------------ сервис ---
class SlamMapService:
    """Карта лидара целиком: сетка, поза, настройки, файл на рабочем столе."""

    def __init__(self, settings: Optional[MapSettings] = None,
                 directory: Optional[Path] = None) -> None:
        self._lock = threading.RLock()
        self._settings = settings or MapSettings()
        self._grid = OccupancyGrid(self._settings)
        self._pose = PoseEstimator()
        self._directory = Path(directory) if directory is not None else None
        self._png_cache: Optional[bytes] = None
        self._dirty = True      # карта изменилась — файл пора перезаписать
        self._png_stale = True  # картинка устарела — надо перерисовать
        self._scans = 0
        self._points = 0
        self._updated_at = 0.0
        self._saved_at = 0.0
        self._last_decay_at = 0.0
        self._last_save_at = 0.0
        # Монотонная ревизия для навигации: цель нужно перепланировать, если
        # карта изменилась между расчётом пути и отправкой моторам.
        self._revision = 0

    # -- настройки ---------------------------------------------------------- #
    @property
    def settings(self) -> MapSettings:
        with self._lock:
            return self._settings

    def set_settings(self, patch: Any) -> dict:
        """Применить настройки. Смена геометрии очищает карту (об этом честно
        пишем в ответе — экран покажет, что карту надо строить заново)."""
        with self._lock:
            current = self._settings.to_dict()
            if isinstance(patch, dict):
                merged = dict(current)
                merged.update({k: v for k, v in patch.items() if k in current})
            else:
                merged = current
            new = MapSettings.from_dict(merged)
            geometry_changed = (new.resolution != self._settings.resolution
                                or new.size_m != self._settings.size_m)
            self._settings = new
            if geometry_changed:
                self._grid = OccupancyGrid(new)
                self._png_cache = None
                self._dirty = True
                self._png_stale = True
                self._revision += 1
            self._save_settings_file()
            return {"settings": new.to_dict(), "mapReset": geometry_changed}

    def _save_settings_file(self) -> None:
        try:
            path = settings_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps(self._settings.to_dict(), ensure_ascii=False, indent=2),
                encoding="utf-8")
        except OSError:
            pass    # настройки просто не переживут перезапуск — не беда

    def load_settings_file(self) -> None:
        """Поднять настройки из файла (если он есть)."""
        try:
            path = settings_path()
            if path.is_file():
                raw = json.loads(path.read_text(encoding="utf-8"))
                with self._lock:
                    self._settings = MapSettings.from_dict(raw)
                    self._grid = OccupancyGrid(self._settings)
                    self._png_cache = None
                    self._dirty = True
                    self._png_stale = True
                    self._revision += 1
        except (OSError, ValueError):
            pass

    # -- скан --------------------------------------------------------------- #
    def ingest(self, points: Sequence[dict], stamp: Optional[float] = None) -> bool:
        """Вписать скан в карту. True — карта изменилась."""
        if not points:
            return False
        with self._lock:
            settings = self._settings
            pose = self._pose.update(points, enabled=settings.matching)
            self._grid.insert_scan(pose, points, settings)
            self._scans += 1
            self._points += len(points)
            self._updated_at = time.time() if stamp is None else stamp
            self._dirty = True
            self._png_stale = True
            self._revision += 1
            self._maybe_decay()
        return True

    def _maybe_decay(self) -> None:
        """Забыть старые препятствия (если включено)."""
        settings = self._settings
        if settings.decay <= 0:
            return
        now = time.time()
        if not self._last_decay_at:
            self._last_decay_at = now
            return
        elapsed = now - self._last_decay_at
        if elapsed < 10.0:
            return
        self._last_decay_at = now
        factor = 0.5 ** (elapsed / settings.decay)
        self._grid.apply_decay(factor)
        self._dirty = True
        self._png_stale = True

    # -- сброс и сохранение ------------------------------------------------- #
    def reset(self, save: bool = True) -> dict:
        """Очистить карту. Файл карты не удаляется — он просто перезаписывается."""
        with self._lock:
            self._grid.reset()
            self._pose.reset()
            self._png_cache = None
            self._dirty = True
            self._png_stale = True
            self._updated_at = time.time()
            self._scans = 0
            self._points = 0
            self._revision += 1
            saved = self.save() if save else {"ok": True, "paths": map_paths(self._directory)}
            return {"ok": True, "paths": saved.get("paths") or map_paths(self._directory),
                    "saved": bool(save), "savedAt": saved.get("savedAt", 0.0)}

    def save(self) -> dict:
        """Записать карту на рабочий стол: PGM + YAML (ROS) и PNG."""
        with self._lock:
            paths = map_paths(self._directory)
            grid = self._grid
            try:
                _atomic_write(Path(paths["pgm"]), grid.pgm_bytes())
                _atomic_write(Path(paths["yaml"]),
                              grid.yaml_text(Path(paths["pgm"]).name).encode("utf-8"))
                _atomic_write(Path(paths["png"]), grid.png_bytes())
                self._saved_at = time.time()
                self._last_save_at = self._saved_at
                self._dirty = False
                return {"ok": True, "paths": paths, "savedAt": self._saved_at}
            except OSError as exc:
                return {"ok": False, "error": str(exc), "paths": paths}

    def maybe_autosave(self) -> None:
        """Перезаписать файл карты, если она менялась и подошло время."""
        with self._lock:
            settings = self._settings
            if not settings.autosave or not self._dirty:
                return
            now = time.time()
            if now - self._last_save_at < settings.autosave_s:
                return
            self._dirty = False
        self.save()

    # -- снимок для экрана -------------------------------------------------- #
    def png(self) -> bytes:
        with self._lock:
            if self._png_cache is None or self._png_stale:
                self._png_cache = self._grid.png_bytes()
                self._png_stale = False
            return self._png_cache

    def snapshot(self, with_png: bool = True) -> dict:
        """Всё, что нужно экрану: настройки, статистика, поза, картинка карты."""
        with self._lock:
            stats = self._grid.stats()
            pose = self._pose.pose
            payload = {
                "ok": True,
                "settings": self._settings.to_dict(),
                "stats": stats,
                "pose": {"x": round(pose[0], 3), "y": round(pose[1], 3),
                         "yaw": round(pose[2], 4)},
                "origin": {"x": round(self._grid.origin[0], 3),
                           "y": round(self._grid.origin[1], 3)},
                "quality": self._pose.quality,
                "matched": self._pose.matched,
                "scans": self._scans,
                "points": self._points,
                "updatedAt": self._updated_at,
                "savedAt": self._saved_at,
                "paths": map_paths(self._directory),
                "resolution": self._grid.res,
                "width": self._grid.width,
                "height": self._grid.height,
            }
            if with_png:
                payload["png"] = base64.b64encode(self.png()).decode("ascii")
            return payload

    def navigation_snapshot(self) -> dict:
        """Атомарный снимок сетки и позы для планировщика.

        ``data`` — байты значений сетки (0…100, 255 — не исследовано). Снимок
        копируется под тем же lock, чтобы поиск пути не увидел половину старой
        и половину новой карты. В отличие от ``snapshot()``, PNG здесь не
        кодируется: путь рассчитывается по occupancy grid.
        """
        with self._lock:
            pose = self._pose.pose
            return {
                "data": bytes(self._grid.data),
                "width": self._grid.width,
                "height": self._grid.height,
                "resolution": self._grid.res,
                "origin": {"x": self._grid.origin[0], "y": self._grid.origin[1]},
                "pose": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
                "quality": self._pose.quality,
                "matched": self._pose.matched,
                "scans": self._scans,
                "points": self._points,
                "updatedAt": self._updated_at,
                "revision": self._revision,
                "settings": self._settings.to_dict(),
            }
