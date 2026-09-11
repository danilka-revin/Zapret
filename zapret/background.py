"""
Генерация стеклянного фона (градиент + мягкие цветовые «пятна» + карточки
с эффектом frosted-glass) в чистом Python. Пишет PNG без внешних зависимостей.
"""

import struct
import zlib

_SIGNATURE = b"\x89PNG\r\n\x1a\n"

# Палитра «стекла» в тёмной теме
TOP = (14, 16, 36)        # #0e1024
BOTTOM = (8, 10, 20)      # #080a14
ORBS = [
    (0.18, 0.10, 0.55, (45, 224, 193), 0.32),    # бирюзовый
    (0.88, 0.14, 0.62, (90, 99, 255), 0.36),     # индиго
    (0.28, 0.98, 0.66, (177, 74, 255), 0.26),    # фиолетовый
]

# Параметры «стеклянной» карточки
CARD_FILL_ALPHA = 0.06        # белый, полупрозрачный
CARD_BORDER_ALPHA = 0.13
CARD_BORDER_W = 1.6
CARD_RADIUS = 18


def _chunk(tag: bytes, data: bytes) -> bytes:
    payload = struct.pack(">I", len(data)) + tag + data
    payload += struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    return payload


def _render_base(w: int, h: int, out: bytearray) -> None:
    top_r, top_g, top_b = TOP
    bot_r, bot_g, bot_b = BOTTOM
    orbs = [(cx * w, cy * h, rad * max(w, h), r, g, b, s)
            for cx, cy, rad, (r, g, b), s in ORBS]

    for y in range(h):
        t = y / max(h - 1, 1)
        base_r = top_r + (bot_r - top_r) * t
        base_g = top_g + (bot_g - top_g) * t
        base_b = top_b + (bot_b - top_b) * t
        row_off = y * w * 4
        for x in range(w):
            r, g, b = base_r, base_g, base_b
            for ox, oy, radius, orb_r, orb_g, orb_b, strength in orbs:
                dx = x - ox
                dy = y - oy
                d = (dx * dx + dy * dy) ** 0.5 / radius
                if d < 1.0:
                    a = (1.0 - d) ** 2 * strength
                    r = r * (1 - a) + orb_r * a
                    g = g * (1 - a) + orb_g * a
                    b = b * (1 - a) + orb_b * a
            i = row_off + x * 4
            out[i] = int(round(r))
            out[i + 1] = int(round(g))
            out[i + 2] = int(round(b))
            out[i + 3] = 255


def _inside_rounded(x, y, x1, y1, x2, y2, r) -> bool:
    if x < x1 or x > x2 or y < y1 or y > y2:
        return False
    # углы
    if x < x1 + r and y < y1 + r:
        return (x - (x1 + r)) ** 2 + (y - (y1 + r)) ** 2 <= r * r
    if x > x2 - r and y < y1 + r:
        return (x - (x2 - r)) ** 2 + (y - (y1 + r)) ** 2 <= r * r
    if x < x1 + r and y > y2 - r:
        return (x - (x1 + r)) ** 2 + (y - (y2 - r)) ** 2 <= r * r
    if x > x2 - r and y > y2 - r:
        return (x - (x2 - r)) ** 2 + (y - (y2 - r)) ** 2 <= r * r
    return True


def _composite_cards(w: int, h: int, out: bytearray, cards) -> None:
    for (x1, y1, x2, y2) in cards:
        r = CARD_RADIUS
        for y in range(max(0, y1 - 2), min(h, y2 + 3)):
            for x in range(max(0, x1 - 2), min(w, x2 + 3)):
                if not _inside_rounded(x, y, x1, y1, x2, y2, r):
                    continue
                i = y * w * 4 + x * 4
                inside_shrunk = _inside_rounded(
                    x, y, x1 + CARD_BORDER_W, y1 + CARD_BORDER_W,
                    x2 - CARD_BORDER_W, y2 - CARD_BORDER_W, r - CARD_BORDER_W,
                )
                if inside_shrunk:
                    a = CARD_FILL_ALPHA
                    # лёгкий блик сверху карточки
                    if y - y1 < r:
                        a += 0.05 * (1 - (y - y1) / r)
                else:
                    a = CARD_BORDER_ALPHA
                if a <= 0:
                    continue
                out[i] = int(out[i] * (1 - a) + 255 * a)
                out[i + 1] = int(out[i + 1] * (1 - a) + 255 * a)
                out[i + 2] = int(out[i + 2] * (1 - a) + 255 * a)


def render_ui_png(w: int, h: int, cards) -> bytes:
    out = bytearray(w * h * 4)
    _render_base(w, h, out)
    _composite_cards(w, h, out, cards)

    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw += out[y * w * 4:(y + 1) * w * 4]

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0)  # 8-bit RGBA
    return (
        _SIGNATURE
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        + _chunk(b"IEND", b"")
    )


def backdrop_png(w: int, h: int) -> bytes:
    """Только фон (без карточек)."""
    return render_ui_png(w, h, [])
