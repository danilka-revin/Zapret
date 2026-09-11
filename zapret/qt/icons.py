"""
Векторные иконки (SVG, стиль Feather) с перекраской и кэшем.

Рисуются контурными путями 24×24, поэтому остаются чёткими на любом DPI и
легко перекрашиваются под текущий акцент.
"""

from __future__ import annotations

from functools import lru_cache

from PySide6.QtCore import QByteArray, QRectF, Qt
from PySide6.QtGui import QPainter, QPixmap
from PySide6.QtSvg import QSvgRenderer

# ---------------------------------------------------------------------------
# Геометрия иконок (24×24, контурные)
# ---------------------------------------------------------------------------

ICONS: dict[str, list[str]] = {
    "power": ['<path d="M18.36 6.64A9 9 0 1 1 5.64 6.64"/><path d="M12 2v10"/>'],
    "power-off": ['<path d="M18.36 6.64A9 9 0 0 1 20.77 13"/><path d="M6.16 6.5A9 9 0 1 0 12 21"/>'
                  '<path d="M12 2v6"/><path d="m2 2 20 20"/>'],
    "shield": ['<path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/>'],
    "shield-off": ['<path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/><path d="m4 4 16 16"/>'],
    "shield-check": ['<path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/><path d="m9 12 2 2 4-4"/>'],
    "zap": ['<path d="M13 2 3 14h9l-1 8 10-12h-9l1-8z"/>'],
    "sparkles": ['<path d="M12 3l1.9 5.1L19 10l-5.1 1.9L12 17l-1.9-5.1L5 10l5.1-1.9L12 3z"/>'
                 '<path d="M19 3v3M17.5 4.5h3"/>'],
    "palette": ['<path d="M12 22a10 10 0 1 1 0-20c5.5 0 10 3.6 10 8 0 4-3 5-5 5h-2a2 2 0 0 0-1.5 3.3c.4.5.5 1 .3 1.5-.3.7-1 1.2-1.8 1.2z"/>'
                '<circle cx="13.5" cy="6.5" r="1.2"/><circle cx="17.5" cy="10.5" r="1.2"/>'
                '<circle cx="8.5" cy="7.5" r="1.2"/><circle cx="6.5" cy="12.5" r="1.2"/>'],
    "sun": ['<circle cx="12" cy="12" r="4.2"/>'
            '<path d="M12 1.5v2.2M12 20.3v2.2M4.2 4.2l1.6 1.6M18.2 18.2l1.6 1.6M1.5 12h2.2M20.3 12h2.2M4.2 19.8l1.6-1.6M18.2 5.8l1.6-1.6"/>'],
    "moon": ['<path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"/>'],
    "monitor": ['<rect x="2" y="3.5" width="20" height="13.5" rx="2.5"/><path d="M8 21h8M12 17v4"/>'],
    "activity": ['<path d="M22 12h-4l-3 8L9 4l-3 8H2"/>'],
    "gauge": ['<path d="M20.5 17a10 10 0 1 0-17 0"/><path d="M12 14.5 16 9"/>'],
    "download": ['<path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><path d="m7 10 5 5 5-5"/>'
                 '<path d="M12 15V3"/>'],
    "upload-cloud": ['<path d="M16 16l-4-4-4 4"/><path d="M12 12v9"/>'
                     '<path d="M20.4 18.4A5 5 0 0 0 18 9h-1.3A8 8 0 1 0 3 16.2"/>'],
    "refresh": ['<path d="M23 4v6h-6"/><path d="M1 20v-6h6"/>'
                '<path d="M3.5 9a9 9 0 0 1 14.9-3.4L23 10"/><path d="M1 14l4.6 4.4A9 9 0 0 0 20.5 15"/>'],
    "rotate": ['<path d="M21 12a9 9 0 1 1-3-6.7"/><path d="M21 3v6h-6"/>'],
    "sliders": ['<path d="M4 21v-7M4 10V3M12 21v-9M12 8V3M20 21v-5M20 12V3"/>'
                '<path d="M1 14h6M9 8h6M17 16h6"/>'],
    "settings": ['<circle cx="12" cy="12" r="3.2"/>'
                 '<path d="M19.4 15a1.7 1.7 0 0 0 .3 1.9l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-2.9 1.2 2 2 0 1 1-4 0 1.7 1.7 0 0 0-2.9-1.2l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1A1.7 1.7 0 0 0 3 15a2 2 0 1 1 0-4 1.7 1.7 0 0 0 1.5-2.9l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1A1.7 1.7 0 0 0 10 4.1a2 2 0 1 1 4 0 1.7 1.7 0 0 0 2.9 1.2l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1A1.7 1.7 0 0 0 21 11a2 2 0 1 1 0 4z"/>'],
    "check": ['<path d="m4 12.5 5 5L20 6.5"/>'],
    "check-circle": ['<circle cx="12" cy="12" r="9.2"/><path d="m8 12.3 2.8 2.8L16 9.5"/>'],
    "close": ['<path d="M18 6 6 18M6 6l12 12"/>'],
    "close-circle": ['<circle cx="12" cy="12" r="9.2"/><path d="M15 9l-6 6M9 9l6 6"/>'],
    "alert": ['<path d="M10.3 3.9 1.9 18a2 2 0 0 0 1.7 3h16.8a2 2 0 0 0 1.7-3L13.7 3.9a2 2 0 0 0-3.4 0z"/>'
               '<path d="M12 9.5v4M12 17.2h.01"/>'],
    "info": ['<circle cx="12" cy="12" r="9.2"/><path d="M12 16.5v-4.5M12 8h.01"/>'],
    "help": ['<circle cx="12" cy="12" r="9.2"/><path d="M9.1 9.3a3 3 0 0 1 5.8 1c0 2-3 3-3 3"/>'
             '<path d="M12 17h.01"/>'],
    "wifi": ['<path d="M5 12.6a11 11 0 0 1 14 0"/><path d="M1.4 9a16 16 0 0 1 21.2 0"/>'
             '<path d="M8.5 16.1a6 6 0 0 1 7 0"/><path d="M12 20h.01"/>'],
    "globe": ['<circle cx="12" cy="12" r="9.2"/><path d="M3 12h18"/>'
              '<path d="M12 2.8a15 15 0 0 1 0 18.4 15 15 0 0 1 0-18.4z"/>'],
    "lock": ['<rect x="4" y="10.5" width="16" height="10.5" rx="2.5"/>'
             '<path d="M8 10.5V7.5a4 4 0 0 1 8 0v3"/>'],
    "send": ['<path d="M22 2 11 13"/><path d="M22 2l-7 20-4-9-9-4 20-7z"/>'],
    "play": ['<path d="M22.5 7.4a2.8 2.8 0 0 0-1.9-2C18.9 5 12 5 12 5s-6.9 0-8.6.45a2.8 2.8 0 0 0-1.9 1.95A29 29 0 0 0 1 12.3a29 29 0 0 0 .46 4.9A2.8 2.8 0 0 0 3.4 19.1C5.1 19.55 12 19.55 12 19.55s6.9 0 8.6-.45a2.8 2.8 0 0 0 1.9-1.9 29 29 0 0 0 .46-4.9 29 29 0 0 0-.46-4.9z"/>'
              '<path d="M9.8 15.2V9.4l5.6 2.9-5.6 2.9z"/>'],
    "gamepad": ['<path d="M17.3 5.5H6.7a4 4 0 0 0-4 3.6c-.1.9-.3 2.9-.3 4.4 0 1.5.5 2 1.5 2 1.2 0 1.6-.9 2-2 .3-.9.7-1.5 1.5-1.5h9.2c.8 0 1.2.6 1.5 1.5.4 1.1.8 2 2 2 1 0 1.5-.5 1.5-2 0-1.5-.2-3.5-.3-4.4a4 4 0 0 0-4-3.6z"/>'
                 '<path d="M6.5 11.2h4M8.5 9.2v4"/><circle cx="15.5" cy="10.3" r=".9"/><circle cx="18" cy="12.8" r=".9"/>'],
    "list": ['<path d="M8 6h13M8 12h13M8 18h13M3.5 6h.01M3.5 12h.01M3.5 18h.01"/>'],
    "terminal": ['<path d="M4 17l6-6-6-6"/><path d="M12 19h8"/>'],
    "layers": ['<path d="M12 2.5 2.5 7.5 12 12.5l9.5-5z"/><path d="M2.5 16.5 12 21.5l9.5-5"/>'
               '<path d="M2.5 12 12 17l9.5-5"/>'],
    "droplet": ['<path d="M12 2.7 6.4 9.2a8 8 0 1 0 11.2 0z"/>'],
    "target": ['<circle cx="12" cy="12" r="9.2"/><circle cx="12" cy="12" r="5"/>'
               '<circle cx="12" cy="12" r="1.2"/>'],
    "clock": ['<circle cx="12" cy="12" r="9.2"/><path d="M12 6.8V12l3.6 2.2"/>'],
    "chart": ['<path d="M6 20v-5M12 20V6M18 20v-9"/>'],
    "cloud": ['<path d="M17.5 19H7a4.5 4.5 0 0 1-.6-9A6 6 0 0 1 18 9.5a4.8 4.8 0 0 1-.5 9.5z"/>'],
    "plug": ['<path d="M9 2v6M15 2v6"/><path d="M6 8h12v3a6 6 0 0 1-6 6 6 6 0 0 1-6-6z"/>'
             '<path d="M12 17v5"/>'],
    "rocket": ['<path d="M5 15c-1.5 1.5-2 5-2 5s3.5-.5 5-2"/>'
               '<path d="M14.5 3.5c3-1 6 0 6 0s1 3 0 6c-1.2 3.4-4.5 6.5-8 8.5L8 13.5C10 10 13.1 6.7 14.5 3.5z"/>'
               '<circle cx="15" cy="9" r="1.6"/>'],
    "trash": ['<path d="M3.5 6.5h17"/><path d="M8.5 6.5v-2a1.5 1.5 0 0 1 1.5-1.5h4a1.5 1.5 0 0 1 1.5 1.5v2"/>'
              '<path d="M18.5 6.5 17.6 20a2 2 0 0 1-2 1.9H8.4a2 2 0 0 1-2-1.9L5.5 6.5"/>'],
    "external": ['<path d="M18 13.5V19a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h5.5"/>'
                 '<path d="M15 3h6v6"/><path d="M10.5 13.5 21 3"/>'],
    "chevron-down": ['<path d="m6 9 6 6 6-6"/>'],
    "chevron-right": ['<path d="m9 6 6 6-6 6"/>'],
    "chevron-left": ['<path d="m15 6-6 6 6 6"/>'],
    "plus": ['<path d="M12 5v14M5 12h14"/>'],
    "minus": ['<path d="M5 12h14"/>'],
    "eye": ['<path d="M1.5 12S5.5 5 12 5s10.5 7 10.5 7-4 7-10.5 7S1.5 12 1.5 12z"/>'
            '<circle cx="12" cy="12" r="3"/>'],
    "key": ['<circle cx="7.5" cy="15.5" r="4"/><path d="m10.5 12.5 8-8"/><path d="m15.5 6.5 3 3"/>'
            '<path d="m18.5 3.5 2 2"/>'],
    "user": ['<path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"/><circle cx="12" cy="7.5" r="4"/>'],
    "flask": ['<path d="M9 3h6"/><path d="M10 3v6.5L4.6 18A2 2 0 0 0 6.3 21h11.4a2 2 0 0 0 1.7-3L14 9.5V3"/>'
              '<path d="M7.5 15h9"/>'],
    "copy": ['<rect x="9" y="9" width="12" height="12" rx="2.5"/>'
             '<path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"/>'],
    "anchor": ['<circle cx="12" cy="5" r="2.6"/><path d="M12 7.6V21"/><path d="M4 14a8 8 0 0 0 16 0"/>'],
    "compass": ['<circle cx="12" cy="12" r="9.2"/><path d="m15.8 8.2-2 6-6 2.6 2-6 6-2.6z"/>'],
    "minimize": ['<path d="M5 12h14"/>'],
    "maximize": ['<rect x="3.5" y="3.5" width="17" height="17" rx="2.5"/>'],
}


def svg_markup(name: str, color: str = "#ffffff", width: float = 1.8,
               fill: bool = False) -> str:
    """Собирает SVG-документ для иконки."""
    body = "".join(ICONS.get(name, ICONS["info"]))
    paint = (f'fill="{color}" stroke="{color}" stroke-width="{width * 0.7}" '
             if fill else
             f'fill="none" stroke="{color}" stroke-width="{width}" ')
    return (
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" '
        f'width="24" height="24" stroke-linecap="round" stroke-linejoin="round" {paint}>'
        f"{body}</svg>"
    )


@lru_cache(maxsize=512)
def icon_pixmap(name: str, size: int, color: str, width: float = 1.8) -> QPixmap:
    """Возвращает (кэшированную) иконку нужного размера и цвета."""
    size = max(8, int(size))
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    renderer = QSvgRenderer(QByteArray(svg_markup(name, color, width).encode("utf-8")))
    painter = QPainter(pm)
    painter.setRenderHint(QPainter.Antialiasing, True)
    painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
    renderer.render(painter, QRectF(0, 0, size, size))
    painter.end()
    return pm


def app_icon(size: int = 64, accent: str = "#d5ff45", dark: bool = True) -> QPixmap:
    """Иконка приложения: щит с молнией на фирменном фоне."""
    from PySide6.QtGui import QColor, QLinearGradient, QPainterPath, QBrush, QPen

    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing, True)

    bg = QLinearGradient(0, 0, size, size)
    if dark:
        bg.setColorAt(0.0, QColor("#16241f"))
        bg.setColorAt(1.0, QColor("#0a100e"))
    else:
        bg.setColorAt(0.0, QColor("#ffffff"))
        bg.setColorAt(1.0, QColor("#e8efeb"))
    path = QPainterPath()
    r = size * 0.28
    path.addRoundedRect(QRectF(0, 0, size, size), r, r)
    p.fillPath(path, QBrush(bg))
    p.setPen(QPen(QColor(accent), max(1.0, size * 0.025)))
    p.setBrush(Qt.BrushStyle.NoBrush)
    p.drawPath(path)
    p.end()

    shield = icon_pixmap("shield-check", int(size * 0.52), accent, 1.7)
    bolt = icon_pixmap("zap", int(size * 0.30), "#ffffff" if not dark else "#0c1310", 1.9)
    p2 = QPainter(pm)
    p2.setRenderHint(QPainter.Antialiasing, True)
    off = (size - shield.width()) // 2
    p2.drawPixmap(off, int(size * 0.16), shield)
    p2.drawPixmap(int(size * 0.44), int(size * 0.36), bolt)
    p2.end()
    return pm
