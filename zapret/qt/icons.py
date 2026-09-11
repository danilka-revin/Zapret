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
    # --- сервисы и группы сайтов ---
    "youtube": ['<path d="M22.5 7.4a2.8 2.8 0 0 0-1.9-2C18.9 5 12 5 12 5s-6.9 0-8.6.45a2.8 2.8 0 0 0-1.9 1.95A29 29 0 0 0 1 12.3a29 29 0 0 0 .46 4.9A2.8 2.8 0 0 0 3.4 19.1C5.1 19.55 12 19.55 12 19.55s6.9 0 8.6-.45a2.8 2.8 0 0 0 1.9-1.9 29 29 0 0 0 .46-4.9 29 29 0 0 0-.46-4.9z"/>'
                '<path d="M9.8 15.2V9.4l5.6 2.9-5.6 2.9z"/>'],
    "twitch": ['<path d="M21 2H3v16h5v4l4-4h5l4-4V2z"/><path d="M11 11V7M16 11V7"/>'],
    "instagram": ['<rect x="2.5" y="2.5" width="19" height="19" rx="5"/>'
                  '<circle cx="12" cy="12" r="4"/><path d="M17.3 6.7h.01"/>'],
    "twitter": ['<path d="M23 3a10.9 10.9 0 0 1-3.14 1.53 4.48 4.48 0 0 0-7.86 3v1A10.66 10.66 0 0 1 3 4s-4 9 5 13a11.64 11.64 0 0 1-7 2c9 5 20 0 20-11.5a4.5 4.5 0 0 0-.08-.83A7.72 7.72 0 0 0 23 3z"/>'],
    "message-circle": ['<path d="M21 11.5a8.5 8.5 0 0 1-12.1 7.7L3 21l1.9-5.7A8.5 8.5 0 1 1 21 11.5z"/>'],
    "music": ['<path d="M9 18V5l12-2v13"/><circle cx="6" cy="18" r="3"/><circle cx="18" cy="16" r="3"/>'],
    "headphones": ['<path d="M3 18v-6a9 9 0 0 1 18 0v6"/>'
                   '<path d="M21 19a2 2 0 0 1-2 2h-1a2 2 0 0 1-2-2v-3a2 2 0 0 1 2-2h3z"/>'
                   '<path d="M3 19a2 2 0 0 0 2 2h1a2 2 0 0 0 2-2v-3a2 2 0 0 0-2-2H3z"/>'],
    "film": ['<rect x="2.5" y="3" width="19" height="18" rx="2.5"/>'
             '<path d="M7 3v18M17 3v18M2.5 9h19M2.5 15h19"/>'],
    "search": ['<circle cx="11" cy="11" r="7.5"/><path d="m20.5 20.5-4.9-4.9"/>'],
    "users": ['<path d="M16 21v-2a4 4 0 0 0-4-4H6a4 4 0 0 0-4 4v2"/><circle cx="9" cy="7" r="4"/>'
              '<path d="M22.5 21v-2a4 4 0 0 0-3-3.87"/><path d="M16.5 3.13a4 4 0 0 1 0 7.75"/>'],
    "box": ['<path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"/>'
            '<path d="m3.3 7 8.7 5 8.7-5"/><path d="M12 22V12"/>'],
    "star": ['<path d="m12 3 2.7 5.9 6.3.7-4.7 4.3 1.3 6.1L12 17.2 6.4 20l1.3-6.1L3 9.6l6.3-.7L12 3z"/>'],
    # --- новые: сеть, система, действия ---
    "wifi-off": ['<path d="m2 2 20 20"/><path d="M8.5 16.1a6 6 0 0 1 7 0"/>'
                '<path d="M5 12.6a11 11 0 0 1 5.2-2.9M19 12.6a11 11 0 0 0-5.2-2.9"/>'
                '<path d="M12 20h.01"/>'],
    "server": ['<rect x="2.5" y="3" width="19" height="7" rx="2"/>'
              '<rect x="2.5" y="14" width="19" height="7" rx="2"/>'
              '<path d="M6.5 6.5h.01M6.5 17.5h.01"/>'],
    "cpu": ['<rect x="5" y="5" width="14" height="14" rx="2"/>'
           '<rect x="9" y="9" width="6" height="6"/>'
           '<path d="M9 2v3M15 2v3M9 19v3M15 19v3M2 9h3M2 15h3M19 9h3M19 15h3"/>'],
    "timer": ['<circle cx="12" cy="13" r="8"/>'
             '<path d="M12 9v4l2.5 2.5M9 2h6"/>'],
    "history": ['<path d="M3 12a9 9 0 1 0 3-6.7"/>'
               '<path d="M3 4v5h5M12 7v5l3.5 2"/>'],
    "bookmark": ['<path d="M19 21l-7-5-7 5V5a2 2 0 0 1 2-2h10a2 2 0 0 1 2 2v16z"/>'],
    "folder": ['<path d="M22 19a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h5l2 3h9a2 2 0 0 1 2 2v11z"/>'],
    "file": ['<path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"/>'
            '<path d="M14 2v6h6M16 13H8M16 17H8"/>'],
    "bell": ['<path d="M18 8a6 6 0 0 0-12 0c0 7-3 9-3 9h18s-3-2-3-9"/>'
            '<path d="M13.7 21a2 2 0 0 1-3.4 0"/>'],
    "bell-off": ['<path d="M13.7 21a2 2 0 0 1-3.4 0"/><path d="m2 2 20 20"/>'
                '<path d="M8.7 3A6 6 0 0 1 18 8c0 4-1.3 6.4-2.4 7.7M6 17.3C4.8 16 4 14.3 4 12"/>'],
    "filter": ['<path d="M22 3H2l8 9.5V19l4 2v-8.5L22 3z"/>'],
    "grid": ['<rect x="3" y="3" width="7" height="7" rx="1.5"/>'
            '<rect x="14" y="3" width="7" height="7" rx="1.5"/>'
            '<rect x="3" y="14" width="7" height="7" rx="1.5"/>'
            '<rect x="14" y="14" width="7" height="7" rx="1.5"/>'],
    "edit": ['<path d="M17 3a2.8 2.8 0 1 1 4 4L7.5 20.5 2 22l1.5-5.5L17 3z"/>'],
    "save": ['<path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/>'
            '<path d="M17 21v-8H7v8M7 3v5h8"/>'],
    "upload": ['<path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>'
              '<path d="m17 8-5-5-5 5M12 3v12"/>'],
    "database": ['<ellipse cx="12" cy="5" rx="8.5" ry="3"/>'
                '<path d="M3.5 5v14c0 1.7 3.8 3 8.5 3s8.5-1.3 8.5-3V5"/>'
                '<path d="M3.5 12c0 1.7 3.8 3 8.5 3s8.5-1.3 8.5-3"/>'],
    "flame": ['<path d="M8.5 14.5A2.5 2.5 0 0 0 11 12c0-1.4-.5-2.5-1-3.5-.5-1.2-1-2.4-1-4.5 3 1 5.5 3.5 7 6.5.9 1.7 1.5 3.6 1.5 5a7 7 0 1 1-14 0c0-1.5.5-3 1.3-4.2.6 1 1.2 1.8 2.2 2.7z"/>'],
    "book": ['<path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20"/>'
            '<path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z"/>'],
    "github": ['<path d="M9 19c-5 1.5-5-2.5-7-3m14 6v-3.9a3.4 3.4 0 0 0-1-2.6c3.3-.4 6.8-1.7 6.8-7.4a5.7 5.7 0 0 0-1.5-4 5.3 5.3 0 0 0 0-4s-1.2-.5-4 1.5a12.6 12.6 0 0 0-6.6 0C6.5 2.8 5.3 3.3 5.3 3.3a5.3 5.3 0 0 0 0 4 5.7 5.7 0 0 0-1.5 4c0 5.7 3.5 7 6.8 7.4a3.4 3.4 0 0 0-1 2.6V22"/>'],
    "video": ['<path d="m23 7-7 5 7 5V7z"/>'
             '<rect x="1" y="5" width="15" height="14" rx="2.5"/>'],
    "mic": ['<rect x="9" y="2" width="6" height="12" rx="3"/>'
           '<path d="M5 10a7 7 0 0 0 14 0M12 19v3"/>'],
    "phone": ['<path d="M22 16.9v3a2 2 0 0 1-2.2 2 19.8 19.8 0 0 1-8.6-3.1 19.5 19.5 0 0 1-6-6A19.8 19.8 0 0 1 2.1 4.2 2 2 0 0 1 4.1 2h3a2 2 0 0 1 2 1.7c.1 1 .4 2 .7 2.8a2 2 0 0 1-.5 2.1L8.1 9.9a16 16 0 0 0 6 6l1.3-1.2a2 2 0 0 1 2.1-.5c.9.3 1.9.6 2.8.7a2 2 0 0 1 1.7 2z"/>'],
    "x-circle": ['<circle cx="12" cy="12" r="9.2"/><path d="M15 9l-6 6M9 9l6 6"/>'],
    "plus-circle": ['<circle cx="12" cy="12" r="9.2"/><path d="M12 8v8M8 12h8"/>'],
    "pause": ['<rect x="6" y="4" width="4" height="16" rx="1.2"/>'
             '<rect x="14" y="4" width="4" height="16" rx="1.2"/>'],
    "stop": ['<circle cx="12" cy="12" r="9.2"/><rect x="9" y="9" width="6" height="6" rx="1"/>'],
    "award": ['<circle cx="12" cy="8.5" r="5.5"/><path d="M8.5 13.5 7 22l5-3 5 3-1.5-8.5"/>'],
    "trending": ['<path d="m23 6-9.5 9.5-5-5L1 18"/>'
                '<path d="M17 6h6v6"/>'],
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
    """Иконка приложения: щит с молнией на фирменной подложке."""
    from PySide6.QtGui import QColor, QLinearGradient, QPainterPath, QBrush, QPen

    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing, True)
    p.setRenderHint(QPainter.SmoothPixmapTransform, True)

    # Подложка со скруглением
    bg = QLinearGradient(0, 0, size, size)
    if dark:
        bg.setColorAt(0.0, QColor("#1b2a24"))
        bg.setColorAt(1.0, QColor("#0a100e"))
    else:
        bg.setColorAt(0.0, QColor("#ffffff"))
        bg.setColorAt(1.0, QColor("#e6ece9"))
    radius = size * 0.26
    plate = QPainterPath()
    plate.addRoundedRect(QRectF(0, 0, size, size), radius, radius)
    p.fillPath(plate, QBrush(bg))
    p.setPen(QPen(QColor(accent), max(1.0, size * 0.02)))
    p.setBrush(Qt.BrushStyle.NoBrush)
    p.drawPath(plate)

    # Щит: контур акцентом
    scale = size / 24.0
    p.translate(size / 2, size / 2)
    p.scale(scale, scale)
    p.setPen(QPen(QColor(accent), 2.1, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap,
                  Qt.PenJoinStyle.RoundJoin))
    shield = QPainterPath()
    shield.moveTo(0.6, -9.4)
    shield.lineTo(-7.8, -6.7)
    shield.lineTo(-7.8, 0.6)
    shield.cubicTo(-7.8, 6.4, -3.6, 9.6, 0.6, 11.4)
    shield.cubicTo(4.8, 9.6, 9.0, 6.4, 9.0, 0.6)
    shield.lineTo(9.0, -6.7)
    shield.closeSubpath()
    p.drawPath(shield)

    # Молния внутри щита
    bolt = QPainterPath()
    bolt.moveTo(2.4, -6.6)
    bolt.lineTo(-3.4, 0.6)
    bolt.lineTo(-0.1, 0.6)
    bolt.lineTo(-1.6, 6.6)
    bolt.lineTo(4.2, -1.0)
    bolt.lineTo(0.7, -1.0)
    bolt.closeSubpath()
    p.fillPath(bolt, QBrush(QColor(accent)))
    p.end()
    return pm
