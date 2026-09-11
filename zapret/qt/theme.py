"""
Тема и кастомизация интерфейса Zapret Control (Qt6).

Модель настроек повторяет подход zmk-videoanalytics: тема (светлая/тёмная/
системная), акцентный цвет, стекло, тонировка, плотность, углы, анимации и
размер текста. Все параметры сохраняются в config.json и применяются мгновенно
без перезапуска приложения.
"""

from __future__ import annotations

import re
from dataclasses import asdict, dataclass, field
from typing import TYPE_CHECKING

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QColor, QFontDatabase, QPalette

if TYPE_CHECKING:  # pragma: no cover
    from PySide6.QtWidgets import QApplication

# ---------------------------------------------------------------------------
# Пресеты акцентов (как круглые свотчи в настройках zmk)
# ---------------------------------------------------------------------------

ACCENTS: list[tuple[str, str, str]] = [
    ("lime", "#d5ff45", "Лайм"),
    ("sky", "#69a7ff", "Небо"),
    ("orange", "#ffad4d", "Закат"),
    ("violet", "#a98cff", "Фиалка"),
    ("mint", "#34e0a1", "Мята"),
    ("rose", "#ff6b8b", "Роза"),
]

PRESETS: list[tuple[str, str, dict]] = [
    ("glass", "Тёмное стекло", {"mode": "dark", "accent": "#69a7ff", "glass": "vivid",
                                "radius": "soft", "density": "comfortable"}),
    ("lime", "Как в zmk", {"mode": "light", "accent": "#d5ff45", "glass": "soft",
                           "radius": "soft", "density": "comfortable"}),
    ("night", "Ночной дозор", {"mode": "dark", "accent": "#34e0a1", "glass": "soft",
                               "radius": "square", "density": "compact"}),
    ("contrast", "Контраст", {"mode": "dark", "accent": "#ffad4d", "glass": "off",
                              "radius": "square", "density": "comfortable"}),
]

HEX_RE = re.compile(r"^#(?:[0-9a-fA-F]{3}|[0-9a-fA-F]{6})$")


def normalize_hex(value: str, fallback: str = "#d5ff45") -> str:
    """Приводит цвет к #rrggbb, отбрасывая мусор из конфига."""
    if not isinstance(value, str) or not HEX_RE.match(value.strip()):
        return fallback
    v = value.strip()
    if len(v) == 4:
        v = "#" + "".join(ch * 2 for ch in v[1:])
    return v.lower()


def _rgb(hex_color: str) -> tuple[int, int, int]:
    h = normalize_hex(hex_color).lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def mix(color_a: str, color_b: str, t: float) -> str:
    """Смешивает два цвета: t=0 → color_a, t=1 → color_b."""
    ar, ag, ab = _rgb(color_a)
    br, bg, bb = _rgb(color_b)
    return "#%02x%02x%02x" % (
        round(ar + (br - ar) * t),
        round(ag + (bg - ag) * t),
        round(ab + (bb - ab) * t),
    )


def with_alpha(hex_color: str, alpha: float) -> QColor:
    r, g, b = _rgb(hex_color)
    color = QColor(r, g, b)
    color.setAlphaF(max(0.0, min(1.0, alpha)))
    return color


def luminance(hex_color: str) -> float:
    """Относительная яркость 0..1 — нужна для контрастного текста на акценте."""
    r, g, b = _rgb(hex_color)
    return (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0


def contrast_text(bg: str, light: str = "#ffffff", dark: str = "#101510") -> str:
    return dark if luminance(bg) > 0.55 else light


# ---------------------------------------------------------------------------
# Настройки кастомизации
# ---------------------------------------------------------------------------

@dataclass
class UISettings:
    mode: str = "dark"            # light | dark | system
    accent: str = "#d5ff45"
    glass: str = "vivid"           # off | soft | vivid
    glass_tint: str = "auto"      # auto | cool | warm
    density: str = "comfortable"  # comfortable | compact
    radius: str = "soft"          # soft | square
    motion: str = "full"          # full | reduced | off
    font_scale: str = "large"    # small | normal | large
    orbs: bool = True             # цветные «пятна» на фоне
    orbs_intensity: int = 140     # 0..160 (%)

    # поведение: минимум настроек, максимум автономики
    autopilot: bool = True        # сам подбирает и проверяет стратегию
    autostart_protection: bool = True   # включать защиту при старте приложения
    start_minimized: bool = True        # при автозапуске — сразу в трей
    auto_recover: bool = True           # сама поднимает защиту, если упала
    tray: bool = True                   # иконка в системном трее
    notifications: bool = True          # уведомления о смене состояния
    watch_services: list = field(default_factory=lambda: ["youtube", "discord", "telegram"])
    check_interval: int = 60      # период фоновой проверки сервисов, сек

    def sanitized(self) -> "UISettings":
        s = UISettings()
        s.mode = self.mode if self.mode in ("light", "dark", "system") else "dark"
        s.accent = normalize_hex(self.accent)
        s.glass = self.glass if self.glass in ("off", "soft", "vivid") else "soft"
        s.glass_tint = self.glass_tint if self.glass_tint in ("auto", "cool", "warm") else "auto"
        s.density = self.density if self.density in ("comfortable", "compact") else "comfortable"
        s.radius = self.radius if self.radius in ("soft", "square") else "soft"
        s.motion = self.motion if self.motion in ("full", "reduced", "off") else "full"
        s.font_scale = self.font_scale if self.font_scale in ("small", "normal", "large") else "normal"
        s.orbs = bool(self.orbs)
        try:
            s.orbs_intensity = int(self.orbs_intensity)
        except (TypeError, ValueError):
            s.orbs_intensity = 100
        s.orbs_intensity = max(0, min(160, s.orbs_intensity))
        s.autopilot = bool(self.autopilot)
        s.autostart_protection = bool(self.autostart_protection)
        s.start_minimized = bool(self.start_minimized)
        s.auto_recover = bool(self.auto_recover)
        s.tray = bool(self.tray)
        s.notifications = bool(self.notifications)
        watch = [str(x) for x in (self.watch_services or []) if isinstance(x, (str, int))]
        s.watch_services = watch or ["youtube", "discord", "telegram"]
        try:
            s.check_interval = max(20, min(900, int(self.check_interval)))
        except (TypeError, ValueError):
            s.check_interval = 60
        return s

    def to_dict(self) -> dict:
        return asdict(self.sanitized())

    @classmethod
    def from_dict(cls, data: dict | None) -> "UISettings":
        data = data or {}
        s = cls()
        for key in asdict(s):
            if key in data:
                setattr(s, key, data[key])
        return s.sanitized()


# ---------------------------------------------------------------------------
# Палитра — вычисляется из настроек
# ---------------------------------------------------------------------------

class Palette:
    """Набор цветов и размеров, производный от UISettings."""

    def __init__(self, settings: UISettings):
        self.s = settings
        mode = settings.mode
        if mode == "system":
            mode = "dark" if _system_is_dark() else "light"
        self.dark = mode == "dark"

        accent = settings.accent
        self.accent = accent
        self.accent_rgb = _rgb(accent)
        self.on_accent = contrast_text(accent)
        # В светлой теме яркий акцент нечитаем на белом — используем затемнённый
        self.accent_text = accent if self.dark else mix(accent, "#0b1512", 0.42)

        if self.dark:
            self.page = "#0b100e"
            self.page_alt = "#070b0a"
            self.surface = "#131a17"
            self.surface_2 = "#182220"
            self.surface_3 = "#1e2a26"
            self.line = "#25332e"
            self.line_strong = "#33463f"
            self.text = "#eef4f0"
            self.muted = "#8ea49a"
            self.shadow = "#000000"
        else:
            self.page = "#f2f5f3"
            self.page_alt = "#e7ece9"
            self.surface = "#ffffff"
            self.surface_2 = "#f6f9f7"
            self.surface_3 = "#eef3f0"
            self.line = "#e0e7e3"
            self.line_strong = "#cdd8d2"
            self.text = "#16211c"
            self.muted = "#7d8d86"
            self.shadow = "#22322c"

        self.good = "#34e0a1" if self.dark else "#12a56f"
        self.warn = "#ffc061" if self.dark else "#c8801b"
        self.bad = "#ff6b7f" if self.dark else "#d8445f"
        self.idle = self.muted

        # Стекло
        glass_alpha = {"off": 0.0, "soft": 0.55, "vivid": 0.95}[settings.glass]
        self.glass_alpha = glass_alpha
        if settings.glass_tint == "cool":
            tint = mix(accent, "#5dade2", 1.0)
        elif settings.glass_tint == "warm":
            tint = mix(accent, "#ffb35c", 1.0)
        else:
            tint = accent
        self.tint = tint
        self.card_border = with_alpha(self.text if self.dark else "#ffffff", 0.10 if self.dark else 0.5)
        self.card_top_line = with_alpha("#ffffff", 0.10 if self.dark else 0.9)

        # Орбы фона
        self.orb_colors = [
            mix(accent, "#ffffff", 0.0),
            "#5dade2",
            mix(accent, "#a98cff", 0.5),
        ]

        # Метрики
        compact = settings.density == "compact"
        scale = {"small": 0.92, "normal": 1.0, "large": 1.35}[settings.font_scale]
        self.scale = scale
        self.pad = int((14 if compact else 22) * scale)
        self.gap = int((12 if compact else 18) * scale)
        base = 10.5 if compact else 13.0
        self.font_xs = max(7.0, base * 0.78 * scale)
        self.font_sm = base * 0.9 * scale
        self.font_md = base * scale
        self.font_lg = base * 1.22 * scale
        self.font_xl = base * 1.6 * scale
        self.font_hero = base * 2.6 * scale

        if settings.radius == "square":
            self.r_sm, self.r_md, self.r_lg, self.r_card = 4, 6, 8, 10
        else:
            self.r_sm, self.r_md, self.r_lg, self.r_card = 10, 14, 20, 22

    # -- производные цвета -------------------------------------------------

    def alpha(self, color: str, a: float) -> QColor:
        return with_alpha(color, a)

    def surface_at(self, a: float) -> QColor:
        return with_alpha(self.surface, a)

    def accent_soft(self, a: float = 0.16) -> QColor:
        return with_alpha(self.accent, a)

    def muted_alpha(self, a: float = 0.5) -> QColor:
        return with_alpha(self.muted, a)

    # -- анимации ----------------------------------------------------------

    @property
    def anim_ms(self) -> int:
        return {"full": 300, "reduced": 180, "off": 0}[self.s.motion]

    @property
    def animated(self) -> bool:
        return self.s.motion != "off"


def _system_is_dark() -> bool:
    """Определяет системную тему без обращения к приватным API."""
    try:
        from PySide6.QtGui import QGuiApplication

        app = QGuiApplication.instance()
        if app is not None:
            scheme = app.styleHints().colorScheme()
            if scheme == Qt.ColorScheme.Dark:
                return True
            if scheme == Qt.ColorScheme.Light:
                return False
    except Exception:  # noqa: BLE001
        pass
    return True


# ---------------------------------------------------------------------------
# Системная палитра Qt
# ---------------------------------------------------------------------------

def build_qpalette(pal: Palette) -> QPalette:
    """Собирает QPalette из нашей темы.

    Без этого стандартные элементы Qt (области прокрутки, всплывающие меню,
    подсказки, диалоги) рисуются системными цветами — в тёмной теме вокруг
    карточек оставался светлый фон.
    """
    qp = QPalette()
    surface = QColor(pal.surface)
    surface_2 = QColor(pal.surface_2)
    surface_3 = QColor(pal.surface_3)
    text = QColor(pal.text)
    muted = QColor(pal.muted)
    accent = QColor(pal.accent)
    base = QColor(pal.page)
    shadow = QColor(pal.shadow)

    qp.setColor(QPalette.ColorRole.Window, base)
    qp.setColor(QPalette.ColorRole.WindowText, text)
    qp.setColor(QPalette.ColorRole.Base, surface)
    qp.setColor(QPalette.ColorRole.AlternateBase, surface_2)
    qp.setColor(QPalette.ColorRole.ToolTipBase, surface_3)
    qp.setColor(QPalette.ColorRole.ToolTipText, text)
    qp.setColor(QPalette.ColorRole.Text, text)
    qp.setColor(QPalette.ColorRole.Button, surface_2)
    qp.setColor(QPalette.ColorRole.ButtonText, text)
    qp.setColor(QPalette.ColorRole.BrightText, QColor("#ffffff"))
    qp.setColor(QPalette.ColorRole.Light, surface_3)
    qp.setColor(QPalette.ColorRole.Midlight, surface_2)
    qp.setColor(QPalette.ColorRole.Mid, surface)
    qp.setColor(QPalette.ColorRole.Dark, surface_3)
    qp.setColor(QPalette.ColorRole.Shadow, shadow)
    qp.setColor(QPalette.ColorRole.Highlight, accent)
    qp.setColor(QPalette.ColorRole.HighlightedText, QColor(pal.on_accent))
    qp.setColor(QPalette.ColorRole.Link, accent)
    qp.setColor(QPalette.ColorRole.LinkVisited, accent)
    qp.setColor(QPalette.ColorRole.PlaceholderText, muted)
    for role in (QPalette.ColorRole.WindowText, QPalette.ColorRole.Text,
                 QPalette.ColorRole.ButtonText):
        qp.setColor(QPalette.ColorGroup.Disabled, role, muted)
    return qp


def style_sheet(pal: "Palette") -> str:
    """Минимальный QSS: убирает системные светлые подложки у прокрутки и меню."""
    return f"""
QWidget {{ color: {pal.text}; }}
QScrollArea, QScrollArea > QWidget#qt_scrollarea_viewport,
QAbstractScrollArea, QAbstractScrollArea > QWidget#qt_scrollarea_viewport {{
    background: transparent; border: none;
}}
QToolTip {{
    background: {pal.surface_3}; color: {pal.text};
    border: 1px solid {pal.line_strong}; padding: 6px 8px; border-radius: 8px;
}}
QMenu {{
    background: {pal.surface}; color: {pal.text};
    border: 1px solid {pal.line_strong}; border-radius: 10px; padding: 6px;
}}
QMenu::item {{ padding: 7px 14px; border-radius: 7px; }}
QMenu::item:selected {{ background: {pal.surface_3}; color: {pal.text}; }}
QMenu::separator {{ height: 1px; background: {pal.line}; margin: 5px 8px; }}
QDialog, QMessageBox, QColorDialog {{ background: {pal.surface}; color: {pal.text}; }}
QLineEdit {{
    background: {pal.surface_2}; color: {pal.text};
    border: 1px solid {pal.line_strong}; border-radius: 10px;
    padding: 8px 12px; selection-background-color: {pal.accent};
    selection-color: {pal.on_accent};
}}
QLineEdit:focus {{ border: 1px solid {pal.accent}; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{
    background: {pal.muted}; border-radius: 5px; min-height: 36px; opacity: .5;
}}
QScrollBar::handle:vertical:hover {{ background: {pal.text}; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: transparent; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: {pal.muted}; border-radius: 5px; min-width: 36px; }}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{ background: transparent; }}
"""


def apply_theme_to_app(app: "QApplication", theme: ThemeManager) -> None:
    """Применяет тему (палитра + QSS) ко всему приложению."""
    pal = theme.palette
    app.setPalette(build_qpalette(pal))
    app.setStyleSheet(style_sheet(pal))


# ---------------------------------------------------------------------------
# Шрифты
# ---------------------------------------------------------------------------

FONT_CANDIDATES = [
    "Inter", "SF Pro Display", "Segoe UI Variable Display", "Segoe UI",
    "Manrope", "Ubuntu", "Noto Sans", "Cantarell", "DejaVu Sans",
]


def pick_font_family() -> str:
    try:
        families = set(QFontDatabase.families())
    except Exception:  # noqa: BLE001
        return "Sans Serif"
    for name in FONT_CANDIDATES:
        if name in families:
            return name
    return "Sans Serif"


def pick_mono_family() -> str:
    try:
        families = set(QFontDatabase.families())
    except Exception:  # noqa: BLE001
        return "Monospace"
    for name in ("JetBrains Mono", "Fira Code", "SF Mono", "Cascadia Code",
                 "Ubuntu Mono", "DejaVu Sans Mono", "Noto Sans Mono"):
        if name in families:
            return name
    return "Monospace"


# ---------------------------------------------------------------------------
# Менеджер темы (синглтон на приложение)
# ---------------------------------------------------------------------------

class ThemeManager(QObject):
    """Хранит UISettings, пересчитывает палитру и оповещает виджеты."""

    changed = Signal()

    def __init__(self, settings: UISettings | None = None, parent=None):
        super().__init__(parent)
        self._settings = (settings or UISettings()).sanitized()
        self._palette = Palette(self._settings)
        self.font_family = pick_font_family()
        self.mono_family = pick_mono_family()

    # -- доступ ------------------------------------------------------------

    @property
    def settings(self) -> UISettings:
        return self._settings

    @property
    def palette(self) -> Palette:
        return self._palette

    # -- изменение ---------------------------------------------------------

    def update(self, **changes) -> None:
        data = self._settings.to_dict()
        data.update(changes)
        new = UISettings.from_dict(data)
        if new.to_dict() == self._settings.to_dict():
            return
        self._settings = new
        self._palette = Palette(new)
        self.changed.emit()

    def apply_preset(self, preset_key: str) -> None:
        for key, _label, values in PRESETS:
            if key == preset_key:
                self.update(**values)
                return

    def reset(self) -> None:
        self._settings = UISettings()
        self._palette = Palette(self._settings)
        self.changed.emit()

    def refresh_system_theme(self) -> None:
        if self._settings.mode == "system":
            self._palette = Palette(self._settings)
            self.changed.emit()
