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
from pathlib import Path
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
    ("cyan", "#4de3ff", "Циан"),
    ("gold", "#ffd166", "Золото"),
    ("coral", "#ff7a59", "Коралл"),
    ("azure", "#4d7cff", "Лазурь"),
    ("orchid", "#e08cff", "Орхидея"),
    ("slate", "#9fb3c8", "Графит"),
]

PRESETS: list[tuple[str, str, dict]] = [
    ("aurora", "Аврора", {"mode": "dark", "accent": "#34e0a1", "glass": "vivid",
                          "glass_tint": "auto", "radius": "soft", "density": "comfortable",
                          "motion": "full", "font_scale": "normal", "orbs": True,
                          "orbs_intensity": 140}),
    ("glass", "Тёмное стекло", {"mode": "dark", "accent": "#69a7ff", "glass": "vivid",
                                "glass_tint": "cool", "radius": "soft",
                                "density": "comfortable", "orbs": True,
                                "orbs_intensity": 120}),
    ("lime", "Лайм", {"mode": "light", "accent": "#d5ff45", "glass": "soft",
                      "glass_tint": "auto", "radius": "soft", "density": "comfortable",
                      "orbs": True, "orbs_intensity": 100}),
    ("sunset", "Закат", {"mode": "dark", "accent": "#ffad4d", "glass": "vivid",
                         "glass_tint": "warm", "radius": "soft", "density": "comfortable",
                         "orbs": True, "orbs_intensity": 150}),
    ("ocean", "Океан", {"mode": "dark", "accent": "#4de3ff", "glass": "soft",
                        "glass_tint": "cool", "radius": "soft", "density": "comfortable",
                        "orbs": True, "orbs_intensity": 110}),
    ("rose", "Роза", {"mode": "light", "accent": "#ff6b8b", "glass": "soft",
                      "glass_tint": "warm", "radius": "soft", "density": "comfortable",
                      "orbs": True, "orbs_intensity": 90}),
    ("night", "Ночной дозор", {"mode": "dark", "accent": "#a98cff", "glass": "soft",
                               "glass_tint": "auto", "radius": "square", "density": "compact",
                               "orbs": False, "orbs_intensity": 60}),
    ("contrast", "Контраст", {"mode": "dark", "accent": "#ffd166", "glass": "off",
                              "glass_tint": "auto", "radius": "square",
                              "density": "comfortable", "orbs": False,
                              "orbs_intensity": 0}),
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
            self.page = "#0a0f0e"
            self.page_alt = "#060a09"
            self.surface = "#121917"
            self.surface_2 = "#17211e"
            self.surface_3 = "#1f2c28"
            self.line = "#243230"
            self.line_strong = "#354844"
            self.text = "#f0f5f2"
            self.muted = "#93a8a0"
            self.shadow = "#000000"
        else:
            self.page = "#eef3f1"
            self.page_alt = "#e2e9e6"
            self.surface = "#ffffff"
            self.surface_2 = "#f4f8f6"
            self.surface_3 = "#e9f0ed"
            self.line = "#dde6e2"
            self.line_strong = "#c6d4ce"
            self.text = "#14201b"
            self.muted = "#75867f"
            self.shadow = "#1d2b26"

        self.good = "#2fe39b" if self.dark else "#0da271"
        self.good_soft = mix(self.good, self.surface, 0.82)
        self.warn = "#ffc25e" if self.dark else "#c07c14"
        self.warn_soft = mix(self.warn, self.surface, 0.82)
        self.bad = "#ff6b81" if self.dark else "#d63a58"
        self.bad_soft = mix(self.bad, self.surface, 0.82)
        self.info = "#69a7ff" if self.dark else "#2f6fe0"
        self.info_soft = mix(self.info, self.surface, 0.82)
        self.idle = self.muted

        # Стекло
        glass_alpha = {"off": 0.0, "soft": 0.55, "vivid": 0.92}[settings.glass]
        self.glass_alpha = glass_alpha
        if settings.glass_tint == "cool":
            tint = mix(accent, "#5dade2", 0.55)
        elif settings.glass_tint == "warm":
            tint = mix(accent, "#ffb35c", 0.55)
        else:
            tint = accent
        self.tint = tint
        self.card_border = with_alpha(self.text if self.dark else "#ffffff",
                                      0.12 if self.dark else 0.55)
        self.card_top_line = with_alpha("#ffffff", 0.12 if self.dark else 0.9)

        # Орбы фона: акцент + два гармонирующих оттенка
        self.orb_colors = [
            accent,
            mix(accent, "#5dade2", 0.65),
            mix(accent, "#a98cff", 0.55),
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
    """QSS приложения: системные элементы в цветах темы, без светлых заплаток."""
    return f"""
QWidget {{ color: {pal.text}; }}
QMainWindow, QDialog, QMessageBox, QColorDialog, QFileDialog {{
    background: {pal.page}; color: {pal.text};
}}
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
QMenu::item:disabled {{ color: {pal.muted}; }}
QMenu::separator {{ height: 1px; background: {pal.line}; margin: 5px 8px; }}
QLineEdit, QTextEdit, QPlainTextEdit {{
    background: {pal.surface_2}; color: {pal.text};
    border: 1px solid {pal.line_strong}; border-radius: 10px;
    padding: 8px 12px; selection-background-color: {pal.accent};
    selection-color: {pal.on_accent};
}}
QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus {{ border: 1px solid {pal.accent}; }}
QLineEdit:disabled {{ color: {pal.muted}; }}
QComboBox {{
    background: {pal.surface_2}; color: {pal.text};
    border: 1px solid {pal.line_strong}; border-radius: 10px;
    padding: 7px 12px; min-height: 20px;
}}
QComboBox:focus {{ border: 1px solid {pal.accent}; }}
QComboBox QAbstractItemView {{
    background: {pal.surface}; color: {pal.text};
    border: 1px solid {pal.line_strong}; border-radius: 8px;
    selection-background-color: {pal.surface_3}; selection-color: {pal.text};
    padding: 4px;
}}
QTableWidget {{
    background: {pal.surface}; color: {pal.text};
    border: 1px solid {pal.line}; border-radius: 12px;
    gridline-color: {pal.line}; alternate-background-color: {pal.surface_2};
}}
QTableWidget::item {{ padding: 6px; border: none; }}
QTableWidget::item:selected {{ background: {pal.surface_3}; color: {pal.text}; }}
QHeaderView::section {{
    background: {pal.surface_2}; color: {pal.muted};
    border: none; border-bottom: 1px solid {pal.line_strong};
    padding: 8px; font-weight: bold;
}}
QTabWidget::pane {{ border: 1px solid {pal.line}; border-radius: 12px; background: {pal.surface}; }}
QTabBar::tab {{
    background: transparent; color: {pal.muted};
    padding: 8px 16px; margin: 2px; border-radius: 8px;
}}
QTabBar::tab:selected {{ background: {pal.surface_3}; color: {pal.text}; }}
QCheckBox, QRadioButton {{ color: {pal.text}; spacing: 8px; }}
QGroupBox {{
    color: {pal.text}; border: 1px solid {pal.line}; border-radius: 12px;
    margin-top: 12px; padding-top: 8px;
}}
QGroupBox::title {{ subcontrol-origin: margin; left: 12px; padding: 0 4px; color: {pal.muted}; }}
QSplitter::handle {{ background: {pal.line}; }}
QSplitter::handle:horizontal {{ width: 2px; }}
QSplitter::handle:vertical {{ height: 2px; }}
QPushButton:focus {{ outline: none; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{
    background: {pal.muted}; border-radius: 5px; min-height: 36px;
}}
QScrollBar::handle:vertical:hover {{ background: {pal.text}; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: transparent; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: {pal.muted}; border-radius: 5px; min-width: 36px; }}
QScrollBar::handle:horizontal:hover {{ background: {pal.text}; }}
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
    "Inter", "Geist", "Manrope", "Plus Jakarta Sans", "Onest",
    "SF Pro Display", "Segoe UI Variable Display", "Segoe UI",
    "Ubuntu", "Noto Sans", "Cantarell", "DejaVu Sans",
]

INTER_FONT_PATH = str(Path(__file__).resolve().parent.parent / "fonts" / "Inter-Regular.ttf")


def _load_inter_font() -> bool:
    if Path(INTER_FONT_PATH).exists():
        font_id = QFontDatabase.addApplicationFont(INTER_FONT_PATH)
        if font_id >= 0:
            families = QFontDatabase.applicationFontFamilies(font_id)
            if families:
                return True
    return False


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
        _load_inter_font()
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

    def export_theme(self) -> dict:
        """Только оформление (без поведения) — для шаринга тем."""
        data = self._settings.to_dict()
        return {key: data[key] for key in (
            "mode", "accent", "glass", "glass_tint", "density", "radius",
            "motion", "font_scale", "orbs", "orbs_intensity") if key in data}

    def import_theme(self, data: dict | None) -> bool:
        """Применяет ранее экспортированную тему. True — что-то сменилось."""
        if not isinstance(data, dict):
            return False
        allowed = {"mode", "accent", "glass", "glass_tint", "density",
                   "radius", "motion", "font_scale", "orbs", "orbs_intensity"}
        changes = {k: v for k, v in data.items() if k in allowed}
        if not changes:
            return False
        before = self._settings.to_dict()
        self.update(**changes)
        return self._settings.to_dict() != before
