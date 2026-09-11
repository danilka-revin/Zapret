"""
Стеклянные виджеты Qt6 для Zapret Control.

Собственный набор примитивов вместо стандартных Qt-контролов: карточки с
программным «матовым стеклом», большая кнопка питания, пилюли статуса,
сегментированные переключатели, график трафика, тосты и выезжающая панель
настроек. Всё рисуется QPainter и мгновенно перекрашивается под тему.
"""

from __future__ import annotations

from PySide6.QtCore import (QEasingCurve, QPoint, QPointF, QRect, QRectF,
                            QSize, Qt, QTimer, QVariantAnimation, Signal)
from PySide6.QtGui import (QBrush, QColor, QFont, QFontMetrics, QLinearGradient,
                           QPainter, QPainterPath, QPen, QPixmap, QRadialGradient)
from PySide6.QtWidgets import (QFrame, QGraphicsOpacityEffect, QHBoxLayout, QLabel,
                               QPlainTextEdit, QPushButton, QScrollArea, QSizePolicy,
                               QVBoxLayout, QWidget)

from . import icons


# ---------------------------------------------------------------------------
# Вспомогательные функции
# ---------------------------------------------------------------------------

def animate(owner: QWidget, name: str, start: float, end: float, duration: int,
            apply, curve: QEasingCurve.Type = QEasingCurve.Type.OutCubic):
    """Анимирует значение без объявления Qt-свойств (хранит анимации на виджете)."""
    store: dict = owner.__dict__.setdefault("_zapret_anims", {})
    old = store.get(name)
    if old is not None:
        old.stop()
        store.pop(name, None)
    if duration <= 0 or start == end:
        apply(float(end))
        return None
    anim = QVariantAnimation(owner)
    anim.setStartValue(float(start))
    anim.setEndValue(float(end))
    anim.setDuration(int(duration))
    anim.setEasingCurve(curve)
    anim.valueChanged.connect(lambda value: apply(float(value)))
    anim.finished.connect(lambda: store.pop(name, None))
    store[name] = anim
    anim.start()
    return anim


def font(family: str, size: float, weight: QFont.Weight = QFont.Weight.Normal,
         mono: bool = False, letter_spacing: float = 0.0) -> QFont:
    f = QFont(family, -1)
    f.setPixelSize(max(8, int(round(size))))
    f.setWeight(weight)
    f.setStyleStrategy(QFont.StyleStrategy.PreferAntialias)
    if letter_spacing:
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, letter_spacing)
    return f


def round_path(rect: QRectF, radius: float) -> QPainterPath:
    path = QPainterPath()
    path.addRoundedRect(rect, radius, radius)
    return path


class _Glass:
    """Ссылка на общий фон приложения (для матового стекла в карточках)."""

    backdrop = None
    blur_factor = 7


# ---------------------------------------------------------------------------
# Фон
# ---------------------------------------------------------------------------

class Backdrop(QWidget):
    """Фон окна: градиент, мягкие цветные пятна и «размытая» копия для стекла."""

    def __init__(self, theme, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self._low: QPixmap | None = None
        self._low_size = QSize(0, 0)
        self._orbs = [(0.16, 0.06, 0.62), (0.86, 0.12, 0.70), (0.34, 1.02, 0.74)]
        theme.changed.connect(self._invalidate)

    # -- кэш «размытия» ----------------------------------------------------

    def _invalidate(self):
        self._low = None
        self.update()

    def low_pixmap(self) -> QPixmap:
        """Маленькая копия фона — растянутая обратно, заменяет размытие по Гауссу."""
        size = self.size()
        if size.isEmpty():
            return QPixmap(1, 1)
        if self._low is not None and self._low_size == size:
            return self._low
        factor = _Glass.blur_factor
        small = QSize(max(8, size.width() // factor), max(8, size.height() // factor))
        self._low = self._render(small, blurry=True)
        self._low_size = size
        return self._low

    # -- отрисовка ---------------------------------------------------------

    def _render(self, size: QSize, blurry: bool = False) -> QPixmap:
        pal = self.theme.palette
        pm = QPixmap(size)
        pm.fill(Qt.GlobalColor.transparent)
        p = QPainter(pm)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, not blurry)

        grad = QLinearGradient(0, 0, size.width() * 0.35, size.height())
        grad.setColorAt(0.0, QColor(pal.page_alt))
        grad.setColorAt(1.0, QColor(pal.page))
        p.fillRect(QRect(0, 0, size.width(), size.height()), QBrush(grad))

        if pal.s.orbs and pal.s.orbs_intensity > 0:
            strength = pal.s.orbs_intensity / 100.0
            for (cx, cy, radius), color in zip(self._orbs, pal.orb_colors, strict=False):
                center = QPointF(cx * size.width(), cy * size.height())
                rad = radius * max(size.width(), size.height()) * 0.55
                glow = QRadialGradient(center, rad)
                alpha = (0.30 if not pal.dark else 0.34) * strength
                glow.setColorAt(0.0, _with_alpha(color, alpha))
                glow.setColorAt(0.55, _with_alpha(color, alpha * 0.28))
                glow.setColorAt(1.0, _with_alpha(color, 0.0))
                p.setBrush(QBrush(glow))
                p.setPen(Qt.PenStyle.NoPen)
                p.drawEllipse(center, rad, rad)
        p.end()
        return pm

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.drawPixmap(0, 0, self._render(self.size()))
        p.end()
        if pal.dark:
            return


def _with_alpha(hex_color: str, alpha: float) -> QColor:
    color = QColor(hex_color)
    color.setAlphaF(max(0.0, min(1.0, alpha)))
    return color


# ---------------------------------------------------------------------------
# Карточка
# ---------------------------------------------------------------------------

class Card(QFrame):
    """Панель с матовым стеклом, тонкой рамкой и мягкой тенью."""

    def __init__(self, theme, title: str = "", subtitle: str = "", icon: str = "",
                 parent=None):
        super().__init__(parent)
        self.theme = theme
        self.title = title
        self.subtitle = subtitle
        self.icon_name = icon
        self.radius_extra = 0
        self.accent_edge = False
        self._backdrop = None
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        self.body = QVBoxLayout(self)
        self._apply_margins()
        if title:
            header = QWidget(self)
            hl = QHBoxLayout(header)
            hl.setContentsMargins(0, 0, 0, 0)
            hl.setSpacing(8)
            if icon:
                self.icon_label = QLabel(header)
                self.icon_label.setFixedSize(22, 22)
                hl.addWidget(self.icon_label)
            self.title_label = QLabel(title, header)
            self.title_label.setObjectName("cardTitle")
            hl.addWidget(self.title_label)
            hl.addStretch(1)
            self.header_row = hl
            self.body.addWidget(header)
            if subtitle:
                self.subtitle_label = QLabel(subtitle, header)
                self.subtitle_label.setObjectName("cardSubtitle")
                self.subtitle_label.setWordWrap(True)
                self.body.addWidget(self.subtitle_label)
        self._restyle()
        theme.changed.connect(self._restyle)

    def _apply_margins(self):
        pad = self.theme.palette.pad
        self.body.setContentsMargins(pad, int(pad * 0.85), pad, int(pad * 0.85))
        self.body.setSpacing(max(6, int(self.theme.palette.gap * 0.7)))

    def _restyle(self):
        pal = self.theme.palette
        self._apply_margins()
        if hasattr(self, "title_label"):
            self.title_label.setFont(font(self.theme.font_family, pal.font_md,
                                          QFont.Weight.DemiBold))
            self.title_label.setStyleSheet(f"color:{pal.text};")
        if hasattr(self, "subtitle_label"):
            self.subtitle_label.setFont(font(self.theme.font_family, pal.font_xs))
            self.subtitle_label.setStyleSheet(f"color:{pal.muted};")
        if hasattr(self, "icon_label"):
            self.icon_label.setPixmap(icons.icon_pixmap(self.icon_name, 20, pal.accent, 1.7))
        self.update()

    # -- отрисовка ---------------------------------------------------------

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        radius = pal.r_card + self.radius_extra
        path = round_path(rect, radius)

        # Мягкая тень
        shadow = QColor(pal.shadow)
        for i in range(6, 0, -1):
            shadow.setAlphaF(0.035 if pal.dark else 0.05)
            p.setPen(QPen(shadow, i * 2.0))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPath(round_path(rect.adjusted(i * 0.6, i * 1.1, -i * 0.6, -i * 0.2), radius))

        # Стеклянная подложка
        p.setPen(Qt.PenStyle.NoPen)
        if (pal.glass_alpha > 0 and _Glass.backdrop is not None
                and getattr(_Glass.backdrop, "low_pixmap", None) is not None):
            self._backdrop = _Glass.backdrop
        if self._backdrop is not None and pal.glass_alpha > 0:
            low = self._backdrop.low_pixmap()
            if not low.isNull():
                factor = low.width() / max(1, self._backdrop.width())
                global_pos = self.mapToGlobal(QPoint(0, 0))
                origin = self._backdrop.mapFromGlobal(global_pos)
                source = QRectF(origin.x() * factor, origin.y() * factor,
                                self.width() * factor, self.height() * factor)
                p.setClipPath(path)
                p.drawPixmap(rect, low, source)
                p.setClipping(False)

        # Тонировка и заливка
        pal_color = QColor(pal.surface)
        if pal.dark:
            pal_color.setAlphaF(min(1.0, 0.55 + (1.0 - pal.glass_alpha) * 0.4))
        else:
            pal_color.setAlphaF(min(1.0, 0.72 + (1.0 - pal.glass_alpha) * 0.28))
        p.setBrush(pal_color)
        p.drawPath(path)

        if pal.glass_alpha > 0:
            tint = QLinearGradient(rect.topLeft(), QPointF(rect.left(), rect.bottom()))
            base = 0.09 if pal.dark else 0.05
            tint.setColorAt(0.0, _with_alpha(pal.tint, base * pal.glass_alpha))
            tint.setColorAt(1.0, _with_alpha(pal.tint, 0.0))
            p.setBrush(QBrush(tint))
            p.drawPath(path)

        # Рамка и верхний блик
        border = QColor(pal.card_border)
        if self.accent_edge:
            border = _with_alpha(pal.accent, 0.55)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(border, 1.2))
        p.drawPath(path)
        p.setPen(QPen(_with_alpha("#ffffff", 0.14 if pal.dark else 0.65), 1.0))
        p.drawLine(QPointF(rect.left() + radius * 0.8, rect.top() + 1),
                   QPointF(rect.right() - radius * 0.8, rect.top() + 1))
        p.end()


# ---------------------------------------------------------------------------
# Большая кнопка питания
# ---------------------------------------------------------------------------

class PowerSwitch(QWidget):
    """
    Главный элемент интерфейса: большая круглая кнопка включения обхода.

    Состояния: off → idle-контур; on → акцентное свечение и дуга прогресса;
    busy → вращающаяся дуга и пульс. Один клик — включить или выключить.
    """

    clicked = Signal()

    def __init__(self, theme, size: int = 236, parent=None):
        super().__init__(parent)
        self.theme = theme
        self._size = size
        self.state = "off"
        self.progress = 0.0
        self.spin = 0.0
        self.hover = 0.0
        self.press = 0.0
        self.pulse = 0.0
        self.caption = "ВКЛЮЧИТЬ"
        self.hint = "Нажмите, чтобы включить обход"
        self._pulse_dir = 1.0
        self.setFixedSize(size, size + 54)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setAttribute(Qt.WidgetAttribute.WA_Hover, True)
        self.setMouseTracking(True)
        self._pulse_timer = QTimer(self)
        self._pulse_timer.timeout.connect(self._on_pulse)
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._on_spin)
        theme.changed.connect(self.update)

    # -- внешний вид -------------------------------------------------------

    def _disc_rect(self) -> QRectF:
        pad = 14
        return QRectF(pad, pad, self._size - pad * 2, self._size - pad * 2)

    def set_state(self, state: str, caption: str = "", hint: str = ""):
        if state not in ("off", "on", "busy", "error"):
            state = "off"
        changed = state != self.state
        self.state = state
        if caption:
            self.caption = caption
        if hint:
            self.hint = hint
        if state == "busy":
            if not self._spin_timer.isActive():
                self._spin_timer.start(16)
            self._pulse_timer.stop()
        else:
            self._spin_timer.stop()
            self.spin = 0.0
            if state == "on" and self.theme.palette.animated and changed:
                self._pulse_timer.start(30)
            else:
                self._pulse_timer.stop()
                self.pulse = 0.0
        self.update()

    def set_progress(self, value: float):
        self.progress = max(0.0, min(1.0, float(value)))
        self.update()

    # -- анимации ----------------------------------------------------------

    def _on_pulse(self):
        step = 0.02 if self.theme.palette.anim_ms >= 200 else 0.012
        self.pulse += step * self._pulse_dir
        if self.pulse >= 1.0:
            self.pulse, self._pulse_dir = 1.0, -1.0
        elif self.pulse <= 0.0:
            self.pulse, self._pulse_dir = 0.0, 1.0
        self.update()

    def _on_spin(self):
        self.spin = (self.spin + 4.0) % 360.0
        self.update()

    # -- события -----------------------------------------------------------

    def enterEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 1.0, self.theme.palette.anim_ms, self._set_hover)

    def leaveEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 0.0, self.theme.palette.anim_ms, self._set_hover)

    def _set_hover(self, value: float):
        self.hover = value
        self.update()

    def _set_press(self, value: float):
        self.press = value
        self.update()

    def mousePressEvent(self, event):  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton:
            animate(self, "press", self.press, 1.0, 90, self._set_press,
                    QEasingCurve.Type.OutQuad)
            event.accept()

    def mouseReleaseEvent(self, event):  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton:
            animate(self, "press", self.press, 0.0, 160, self._set_press,
                    QEasingCurve.Type.OutQuad)
            if self.rect().contains(event.position().toPoint()):
                self.clicked.emit()
            event.accept()

    def sizeHint(self) -> QSize:  # noqa: N802
        return QSize(self._size, self._size + 54)

    # -- отрисовка ---------------------------------------------------------

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        scale = 1.0 + 0.02 * self.hover - 0.03 * self.press
        center = QPointF(self._size / 2.0, self._size / 2.0)
        p.translate(center)
        p.scale(scale, scale)

        on = self.state in ("on", "busy")
        accent = QColor(pal.accent)
        if self.state == "error":
            accent = QColor(pal.bad)
        disc = QRectF(-(self._size / 2 - 16), -(self._size / 2 - 16),
                      self._size - 32, self._size - 32)

        # 1. Свечение вокруг кнопки
        glow_strength = (0.55 if on else 0.16) + 0.10 * self.pulse + 0.18 * self.hover
        glow_radius = disc.width() * (0.78 + 0.05 * self.pulse)
        glow = QRadialGradient(QPointF(0, 0), glow_radius)
        glow.setColorAt(0.62, _with_alpha(accent.name(), 0.30 * glow_strength))
        glow.setColorAt(0.80, _with_alpha(accent.name(), 0.16 * glow_strength))
        glow.setColorAt(1.0, _with_alpha(accent.name(), 0.0))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(glow))
        p.drawEllipse(QPointF(0, 0), glow_radius, glow_radius)

        # 2. Кольцо-трек и дуга прогресса
        ring_w = max(8.0, self._size * 0.045)
        ring_rect = disc.adjusted(-ring_w * 0.7, -ring_w * 0.7, ring_w * 0.7, ring_w * 0.7)
        track_color = _with_alpha(pal.text, 0.10 if pal.dark else 0.08)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(track_color, ring_w, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawArc(ring_rect, 0, 360 * 16)

        if self.state == "busy":
            p.setPen(QPen(accent, ring_w, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawArc(ring_rect, int(-self.spin * 16), int(-110 * 16))
            p.setPen(QPen(_with_alpha(accent.name(), 0.35), ring_w,
                          Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawArc(ring_rect, int((-self.spin + 180) * 16), int(-80 * 16))
        elif self.progress > 0.001:
            p.setPen(QPen(accent, ring_w, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawArc(ring_rect, 90 * 16, int(-self.progress * 360 * 16))

        # 3. Внутренний диск
        if on:
            grad = QLinearGradient(0, -disc.height() / 2, 0, disc.height() / 2)
            grad.setColorAt(0.0, QColor(pal.accent).lighter(112))
            grad.setColorAt(1.0, QColor(pal.accent).darker(112))
            disc_brush = QBrush(grad)
        else:
            grad = QLinearGradient(0, -disc.height() / 2, 0, disc.height() / 2)
            if pal.dark:
                grad.setColorAt(0.0, QColor(pal.surface_3))
                grad.setColorAt(1.0, QColor(pal.surface))
            else:
                grad.setColorAt(0.0, QColor("#ffffff"))
                grad.setColorAt(1.0, QColor(pal.surface_2))
            disc_brush = QBrush(grad)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(disc_brush)
        p.drawEllipse(disc)

        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(_with_alpha("#ffffff", 0.28 if on else (0.16 if pal.dark else 0.9)), 1.4))
        p.drawEllipse(disc.adjusted(0.6, 0.6, -0.6, -0.6))

        # 4. Иконка и подписи
        icon_color = pal.on_accent if on else pal.text
        icon_size = int(self._size * 0.30)
        pm = icons.icon_pixmap("power", icon_size, icon_color, 2.0)
        p.drawPixmap(QPointF(-pm.width() / 2, -pm.height() / 2 - self._size * 0.085), pm)

        caption_color = QColor(pal.on_accent if on else pal.text)
        p.setPen(caption_color)
        p.setFont(font(self.theme.font_family, max(11.0, self._size * 0.072),
                       QFont.Weight.Bold, letter_spacing=1.2))
        cap_rect = QRectF(-disc.width() / 2, self._size * 0.055, disc.width(), self._size * 0.13)
        p.drawText(cap_rect, Qt.AlignmentFlag.AlignCenter, self.caption)

        # На включённой кнопке подпись рисуем цветом, контрастным к акценту
        baseline = pal.on_accent if on else pal.muted
        hint_color = QColor(baseline)
        hint_color.setAlphaF(0.72 if on else 0.95)
        p.setPen(hint_color)
        p.setFont(font(self.theme.font_family, max(8.5, pal.font_xs)))
        p.drawText(QRectF(-self._size, self._size * 0.185, self._size * 2, 40),
                   Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, self.hint)
        p.end()


# ---------------------------------------------------------------------------
# Пилюли, точки статуса, кнопки
# ---------------------------------------------------------------------------

class StatusPill(QFrame):
    """Компактная «пилюля» статуса с индикатором и текстом."""

    def __init__(self, theme, text: str = "", state: str = "idle", parent=None):
        super().__init__(parent)
        self.theme = theme
        self.text = text
        self.state = state
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.setFixedHeight(34)
        theme.changed.connect(self.update)

    def set_status(self, text: str, state: str = "idle"):
        self.text = text
        self.state = state
        font_metrics = QFontMetrics(font(self.theme.font_family, self.theme.palette.font_sm,
                                         QFont.Weight.DemiBold))
        self.setMinimumWidth(font_metrics.horizontalAdvance(text) + 46)
        self.setFixedHeight(34)
        self.update()

    def _color(self) -> QColor:
        pal = self.theme.palette
        return {"ok": QColor(pal.good), "warn": QColor(pal.warn),
                "bad": QColor(pal.bad), "accent": QColor(pal.accent),
                "idle": QColor(pal.muted)}.get(self.state, QColor(pal.muted))

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        color = self._color()
        path = round_path(rect, rect.height() / 2)
        p.setBrush(_with_alpha(color.name(), 0.14 if pal.dark else 0.16))
        p.setPen(QPen(_with_alpha(color.name(), 0.45), 1.1))
        p.drawPath(path)
        cy = rect.center().y()
        p.setBrush(color)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(rect.left() + 15, cy), 4.5, 4.5)
        if self.state in ("ok", "accent"):
            ring = QRadialGradient(QPointF(rect.left() + 15, cy), 11)
            ring.setColorAt(0.0, _with_alpha(color.name(), 0.45))
            ring.setColorAt(1.0, _with_alpha(color.name(), 0.0))
            p.setBrush(QBrush(ring))
            p.drawEllipse(QPointF(rect.left() + 15, cy), 11, 11)
        p.setPen(QColor(pal.text))
        p.setFont(font(self.theme.font_family, pal.font_sm, QFont.Weight.DemiBold))
        p.drawText(QRectF(rect.left() + 26, rect.top(), rect.width() - 34, rect.height()),
                   Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self.text)
        p.end()


class GlassButton(QPushButton):
    """Кнопка в стиле стекла: primary / secondary / ghost / danger / accent-soft."""

    def __init__(self, theme, text: str = "", icon: str = "", kind: str = "secondary",
                 parent=None, compact: bool = False, icon_only: bool = False):
        super().__init__(text, parent)
        self.theme = theme
        self.icon_name = icon
        self.kind = kind
        self.hover = 0.0
        self.press = 0.0
        self.compact = compact
        self.icon_only = icon_only
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setAttribute(Qt.WidgetAttribute.WA_Hover, True)
        h = 34 if compact else 40
        self.setFixedHeight(h)
        if icon_only:
            self.setFixedWidth(h)
        else:
            self.setMinimumWidth(96)
        self._update_font()
        theme.changed.connect(self._update_font)

    def _update_font(self):
        pal = self.theme.palette
        self.setFont(font(self.theme.font_family,
                          pal.font_sm if self.compact else pal.font_md,
                          QFont.Weight.DemiBold))
        fm = QFontMetrics(self.font())
        if not self.icon_only:
            pad = 34 if self.icon_name else 26
            self.setMinimumWidth(min(320, fm.horizontalAdvance(self.text()) + pad))
        self.update()

    def set_kind(self, kind: str):
        self.kind = kind
        self.update()

    def set_icon(self, name: str):
        self.icon_name = name
        self.update()

    def enterEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 1.0, self.theme.palette.anim_ms, self._set_hover)

    def leaveEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 0.0, self.theme.palette.anim_ms, self._set_hover)

    def _set_hover(self, value: float):
        self.hover = value
        self.update()

    def mousePressEvent(self, event):  # noqa: N802
        animate(self, "press", self.press, 1.0, 80, self._set_press, QEasingCurve.Type.OutQuad)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):  # noqa: N802
        animate(self, "press", self.press, 0.0, 160, self._set_press, QEasingCurve.Type.OutQuad)
        super().mouseReleaseEvent(event)

    def _set_press(self, value: float):
        self.press = value
        self.update()

    def _colors(self) -> tuple[QColor, QColor, QColor]:
        """(фон, рамка, текст/иконка)"""
        pal = self.theme.palette
        if self.kind == "primary":
            bg = QColor(pal.accent)
            return bg, QColor(pal.accent), QColor(pal.on_accent)
        if self.kind == "danger":
            return (_with_alpha(pal.bad, 0.16), _with_alpha(pal.bad, 0.5), QColor(pal.bad))
        if self.kind == "ghost":
            return (QColor(0, 0, 0, 0), _with_alpha(pal.text, 0.10), QColor(pal.text))
        if self.kind == "accent-soft":
            return (_with_alpha(pal.accent, 0.18), _with_alpha(pal.accent, 0.45),
                    QColor(pal.accent_text))
        base = QColor(pal.surface_2 if pal.dark else "#ffffff")
        return base, _with_alpha(pal.text, 0.12 if pal.dark else 0.14), QColor(pal.text)

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        bg, border, fg = self._colors()
        if not self.isEnabled():
            bg = _with_alpha(pal.text, 0.05)
            border = _with_alpha(pal.text, 0.08)
            fg = _with_alpha(pal.text, 0.35)
        else:
            if self.hover:
                bg = QColor(bg)
                if self.kind == "primary":
                    bg = bg.lighter(int(100 + 10 * self.hover))
                else:
                    bg.setAlphaF(min(1.0, bg.alphaF() + 0.18 * self.hover))
            if self.press:
                bg = QColor(bg).darker(104)

        radius = pal.r_md if pal.s.radius == "soft" else pal.r_sm
        path = round_path(rect, radius)
        if self.kind == "primary":
            grad = QLinearGradient(rect.topLeft(), rect.bottomLeft())
            grad.setColorAt(0.0, QColor(bg).lighter(106))
            grad.setColorAt(1.0, QColor(bg).darker(104))
            p.setBrush(QBrush(grad))
        else:
            p.setBrush(bg)
        p.setPen(QPen(border, 1.1))
        p.drawPath(path)

        if self.kind == "primary":
            p.setPen(QPen(_with_alpha("#ffffff", 0.35), 1.0))
            p.drawLine(QPointF(rect.left() + radius, rect.top() + 1.4),
                       QPointF(rect.right() - radius, rect.top() + 1.4))

        gap = 8
        icon_size = 18 if not self.compact else 16
        text = self.text()
        fm = QFontMetrics(self.font())
        text_w = fm.horizontalAdvance(text) if text else 0
        icon_w = icon_size + gap if self.icon_name else 0
        content_w = text_w + icon_w
        x = rect.center().x() - content_w / 2
        if self.icon_name:
            pm = icons.icon_pixmap(self.icon_name, icon_size, fg.name(), 1.8)
            p.setOpacity(1.0 if self.isEnabled() else 0.5)
            p.drawPixmap(QPointF(x, rect.center().y() - pm.height() / 2), pm)
            p.setOpacity(1.0)
            x += icon_size + gap
        if text:
            p.setPen(fg)
            p.setFont(self.font())
            p.drawText(QRectF(x, rect.top(), text_w + 2, rect.height()),
                       Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, text)
        p.end()


class Switch(QWidget):
    """Анимированный переключатель (аналог <input type=checkbox> в zmk)."""

    toggled = Signal(bool)

    def __init__(self, theme, checked: bool = False, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.checked = bool(checked)
        self.knob = 1.0 if checked else 0.0
        self.hover = 0.0
        self.w, self.h = 46, 26
        self.setFixedSize(self.w + 6, self.h + 6)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setAttribute(Qt.WidgetAttribute.WA_Hover, True)
        theme.changed.connect(self.update)

    def sizeHint(self) -> QSize:  # noqa: N802
        return QSize(self.w + 6, self.h + 6)

    def isChecked(self) -> bool:  # noqa: N802
        return self.checked

    def setChecked(self, value: bool, animate_value: bool = True):  # noqa: N802
        value = bool(value)
        if value == self.checked:
            return
        self.checked = value
        target = 1.0 if value else 0.0
        if animate_value and self.theme.palette.animated:
            animate(self, "knob", self.knob, target, self.theme.palette.anim_ms, self._set_knob)
        else:
            self._set_knob(target)

    def _set_knob(self, value: float):
        self.knob = value
        self.update()

    def _set_hover(self, value: float):
        self.hover = value
        self.update()

    def mouseReleaseEvent(self, event):  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton and self.rect().contains(
                event.position().toPoint()):
            self.setChecked(not self.checked)
            self.toggled.emit(self.checked)

    def enterEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 1.0, self.theme.palette.anim_ms, self._set_hover)

    def leaveEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 0.0, self.theme.palette.anim_ms, self._set_hover)

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        track = QRectF(3, 3, self.w, self.h)
        on_color = QColor(pal.accent)
        off_color = QColor(pal.surface_3 if pal.dark else "#d8e0dc")
        color = _mix_qcolor(off_color, on_color, self.knob)
        if self.hover and self.isEnabled():
            color = color.lighter(106 if pal.dark else 102)
        p.setPen(QPen(_with_alpha(pal.text, 0.08), 1.0))
        p.setBrush(color)
        p.drawRoundedRect(track, track.height() / 2, track.height() / 2)
        if self.knob > 0.05:
            glow = QRadialGradient(track.center(), self.w * 0.8)
            glow.setColorAt(0.0, _with_alpha(pal.accent, 0.25 * self.knob))
            glow.setColorAt(1.0, _with_alpha(pal.accent, 0.0))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(glow))
            p.drawEllipse(track.center(), self.w * 0.8, self.w * 0.8)

        knob_d = self.h - 6
        x = track.left() + 3 + (track.width() - knob_d - 6) * self.knob
        knob_rect = QRectF(x, track.top() + 3, knob_d, knob_d)
        knob_color = QColor(pal.on_accent if self.knob > 0.5 else (pal.surface if pal.dark else "#ffffff"))
        if self.knob > 0.5:
            knob_color = QColor("#ffffff") if pal.dark else QColor(pal.on_accent)
        p.setBrush(knob_color)
        p.setPen(QPen(_with_alpha("#000000", 0.10), 1.0))
        p.drawEllipse(knob_rect)
        p.end()


def _mix_qcolor(a: QColor, b: QColor, t: float) -> QColor:
    t = max(0.0, min(1.0, t))
    return QColor(
        round(a.red() + (b.red() - a.red()) * t),
        round(a.green() + (b.green() - a.green()) * t),
        round(a.blue() + (b.blue() - a.blue()) * t),
    )


class SegmentedControl(QWidget):
    """Группа кнопок-сегментов с анимированной подсветкой выбранного."""

    changed = Signal(str)

    def __init__(self, theme, options: list[tuple[str, str]], value: str = "",
                 parent=None, stretch: bool = True):
        super().__init__(parent)
        self.theme = theme
        self.options = options
        self.value = value or (options[0][0] if options else "")
        self.index = max(0, [key for key, _ in options].index(self.value)
                         if self.value in [k for k, _ in options] else 0)
        self.slide = float(self.index)
        self.hover_index = -1
        self.setFixedHeight(38)
        self.setMouseTracking(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setAttribute(Qt.WidgetAttribute.WA_Hover, True)
        self._stretch = stretch
        if not stretch:
            self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        theme.changed.connect(self.update)

    def sizeHint(self) -> QSize:  # noqa: N802
        return QSize(self.width() or 240, 38)

    def set_value(self, value: str, emit: bool = False):
        keys = [k for k, _ in self.options]
        if value not in keys:
            return
        index = keys.index(value)
        if index == self.index and value == self.value:
            return
        self.value = value
        old = self.index
        self.index = index
        if self.theme.palette.animated:
            animate(self, "slide", old, index, self.theme.palette.anim_ms, self._set_slide)
        else:
            self._set_slide(index)
        self.update()
        if emit:
            self.changed.emit(value)

    def _set_slide(self, value: float):
        self.slide = value
        self.update()

    def _segment_rect(self, index: int) -> QRectF:
        n = max(1, len(self.options))
        gap = 4.0
        w = (self.width() - gap * (n + 1)) / n
        return QRectF(gap + index * (w + gap), gap, w, self.height() - gap * 2)

    def _index_at(self, pos) -> int:
        for i in range(len(self.options)):
            if self._segment_rect(i).contains(QPointF(pos)):
                return i
        return -1

    def mouseMoveEvent(self, event):  # noqa: N802
        index = self._index_at(event.position())
        if index != self.hover_index:
            self.hover_index = index
            self.update()

    def leaveEvent(self, event):  # noqa: N802
        self.hover_index = -1
        self.update()

    def mouseReleaseEvent(self, event):  # noqa: N802
        if event.button() != Qt.MouseButton.LeftButton:
            return
        index = self._index_at(event.position())
        if index >= 0:
            self.set_value(self.options[index][0], emit=True)

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        radius = (rect.height() / 2) if pal.s.radius == "soft" else pal.r_sm
        p.setBrush(_with_alpha(pal.text, 0.06))
        p.setPen(QPen(_with_alpha(pal.text, 0.08), 1.0))
        p.drawRoundedRect(rect, radius, radius)

        # подсветка выбранного
        if self.options:
            w = (self.width() - 4.0 * (len(self.options) + 1)) / len(self.options)
            gap = 4.0
            x = gap + self.slide * (w + gap)
            sel = QRectF(x, gap, w, self.height() - gap * 2)
            sel_radius = (sel.height() / 2) if pal.s.radius == "soft" else max(2, pal.r_sm - 2)
            grad = QLinearGradient(sel.topLeft(), sel.bottomLeft())
            if pal.dark:
                grad.setColorAt(0.0, QColor(pal.surface_3))
                grad.setColorAt(1.0, QColor(pal.surface_2))
            else:
                grad.setColorAt(0.0, QColor("#ffffff"))
                grad.setColorAt(1.0, QColor("#f2f6f4"))
            p.setBrush(QBrush(grad))
            p.setPen(QPen(_with_alpha(pal.accent, 0.35), 1.0))
            p.drawRoundedRect(sel, sel_radius, sel_radius)

        for i, (_key, label) in enumerate(self.options):
            seg = self._segment_rect(i)
            selected = abs(i - self.slide) < 0.5
            color = QColor(pal.text if selected else pal.muted)
            if i == self.hover_index and not selected:
                color = QColor(pal.text)
            p.setPen(color)
            p.setFont(font(self.theme.font_family, pal.font_sm,
                           QFont.Weight.DemiBold if selected else QFont.Weight.Normal))
            p.drawText(seg, Qt.AlignmentFlag.AlignCenter, label)
        p.end()


class IconButton(QPushButton):
    """Квадратная кнопка-иконка (используется в шапке и панелях)."""

    def __init__(self, theme, icon: str, tooltip: str = "", size: int = 38,
                 kind: str = "ghost", parent=None):
        super().__init__(parent)
        self.theme = theme
        self.icon_name = icon
        self.kind = kind
        self.hover = 0.0
        self.press = 0.0
        self._size = size
        self.setFixedSize(size, size)
        self.setToolTip(tooltip)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setAttribute(Qt.WidgetAttribute.WA_Hover, True)
        theme.changed.connect(self.update)

    def set_icon(self, name: str):
        self.icon_name = name
        self.update()

    def enterEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 1.0, self.theme.palette.anim_ms, self._set_hover)

    def leaveEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 0.0, self.theme.palette.anim_ms, self._set_hover)

    def _set_hover(self, value: float):
        self.hover = value
        self.update()

    def _set_press(self, value: float):
        self.press = value
        self.update()

    def mousePressEvent(self, event):  # noqa: N802
        animate(self, "press", self.press, 1.0, 80, self._set_press, QEasingCurve.Type.OutQuad)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):  # noqa: N802
        animate(self, "press", self.press, 0.0, 160, self._set_press, QEasingCurve.Type.OutQuad)
        super().mouseReleaseEvent(event)

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(1.5, 1.5, -1.5, -1.5)
        radius = rect.width() / 2 if pal.s.radius == "soft" else pal.r_sm
        if self.hover or self.press or self.kind == "solid":
            if self.kind == "solid":
                bg = _with_alpha(pal.accent, 0.18)
                border = _with_alpha(pal.accent, 0.45)
                fg = QColor(pal.accent_text)
            else:
                bg = _with_alpha(pal.text, 0.05 + 0.06 * self.hover)
                border = _with_alpha(pal.text, 0.10)
                fg = QColor(pal.text)
            if self.press:
                bg = _with_alpha(pal.text, 0.10)
            p.setBrush(bg)
            p.setPen(QPen(border, 1.0))
            p.drawRoundedRect(rect, radius, radius)
        else:
            fg = QColor(pal.muted)
        pm = icons.icon_pixmap(self.icon_name, int(self._size * 0.46), fg.name(), 1.8)
        p.drawPixmap(QPointF(rect.center().x() - pm.width() / 2,
                             rect.center().y() - pm.height() / 2), pm)
        p.end()


# ---------------------------------------------------------------------------
# Мелкие информационные виджеты
# ---------------------------------------------------------------------------

class StatTile(QWidget):
    """Плитка с крупным значением и подписью."""

    def __init__(self, theme, caption: str, value: str = "—", icon: str = "",
                 parent=None):
        super().__init__(parent)
        self.theme = theme
        self.caption = caption
        self.value = value
        self.icon_name = icon
        self.accent = False
        self.setMinimumHeight(58)
        theme.changed.connect(self.update)

    def set_value(self, value: str, accent: bool = False):
        if value == self.value and accent == self.accent:
            return
        self.value = value
        self.accent = accent
        self.update()

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        p.setBrush(_with_alpha(pal.text, 0.045))
        p.setPen(QPen(_with_alpha(pal.text, 0.07), 1.0))
        p.drawRoundedRect(rect, pal.r_md, pal.r_md)
        x = rect.left() + 12
        if self.icon_name:
            pm = icons.icon_pixmap(self.icon_name, 16,
                                   pal.accent_text if self.accent else pal.muted, 1.8)
            p.drawPixmap(QPointF(x, rect.top() + 10), pm)
            x += 22
        p.setPen(QColor(pal.muted))
        p.setFont(font(self.theme.font_family, pal.font_xs))
        p.drawText(QRectF(x, rect.top() + 6, rect.width() - 18, 16),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.caption)
        p.setPen(QColor(pal.accent_text if self.accent else pal.text))
        p.setFont(font(self.theme.font_family, pal.font_lg, QFont.Weight.Bold))
        p.drawText(QRectF(rect.left() + 12, rect.top() + 24, rect.width() - 20, 28),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.value)
        p.end()


class Sparkline(QWidget):
    """Живой график скорости: линия с градиентной заливкой и сеткой."""

    def __init__(self, theme, parent=None, points: int = 60):
        super().__init__(parent)
        self.theme = theme
        self.values: list[float] = [0.0] * points
        self.limit = points
        self.setMinimumHeight(52)
        self.peak = 1.0
        self.unit = "Мбит/с"
        theme.changed.connect(self.update)

    def push(self, value: float):
        self.values.append(max(0.0, float(value)))
        if len(self.values) > self.limit:
            self.values = self.values[-self.limit:]
        self.peak = max(1.0, max(self.values))
        self.update()

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        p.setBrush(_with_alpha(pal.text, 0.04))
        p.setPen(QPen(_with_alpha(pal.text, 0.06), 1.0))
        p.drawRoundedRect(rect, pal.r_md, pal.r_md)

        inner = rect.adjusted(8, 8, -8, -18)
        p.setPen(QPen(_with_alpha(pal.text, 0.06), 1.0))
        for i in range(1, 3):
            y = inner.top() + inner.height() * i / 3
            p.drawLine(QPointF(inner.left(), y), QPointF(inner.right(), y))

        if len(self.values) < 2:
            p.end()
            return
        step = inner.width() / (len(self.values) - 1)
        path = QPainterPath()
        fill = QPainterPath()
        for i, value in enumerate(self.values):
            x = inner.left() + step * i
            y = inner.bottom() - inner.height() * min(1.0, value / self.peak)
            if i == 0:
                path.moveTo(x, y)
                fill.moveTo(x, inner.bottom())
                fill.lineTo(x, y)
            else:
                path.lineTo(x, y)
                fill.lineTo(x, y)
        fill.lineTo(inner.right(), inner.bottom())
        fill.closeSubpath()

        grad = QLinearGradient(inner.topLeft(), inner.bottomLeft())
        grad.setColorAt(0.0, _with_alpha(pal.accent_text, 0.45))
        grad.setColorAt(1.0, _with_alpha(pal.accent_text, 0.0))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(grad))
        p.drawPath(fill)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor(pal.accent_text), 1.8, Qt.PenStyle.SolidLine,
                      Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
        p.drawPath(path)

        last = self.values[-1]
        p.setPen(QColor(pal.muted))
        p.setFont(font(self.theme.font_family, pal.font_xs))
        p.drawText(QRectF(rect.left() + 10, rect.bottom() - 17, rect.width() - 20, 16),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                   f"{last:.2f} {self.unit}   ·   пик {self.peak:.2f}")
        p.end()


class ServiceRow(QWidget):
    """Строка сервиса: иконка, название, задержка, статус-индикатор."""

    clicked = Signal(str)

    def __init__(self, theme, key: str, title: str, icon: str, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.key = key
        self.title = title
        self.icon_name = icon
        self.state = "idle"       # idle | ok | warn | bad | checking
        self.detail = "не проверялось"
        self.hover = 0.0
        self.spin = 0.0
        self.setFixedHeight(52)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setAttribute(Qt.WidgetAttribute.WA_Hover, True)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        theme.changed.connect(self.update)

    def _tick(self):
        self.spin = (self.spin + 12) % 360
        self.update()

    def set_status(self, state: str, detail: str):
        self.state = state
        self.detail = detail
        if state == "checking":
            self._timer.start(40)
        else:
            self._timer.stop()
        self.update()

    def enterEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 1.0, self.theme.palette.anim_ms, self._set_hover)

    def leaveEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 0.0, self.theme.palette.anim_ms, self._set_hover)

    def _set_hover(self, value: float):
        self.hover = value
        self.update()

    def mouseReleaseEvent(self, event):  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self.key)

    def _state_color(self) -> QColor:
        pal = self.theme.palette
        return {"ok": QColor(pal.good), "warn": QColor(pal.warn), "bad": QColor(pal.bad),
                "checking": QColor(pal.accent), "idle": QColor(pal.muted)}.get(
                    self.state, QColor(pal.muted))

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        p.setBrush(_with_alpha(pal.text, 0.03 + 0.04 * self.hover))
        p.setPen(QPen(_with_alpha(pal.text, 0.06), 1.0))
        p.drawRoundedRect(rect, pal.r_md, pal.r_md)

        color = self._state_color()
        icon_bg = QRectF(rect.left() + 10, rect.center().y() - 15, 30, 30)
        p.setBrush(_with_alpha(color.name(), 0.16))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(icon_bg, 9, 9)
        icon_color = color.name() if pal.dark else QColor(color).darker(118).name()
        pm = icons.icon_pixmap(self.icon_name, 17, icon_color, 1.8)
        p.drawPixmap(QPointF(icon_bg.center().x() - pm.width() / 2,
                             icon_bg.center().y() - pm.height() / 2), pm)

        p.setPen(QColor(pal.text))
        p.setFont(font(self.theme.font_family, pal.font_md, QFont.Weight.DemiBold))
        p.drawText(QRectF(icon_bg.right() + 10, rect.top() + 8, rect.width() - 150, 20),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.title)
        p.setPen(QColor(pal.muted))
        p.setFont(font(self.theme.font_family, pal.font_xs))
        p.drawText(QRectF(icon_bg.right() + 10, rect.top() + 26, rect.width() - 150, 18),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.detail)

        # индикатор справа
        cx = rect.right() - 20
        cy = rect.center().y()
        if self.state == "checking":
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(color, 2.2, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawArc(QRectF(cx - 8, cy - 8, 16, 16), int(-self.spin * 16), int(-100 * 16))
        else:
            p.setBrush(color)
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(QPointF(cx, cy), 4.5, 4.5)
            if self.state == "ok":
                ring = QRadialGradient(QPointF(cx, cy), 11)
                ring.setColorAt(0.0, _with_alpha(color.name(), 0.4))
                ring.setColorAt(1.0, _with_alpha(color.name(), 0.0))
                p.setBrush(QBrush(ring))
                p.drawEllipse(QPointF(cx, cy), 11, 11)
        p.end()


# ---------------------------------------------------------------------------
# Тосты
# ---------------------------------------------------------------------------

class Toast(QFrame):
    """Всплывающее уведомление внизу окна."""

    def __init__(self, parent: QWidget, theme, text: str, kind: str = "info",
                 timeout: int = 4200):
        super().__init__(parent)
        self.theme = theme
        self.text = text
        self.kind = kind
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.opacity = 0.0
        icon = {"info": "info", "ok": "check-circle", "warn": "alert",
                "error": "close-circle"}.get(kind, "info")
        self.icon_name = icon
        font_metrics = QFontMetrics(font(theme.font_family, theme.palette.font_md))
        width = min(parent.width() - 60, max(280, font_metrics.horizontalAdvance(text) + 90))
        self.setFixedSize(width, 52)
        self._effect = QGraphicsOpacityEffect(self)
        self._effect.setOpacity(0.0)
        self.setGraphicsEffect(self._effect)
        QTimer.singleShot(timeout, self._fade_out)
        animate(self, "opacity", 0.0, 1.0, theme.palette.anim_ms, self._set_opacity)

    def _set_opacity(self, value: float):
        self.opacity = value
        self._effect.setOpacity(value)

    def _fade_out(self):
        animate(self, "opacity", self.opacity, 0.0, self.theme.palette.anim_ms,
                self._set_opacity)
        QTimer.singleShot(max(1, self.theme.palette.anim_ms + 30), self.deleteLater)

    def _color(self) -> QColor:
        pal = self.theme.palette
        return {"info": QColor(pal.accent_text), "ok": QColor(pal.good),
                "warn": QColor(pal.warn),
                "error": QColor(pal.bad)}.get(self.kind, QColor(pal.accent_text))

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        color = self._color()
        p.setBrush(_with_alpha(pal.surface_3 if pal.dark else "#ffffff", 0.97))
        p.setPen(QPen(_with_alpha(color.name(), 0.5), 1.2))
        p.drawRoundedRect(rect, pal.r_lg, pal.r_lg)
        p.setBrush(color)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(QRectF(rect.left() + 8, rect.top() + 10, 3, rect.height() - 20), 1.5, 1.5)
        pm = icons.icon_pixmap(self.icon_name, 20, color.name(), 1.8)
        p.drawPixmap(QPointF(rect.left() + 20, rect.center().y() - pm.height() / 2), pm)
        p.setPen(QColor(pal.text))
        p.setFont(font(self.theme.font_family, pal.font_md))
        p.drawText(QRectF(rect.left() + 48, rect.top(), rect.width() - 60, rect.height()),
                   Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self.text)
        p.end()


class ToastHost(QWidget):
    """Слой, в котором живут тосты (поверх содержимого окна)."""

    def __init__(self, theme, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self._toasts: list[Toast] = []

    def show_toast(self, text: str, kind: str = "info", timeout: int = 4200):
        parent = self.parentWidget()
        if parent is None:
            return
        alive = self._alive_toasts()
        toast = Toast(parent, self.theme, text, kind, timeout)
        toast.destroyed.connect(lambda *_a: None)
        self._toasts = alive + [toast]
        toast.show()
        toast.raise_()
        self._layout_toasts()

    def _alive_toasts(self) -> list:
        """Убирает из очереди уже удалённые виджеты (C++ объект мог исчезнуть)."""
        alive = []
        for toast in self._toasts:
            try:
                if toast.isVisible():
                    alive.append(toast)
            except RuntimeError:
                continue
        return alive

    def _layout_toasts(self):
        parent = self.parentWidget()
        if parent is None:
            return
        self._toasts = self._alive_toasts()
        y = parent.height() - 28
        for toast in reversed(self._toasts):
            y -= toast.height() + 10
            toast.move(max(20, (parent.width() - toast.width()) // 2), y)
            toast.raise_()

    def resizeEvent(self, event):  # noqa: N802
        self._layout_toasts()
        super().resizeEvent(event)


# ---------------------------------------------------------------------------
# Выезжающая панель (кастомизация, журнал, справка)
# ---------------------------------------------------------------------------

class _Overlay(QWidget):
    """Затемнение под выезжающей панелью."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)

    def paintEvent(self, event):  # noqa: N802
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(0, 0, 0, 110))
        p.end()


class Sheet(QWidget):
    """
    Правая панель-«шторка» с затемнением фона.

    Содержимое задаётся через `set_content`. Открывается кликом по кнопке в
    шапке, закрывается по клику вне панели или по Esc.
    """

    closed = Signal()

    def __init__(self, theme, parent=None, width: int = 396):
        super().__init__(parent)
        self.theme = theme
        self._width_target = width
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.hide()
        self.offset = 0.0
        self.overlay = _Overlay(parent)
        self.overlay.hide()
        self.overlay.mouseReleaseEvent = self._overlay_click  # type: ignore[assignment]

        self.inner = QWidget(self)
        self.inner.setObjectName("sheetBody")
        self.inner.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.layout_outer = QVBoxLayout(self)
        self.layout_outer.setContentsMargins(0, 0, 0, 0)
        self.layout_outer.addWidget(self.inner)

        self.scroll = QScrollArea(self.inner)
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.scroll.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.scroll.viewport().setAutoFillBackground(False)
        self.scroll.viewport().setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.scroll.setStyleSheet(
            "QScrollArea, QScrollArea > QWidget > QWidget{background:transparent;border:none;}"
            "QScrollBar:vertical{background:transparent;width:8px;}"
            "QScrollBar::handle:vertical{background:rgba(140,160,150,0.30);border-radius:4px;"
            "min-height:36px;}"
            "QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}"
            "QScrollBar::add-page:vertical,QScrollBar::sub-page:vertical{background:transparent;}")
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.content = QWidget()
        self.content.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        # QScrollArea включает автофон у scroll-widget — гасим, иначе под стеклом
        # появляется системный светлый прямоугольник
        self.content.setAutoFillBackground(False)
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(18, 8, 18, 22)
        self.content_layout.setSpacing(14)
        self.content_layout.addStretch(1)
        self.scroll.setWidget(self.content)
        outer = QVBoxLayout(self.inner)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self.scroll)
        self.header = None

    # -- API ---------------------------------------------------------------

    def set_header(self, widget: QWidget):
        if self.header is not None:
            self.header.setParent(None)
        self.header = widget
        self.inner.layout().insertWidget(0, widget)

    def set_content(self, widgets: list[QWidget]):
        for i in reversed(range(self.content_layout.count())):
            item = self.content_layout.itemAt(i)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
        for w in widgets:
            self.content_layout.insertWidget(self.content_layout.count() - 1, w)

    def open(self):
        parent = self.parentWidget()
        if parent is None:
            return
        self.setGeometry(parent.width(), 0, self._width_target, parent.height())
        self.overlay.setGeometry(0, 0, parent.width(), parent.height())
        self.overlay.show()
        self.overlay.raise_()
        self.show()
        self.raise_()
        self.setFixedWidth(self._width_target)
        self.offset = self._width_target
        self._apply_offset(self.offset)
        target = 0.0
        animate(self, "offset", self.offset, target, self.theme.palette.anim_ms,
                self._apply_offset, QEasingCurve.Type.OutCubic)
        self._animate_overlay(0.0, 0.45)
        self.setFocus()

    def close(self):
        if not self.isVisible():
            return
        self._animate_overlay(0.45, 0.0, hide=True)
        animate(self, "offset", self.offset, self._width_target,
                self.theme.palette.anim_ms, self._apply_offset,
                QEasingCurve.Type.InCubic)
        QTimer.singleShot(max(1, self.theme.palette.anim_ms + 20), self._finish_close)

    def _finish_close(self):
        self.hide()
        self.closed.emit()

    def toggle(self):
        self.close() if self.isVisible() else self.open()

    def _apply_offset(self, value: float):
        self.offset = value
        parent = self.parentWidget()
        if parent is None:
            return
        self.move(int(parent.width() - self._width_target + value), 0)
        self.setFixedWidth(self._width_target)

    def _animate_overlay(self, start: float, end: float, hide: bool = False):
        effect = self.overlay.graphicsEffect()
        if effect is None:
            effect = QGraphicsOpacityEffect(self.overlay)
            self.overlay.setGraphicsEffect(effect)
        if hide:
            animate(self, "overlay", start, end, self.theme.palette.anim_ms,
                    lambda v: effect.setOpacity(v))
            QTimer.singleShot(max(1, self.theme.palette.anim_ms), self.overlay.hide)
        else:
            effect.setOpacity(start)
            animate(self, "overlay", start, end, self.theme.palette.anim_ms,
                    lambda v: effect.setOpacity(v))

    def _overlay_click(self, event):
        self.close()

    def keyPressEvent(self, event):  # noqa: N802
        if event.key() == Qt.Key.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(event)

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect())
        p.setBrush(QColor(pal.surface if pal.dark else "#ffffff"))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRect(rect)
        p.setPen(QPen(_with_alpha(pal.text, 0.10), 1.0))
        p.drawLine(QPointF(rect.left() + 0.5, 0), QPointF(rect.left() + 0.5, rect.bottom()))
        p.end()


class SheetHeader(QWidget):
    """Шапка выезжающей панели: иконка, заголовок, подпись, кнопка закрытия."""

    def __init__(self, theme, title: str, subtitle: str = "", icon: str = "",
                 on_close=None, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.setFixedHeight(76)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(18, 12, 12, 12)
        layout.setSpacing(12)
        if icon:
            badge = QLabel(self)
            badge.setFixedSize(36, 36)
            badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            badge.setStyleSheet(
                f"border-radius:11px;"
                f"background:{_rgba(theme.palette.accent, 0.16)};"
            )
            badge.setPixmap(icons.icon_pixmap(icon, 20, theme.palette.accent, 1.8))
            layout.addWidget(badge)
            self.badge = badge
        text_box = QVBoxLayout()
        text_box.setSpacing(2)
        self.title_label = QLabel(title, self)
        self.subtitle_label = QLabel(subtitle, self)
        text_box.addWidget(self.title_label)
        if subtitle:
            text_box.addWidget(self.subtitle_label)
        layout.addLayout(text_box)
        layout.addStretch(1)
        close_btn = IconButton(theme, "close", "Закрыть", 34, parent=self)
        if on_close:
            close_btn.clicked.connect(on_close)
        layout.addWidget(close_btn)
        self._restyle()
        theme.changed.connect(self._restyle)

    def _restyle(self):
        pal = self.theme.palette
        self.title_label.setFont(font(self.theme.font_family, pal.font_lg, QFont.Weight.Bold))
        self.title_label.setStyleSheet(f"color:{pal.text};background:transparent;")
        self.subtitle_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.subtitle_label.setStyleSheet(f"color:{pal.muted};background:transparent;")
        if hasattr(self, "badge"):
            self.badge.setStyleSheet(
                f"border-radius:11px;background:{_rgba(self.theme.palette.accent, 0.16)};")
            self.badge.setPixmap(icons.icon_pixmap("", 1, "#000000"))


def _rgba(hex_color: str, alpha: float) -> str:
    color = QColor(hex_color)
    return f"rgba({color.red()},{color.green()},{color.blue()},{alpha:.3f})"


# ---------------------------------------------------------------------------
# Журнал
# ---------------------------------------------------------------------------

class LogView(QPlainTextEdit):
    """Моноширинный журнал с подсветкой уровней."""

    LEVELS = {
        "ok": "#34e0a1",
        "warn": "#ffc061",
        "error": "#ff6b7f",
        "accent": "",
    }

    def __init__(self, theme, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.setReadOnly(True)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setMaximumBlockCount(1500)
        self._restyle()
        theme.changed.connect(self._restyle)

    def _restyle(self):
        pal = self.theme.palette
        self.setFont(font(self.theme.mono_family, pal.font_xs))
        self.setStyleSheet(
            f"QPlainTextEdit{{background:transparent;color:{pal.muted};"
            f"border:none;padding:2px;}}"
        )

    def append(self, message: str, kind: str = "info"):
        from datetime import datetime

        pal = self.theme.palette
        color = {"ok": pal.good, "warn": pal.warn, "error": pal.bad,
                 "accent": pal.accent}.get(kind, pal.text)
        stamp = datetime.now().strftime("%H:%M:%S")
        safe = (message.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))
        self.appendHtml(
            f'<span style="color:{pal.muted}">[{stamp}]</span> '
            f'<span style="color:{color}">{safe}</span>'
        )
        self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())


# ---------------------------------------------------------------------------
# Элементы панели настроек
# ---------------------------------------------------------------------------

class SectionTitle(QWidget):
    """Заголовок секции в выезжающей панели: иконка, название, подсказка."""

    def __init__(self, theme, title: str, hint: str = "", icon: str = "", parent=None):
        super().__init__(parent)
        self.theme = theme
        self.title = title
        self.hint = hint
        self.icon_name = icon
        self.setFixedHeight(34 if not hint else 40)
        theme.changed.connect(self.update)

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        x = 0
        if self.icon_name:
            pm = icons.icon_pixmap(self.icon_name, 16, pal.accent_text, 1.8)
            p.drawPixmap(QPointF(0, 8), pm)
            x = 24
        p.setPen(QColor(pal.text))
        p.setFont(font(self.theme.font_family, pal.font_md, QFont.Weight.Bold))
        p.drawText(QRectF(x, 0, self.width() - x, 22),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.title)
        if self.hint:
            p.setPen(QColor(pal.muted))
            p.setFont(font(self.theme.font_family, pal.font_xs))
            p.drawText(QRectF(x, 20, self.width() - x, 18),
                       Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.hint)
        p.end()


class SettingRow(QWidget):
    """Строка настройки: подпись слева, элемент управления справа."""

    def __init__(self, theme, title: str, hint: str = "", control: QWidget | None = None,
                 parent=None):
        super().__init__(parent)
        self.theme = theme
        self.title = title
        self.hint = hint
        self.hover = 0.0
        self.setAttribute(Qt.WidgetAttribute.WA_Hover, True)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(12)
        text_box = QVBoxLayout()
        text_box.setSpacing(1)
        self.title_label = QLabel(title, self)
        self.hint_label = QLabel(hint, self) if hint else None
        text_box.addWidget(self.title_label)
        if self.hint_label is not None:
            text_box.addWidget(self.hint_label)
        layout.addLayout(text_box, 1)
        self.control = control
        if control is not None:
            control.setParent(self)
            layout.addWidget(control, 0, Qt.AlignmentFlag.AlignVCenter)
        self.setMinimumHeight(52 if hint else 44)
        self._restyle()
        theme.changed.connect(self._restyle)

    def _restyle(self):
        pal = self.theme.palette
        self.title_label.setFont(font(self.theme.font_family, pal.font_md))
        self.title_label.setStyleSheet(f"color:{pal.text};background:transparent;")
        if self.hint_label is not None:
            self.hint_label.setFont(font(self.theme.font_family, pal.font_xs))
            self.hint_label.setStyleSheet(f"color:{pal.muted};background:transparent;")
        self.update()

    def enterEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 1.0, 140, self._set_hover)

    def leaveEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 0.0, 140, self._set_hover)

    def _set_hover(self, value: float):
        self.hover = value
        self.update()

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        p.setBrush(_with_alpha(pal.text, 0.03 + 0.03 * self.hover))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(rect, pal.r_md, pal.r_md)
        p.end()


class AccentPicker(QWidget):
    """Круглые свотчи акцента + свой цвет через системный диалог."""

    changed = Signal(str)

    def __init__(self, theme, value: str, presets: list[tuple[str, str, str]],
                 parent=None):
        super().__init__(parent)
        self.theme = theme
        self.value = value
        self.presets = presets
        self.hover_index = -1
        self.custom_index = len(presets)
        self.setFixedHeight(42)
        self.setMouseTracking(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        theme.changed.connect(self.update)

    def set_value(self, value: str, emit: bool = False):
        self.value = value.lower()
        self.update()
        if emit:
            self.changed.emit(self.value)

    def _circle_rect(self, index: int) -> QRectF:
        d = 26.0
        gap = 12.0
        return QRectF(index * (d + gap), (self.height() - d) / 2, d, d)

    def _index_at(self, pos) -> int:
        for i in range(len(self.presets) + 1):
            if self._circle_rect(i).adjusted(-4, -4, 4, 4).contains(QPointF(pos)):
                return i
        return -1

    def mouseMoveEvent(self, event):  # noqa: N802
        index = self._index_at(event.position())
        if index != self.hover_index:
            self.hover_index = index
            self.update()

    def leaveEvent(self, event):  # noqa: N802
        self.hover_index = -1
        self.update()

    def mouseReleaseEvent(self, event):  # noqa: N802
        index = self._index_at(event.position())
        if index < 0:
            return
        if index == self.custom_index:
            self._pick_custom()
        else:
            self.set_value(self.presets[index][1], emit=True)

    def _pick_custom(self):
        from PySide6.QtWidgets import QColorDialog

        color = QColorDialog.getColor(QColor(self.value), self, "Акцентный цвет")
        if color.isValid():
            self.set_value(color.name(), emit=True)

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        for i, (_key, color, _label) in enumerate(self.presets):
            rect = self._circle_rect(i)
            selected = color.lower() == self.value
            if i == self.hover_index:
                p.setBrush(_with_alpha(color, 0.18))
                p.setPen(Qt.PenStyle.NoPen)
                p.drawEllipse(rect.adjusted(-5, -5, 5, 5))
            p.setBrush(QColor(color))
            p.setPen(QPen(_with_alpha(pal.text, 0.25) if not selected else QColor(pal.text),
                          2.4 if selected else 1.0))
            p.drawEllipse(rect)
            if selected:
                pm = icons.icon_pixmap("check", 14, pal.on_accent, 2.4)
                p.drawPixmap(QPointF(rect.center().x() - pm.width() / 2,
                                     rect.center().y() - pm.height() / 2), pm)

        custom = self._circle_rect(self.custom_index)
        is_custom = all(c.lower() != self.value for _k, c, _l in self.presets)
        p.setBrush(_with_alpha(pal.text, 0.06))
        p.setPen(QPen(QColor(pal.text) if is_custom else _with_alpha(pal.text, 0.25),
                      2.4 if is_custom else 1.0))
        p.drawEllipse(custom)
        pm = icons.icon_pixmap("palette", 14,
                               pal.text if is_custom else pal.muted, 1.8)
        p.drawPixmap(QPointF(custom.center().x() - pm.width() / 2,
                             custom.center().y() - pm.height() / 2), pm)
        p.end()


class ChoiceCard(QWidget):
    """Крупная карточка-пресет оформления (как блоки тем в zmk)."""

    clicked = Signal(str)

    def __init__(self, theme, key: str, title: str, hint: str = "", icon: str = "",
                 selected: bool = False, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.key = key
        self.title = title
        self.hint = hint
        self.icon_name = icon
        self.selected = selected
        self.hover = 0.0
        self.setFixedHeight(72)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setAttribute(Qt.WidgetAttribute.WA_Hover, True)
        theme.changed.connect(self.update)

    def set_selected(self, value: bool):
        self.selected = value
        self.update()

    def enterEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 1.0, 140, self._set_hover)

    def leaveEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 0.0, 140, self._set_hover)

    def _set_hover(self, value: float):
        self.hover = value
        self.update()

    def mouseReleaseEvent(self, event):  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self.key)

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect()).adjusted(0.75, 0.75, -0.75, -0.75)
        p.setBrush(_with_alpha(pal.text, 0.04 + 0.04 * self.hover))
        p.setPen(QPen(QColor(pal.accent) if self.selected
                      else _with_alpha(pal.text, 0.12), 1.8 if self.selected else 1.0))
        p.drawRoundedRect(rect, pal.r_md, pal.r_md)
        if self.icon_name:
            pm = icons.icon_pixmap(self.icon_name, 18,
                                   pal.accent if self.selected else pal.muted, 1.8)
            p.drawPixmap(QPointF(rect.left() + 12, rect.center().y() - 9), pm)
        text_left = rect.left() + (34 if self.icon_name else 12)
        p.setPen(QColor(pal.text))
        p.setFont(font(self.theme.font_family, pal.font_md, QFont.Weight.DemiBold))
        p.drawText(QRectF(text_left, rect.top() + 14, rect.width() - text_left - 8, 18),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.title)
        if self.hint:
            p.setPen(QColor(pal.muted))
            p.setFont(font(self.theme.font_family, pal.font_xs))
            p.drawText(QRectF(text_left, rect.top() + 34, rect.width() - text_left - 8, 20),
                       Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop, self.hint)
        p.end()


class LabeledSlider(QWidget):
    """Ползунок с подписью значения (используется для интенсивности фона)."""

    changed = Signal(int)

    def __init__(self, theme, minimum: int, maximum: int, value: int, suffix: str = "%",
                 parent=None):
        super().__init__(parent)
        self.theme = theme
        self.suffix = suffix
        self.minimum, self.maximum = minimum, maximum
        self.value = max(minimum, min(maximum, value))
        self.hover = 0.0
        self.dragging = False
        self.setFixedHeight(30)
        self.setMinimumWidth(120)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setAttribute(Qt.WidgetAttribute.WA_Hover, True)
        theme.changed.connect(self.update)

    def set_value(self, value: int, emit: bool = False):
        value = max(self.minimum, min(self.maximum, int(value)))
        if value == self.value:
            return
        self.value = value
        self.update()
        if emit:
            self.changed.emit(value)

    def _ratio(self) -> float:
        span = max(1, self.maximum - self.minimum)
        return (self.value - self.minimum) / span

    def _track_rect(self) -> QRectF:
        return QRectF(10, self.height() / 2 - 3, self.width() - 20, 6)

    def _set_from_pos(self, x: float):
        track = self._track_rect()
        ratio = (x - track.left()) / max(1.0, track.width())
        ratio = max(0.0, min(1.0, ratio))
        self.set_value(round(self.minimum + ratio * (self.maximum - self.minimum)), emit=True)

    def mousePressEvent(self, event):  # noqa: N802
        self.dragging = True
        self._set_from_pos(event.position().x())

    def mouseMoveEvent(self, event):  # noqa: N802
        if self.dragging:
            self._set_from_pos(event.position().x())

    def mouseReleaseEvent(self, event):  # noqa: N802
        self.dragging = False

    def enterEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 1.0, 140, self._set_hover)

    def leaveEvent(self, event):  # noqa: N802
        animate(self, "hover", self.hover, 0.0, 140, self._set_hover)

    def _set_hover(self, value: float):
        self.hover = value
        self.update()

    def paintEvent(self, event):  # noqa: N802
        pal = self.theme.palette
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        track = self._track_rect()
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(_with_alpha(pal.text, 0.10))
        p.drawRoundedRect(track, track.height() / 2, track.height() / 2)
        filled = QRectF(track.left(), track.top(), track.width() * self._ratio(), track.height())
        p.setBrush(QColor(pal.accent))
        p.drawRoundedRect(filled, track.height() / 2, track.height() / 2)
        handle_x = track.left() + track.width() * self._ratio()
        p.setBrush(QColor(pal.surface if pal.dark else "#ffffff"))
        p.setPen(QPen(QColor(pal.accent), 2.4))
        p.drawEllipse(QPointF(handle_x, track.center().y()), 8, 8)
        p.setPen(QColor(pal.muted))
        p.setFont(font(self.theme.font_family, pal.font_xs))
        p.drawText(QRectF(track.right() - 60, 0, 60, 14),
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                   f"{self.value}{self.suffix}")
        p.end()
