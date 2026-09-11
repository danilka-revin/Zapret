"""
Выезжающие панели Qt-интерфейса: оформление, журнал, диагностика и справка.

Кастомизация повторяет идею zmk-videoanalytics: тема, акцент, стекло, плотность,
углы, анимации и размер текста меняются на лету и сохраняются в config.json.
"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (QComboBox, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
                               QTextEdit, QVBoxLayout, QWidget)

from . import icons
from .theme import ACCENTS, PRESETS
from .widgets import (AccentPicker, Card, Chip, ChoiceCard, Divider, EmptyState,
                      GlassButton, LabeledSlider, LogView, SearchField, SectionTitle,
                      SegmentedControl, ServiceRow, SettingRow, Sheet, SheetHeader,
                      Switch, ThinProgress, font)


class TextBlock(QWidget):
    """Абзац с заголовком — используется в справке и подсказках."""

    def __init__(self, theme, title: str = "", body: str = "", icon: str = "", parent=None):
        super().__init__(parent)
        self.theme = theme
        self.icon_name = icon
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        if icon:
            row = QHBoxLayout()
            row.setSpacing(8)
            self.badge = QLabel(self)
            self.badge.setFixedSize(24, 24)
            self.badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            row.addWidget(self.badge)
            self.title_label = QLabel(title, self)
            row.addWidget(self.title_label, 1)
            layout.addLayout(row)
        else:
            self.title_label = QLabel(title, self) if title else None
            if self.title_label is not None:
                layout.addWidget(self.title_label)
        self.body_label = QLabel(body, self) if body else None
        if self.body_label is not None:
            self.body_label.setWordWrap(True)
            layout.addWidget(self.body_label)
        self._restyle()
        theme.changed.connect(self._restyle)

    def _restyle(self):
        pal = self.theme.palette
        if getattr(self, "title_label", None) is not None:
            self.title_label.setFont(font(self.theme.font_family, pal.font_md, QFont.Weight.DemiBold))
            self.title_label.setStyleSheet(f"color:{pal.text};background:transparent;")
        if getattr(self, "body_label", None) is not None:
            self.body_label.setFont(font(self.theme.font_family, pal.font_sm))
            self.body_label.setStyleSheet(f"color:{pal.muted};background:transparent;")
        if hasattr(self, "badge"):
            color = QColor(pal.accent)
            self.badge.setStyleSheet(
                f"border-radius:8px;background:rgba({color.red()},{color.green()},"
                f"{color.blue()},0.16);")
            self.badge.setPixmap(icons.icon_pixmap(self.icon_name, 14, pal.accent_text, 1.8))


def section(theme, title: str, hint: str, icon: str, widgets: list[QWidget]) -> QWidget:
    """Собирает секцию: заголовок + набор строк настроек."""
    from .widgets import SectionTitle

    box = QWidget()
    layout = QVBoxLayout(box)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(8)
    layout.addWidget(SectionTitle(theme, title, hint, icon))
    for w in widgets:
        layout.addWidget(w)
    return box


# ---------------------------------------------------------------------------
# Оформление
# ---------------------------------------------------------------------------

class CustomizerSheet(Sheet):
    """Панель «Оформление и автономика»."""

    def __init__(self, theme, parent=None):
        super().__init__(theme, parent)
        self.theme = theme
        self.set_header(SheetHeader(theme, "Оформление", "Всё меняется сразу и сохраняется",
                                    "palette", on_close=self.close))
        self._build()
        theme.changed.connect(self._sync)

    # -- сборка ------------------------------------------------------------

    def _build(self):
        theme = self.theme
        s = theme.settings

        # Пресеты
        self.preset_cards: dict[str, ChoiceCard] = {}
        presets_widget = QWidget()
        grid = QGridLayout(presets_widget)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(8)
        icons_for = {"aurora": "sparkles", "glass": "droplet", "lime": "zap",
                       "sunset": "sun", "ocean": "globe", "rose": "star",
                       "night": "moon", "contrast": "chart"}
        for index, (key, label, _values) in enumerate(PRESETS):
            card = ChoiceCard(theme, key, label, "готовый набор", icons_for.get(key, "palette"))
            card.clicked.connect(self._apply_preset)
            grid.addWidget(card, index // 2, index % 2)
            self.preset_cards[key] = card

        # Тема и акцент
        self.theme_seg = SegmentedControl(theme, [("light", "Светлая"), ("dark", "Тёмная"),
                                                  ("system", "Система")], s.mode)
        self.theme_seg.changed.connect(lambda v: theme.update(mode=v))
        self.accent_picker = AccentPicker(theme, s.accent, ACCENTS)
        self.accent_picker.changed.connect(lambda v: theme.update(accent=v))

        # Стекло
        self.glass_seg = SegmentedControl(theme, [("off", "Выкл"), ("soft", "Мягкое"),
                                                  ("vivid", "Яркое")], s.glass)
        self.glass_seg.changed.connect(lambda v: theme.update(glass=v))
        self.tint_seg = SegmentedControl(theme, [("auto", "Акцент"), ("cool", "Холодная"),
                                                 ("warm", "Тёплая")], s.glass_tint)
        self.tint_seg.changed.connect(lambda v: theme.update(glass_tint=v))
        self.orbs_switch = Switch(theme, s.orbs)
        self.orbs_switch.toggled.connect(lambda v: theme.update(orbs=v))
        self.orbs_slider = LabeledSlider(theme, 0, 160, s.orbs_intensity)
        self.orbs_slider.changed.connect(lambda v: theme.update(orbs_intensity=v))

        # Детали интерфейса
        self.radius_seg = SegmentedControl(theme, [("soft", "Мягкие"), ("square", "Строгие")],
                                           s.radius)
        self.radius_seg.changed.connect(lambda v: theme.update(radius=v))
        self.density_seg = SegmentedControl(theme, [("comfortable", "Комфортно"),
                                                    ("compact", "Компактно")], s.density)
        self.density_seg.changed.connect(lambda v: theme.update(density=v))
        self.font_seg = SegmentedControl(theme, [("small", "Малый"), ("normal", "Обычный"),
                                                 ("large", "Крупный")], s.font_scale)
        self.font_seg.changed.connect(lambda v: theme.update(font_scale=v))
        self.motion_seg = SegmentedControl(theme, [("full", "Полные"), ("reduced", "Мягкие"),
                                                   ("off", "Выкл")], s.motion)
        self.motion_seg.changed.connect(lambda v: theme.update(motion=v))

        # Автономика
        self.autopilot_switch = Switch(theme, s.autopilot)
        self.autopilot_switch.toggled.connect(lambda v: theme.update(autopilot=v))
        self.autostart_switch = Switch(theme, s.autostart_protection)
        self.autostart_switch.toggled.connect(lambda v: theme.update(autostart_protection=v))
        self.recover_switch = Switch(theme, s.auto_recover)
        self.recover_switch.toggled.connect(lambda v: theme.update(auto_recover=v))
        self.minimized_switch = Switch(theme, s.start_minimized)
        self.minimized_switch.toggled.connect(lambda v: theme.update(start_minimized=v))
        self.tray_switch = Switch(theme, s.tray)
        self.tray_switch.toggled.connect(lambda v: theme.update(tray=v))
        self.notify_switch = Switch(theme, s.notifications)
        self.notify_switch.toggled.connect(lambda v: theme.update(notifications=v))
        self.interval_seg = SegmentedControl(theme, [("30", "30 с"), ("60", "1 мин"),
                                                     ("300", "5 мин")],
                                             str(s.check_interval))
        self.interval_seg.changed.connect(lambda v: theme.update(check_interval=int(v)))

        self.reset_btn = GlassButton(theme, "Сбросить оформление", "rotate", "ghost")
        self.reset_btn.clicked.connect(theme.reset)
        self.share_btn = GlassButton(theme, "Скопировать тему", "copy", "secondary")
        self.share_btn.setToolTip("Скопировать оформление в буфер обмена — можно отправить другу")
        self.share_btn.clicked.connect(self._share_theme)
        self.paste_btn = GlassButton(theme, "Вставить тему", "save", "ghost")
        self.paste_btn.setToolTip("Применить оформление из буфера обмена")
        self.paste_btn.clicked.connect(self._paste_theme)

        self.set_content([
            section(theme, "Пресеты", "Один клик — готовый образ", "sparkles",
                    [presets_widget]),
            section(theme, "Тема и акцент", "Цвет кнопок, свечения и индикаторов", "sun",
                    [self.theme_seg, self.accent_picker]),
            section(theme, "Стекло и фон", "Матовые панели и цветные пятна", "droplet",
                    [SettingRow(theme, "Стекло", "прозрачность карточек", self.glass_seg),
                     SettingRow(theme, "Тонировка", "оттенок стекла", self.tint_seg),
                     SettingRow(theme, "Цветные пятна", "живой фон", self.orbs_switch),
                     SettingRow(theme, "Интенсивность", "яркость пятен", self.orbs_slider)]),
            section(theme, "Детали интерфейса", "Характер карточек и анимаций", "sliders",
                    [SettingRow(theme, "Углы", "мягкие или строгие", self.radius_seg),
                     SettingRow(theme, "Плотность", "отступы и высота строк", self.density_seg),
                     SettingRow(theme, "Размер текста", "читаемость", self.font_seg),
                     SettingRow(theme, "Анимации", "плавность переходов", self.motion_seg)]),
            section(theme, "Автономика", "Минимум настроек — максимум автоматики", "rocket",
                    [SettingRow(theme, "Автопилот", "сам подбирает стратегию обхода",
                                self.autopilot_switch),
                     SettingRow(theme, "Включать при запуске", "защита стартует вместе с приложением",
                                self.autostart_switch),
                     SettingRow(theme, "Автовосстановление", "поднимает защиту, если она упала",
                                self.recover_switch),
                     SettingRow(theme, "Старт в трее", "не открывать окно при автозапуске",
                                self.minimized_switch),
                     SettingRow(theme, "Иконка в трее", "быстрый доступ к вкл/выкл",
                                self.tray_switch),
                     SettingRow(theme, "Уведомления", "сообщать о смене состояния",
                                self.notify_switch),
                     SettingRow(theme, "Проверка сервисов", "как часто проверять доступность",
                                self.interval_seg)]),
            self.reset_btn,
            section(theme, "Поделиться", "Отправьте оформление другу", "send",
                    [self.share_btn, self.paste_btn]),
        ])

    # -- синхронизация -----------------------------------------------------

    def _apply_preset(self, key: str):
        self.theme.apply_preset(key)
        self._sync()

    def _share_theme(self):
        import json

        from PySide6.QtWidgets import QApplication

        QApplication.clipboard().setText(json.dumps(self.theme.export_theme(),
                                                    ensure_ascii=False))

    def _paste_theme(self):
        import json

        from PySide6.QtWidgets import QApplication

        try:
            data = json.loads(QApplication.clipboard().text() or "")
        except ValueError:
            return
        if self.theme.import_theme(data):
            self._sync()

    def _sync(self):
        s = self.theme.settings
        self.theme_seg.set_value(s.mode)
        self.accent_picker.set_value(s.accent)
        self.glass_seg.set_value(s.glass)
        self.tint_seg.set_value(s.glass_tint)
        self.orbs_switch.setChecked(s.orbs, animate_value=False)
        self.orbs_slider.set_value(s.orbs_intensity)
        self.radius_seg.set_value(s.radius)
        self.density_seg.set_value(s.density)
        self.font_seg.set_value(s.font_scale)
        self.motion_seg.set_value(s.motion)
        self.autopilot_switch.setChecked(s.autopilot, animate_value=False)
        self.autostart_switch.setChecked(s.autostart_protection, animate_value=False)
        self.recover_switch.setChecked(s.auto_recover, animate_value=False)
        self.minimized_switch.setChecked(s.start_minimized, animate_value=False)
        self.tray_switch.setChecked(s.tray, animate_value=False)
        self.notify_switch.setChecked(s.notifications, animate_value=False)
        self.interval_seg.set_value(str(s.check_interval))
        for key, card in self.preset_cards.items():
            values = next((v for k, _l, v in PRESETS if k == key), {})
            selected = all(getattr(s, attr, None) == value for attr, value in values.items())
            card.set_selected(bool(selected))


# ---------------------------------------------------------------------------
# Журнал
# ---------------------------------------------------------------------------

class JournalSheet(Sheet):
    """Полный журнал операций с поиском по уровням."""

    def __init__(self, theme, parent=None):
        super().__init__(theme, parent, width=520)
        self.theme = theme
        self.set_header(SheetHeader(theme, "Журнал", "Что делало приложение", "terminal",
                                    on_close=self.close))
        self.filter_seg = SegmentedControl(theme, [("all", "Всё"), ("ok", "Успехи"),
                                                   ("error", "Ошибки")], "all")
        self.filter_seg.changed.connect(lambda _v: self._rebuild())
        self.search = SearchField(theme, "Поиск по журналу…")
        self.search.textChanged.connect(self._on_search)
        self.copy_btn = GlassButton(theme, "Скопировать", "copy", "secondary", compact=True)
        self.copy_btn.clicked.connect(self._copy)
        self.export_btn = GlassButton(theme, "В файл", "save", "ghost", compact=True)
        self.export_btn.clicked.connect(self._export)
        self.clear_btn = GlassButton(theme, "Очистить", "trash", "ghost", compact=True)
        self.clear_btn.clicked.connect(self._clear)

        self.view = LogView(theme)
        self.view.setMinimumHeight(420)
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(8)
        row_layout.addWidget(self.copy_btn)
        row_layout.addWidget(self.export_btn)
        row_layout.addWidget(self.clear_btn)
        row_layout.addStretch(1)
        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)

        self.set_content([self.filter_seg, self.search, row, self.view, self.status_label])
        self.history: list[tuple[float, str, str]] = []
        self._query = ""
        self._restyle_status()
        theme.changed.connect(self._restyle_status)

    def fill(self, history: list[tuple[float, str, str]]):
        self.history = list(history)
        self._rebuild()

    def append(self, message: str, kind: str = "info"):
        self.history.append((time.time(), message, kind))
        del self.history[:-2000]
        if self._visible_for(kind, message):
            self.view.append(message, kind)

    def _restyle_status(self):
        pal = self.theme.palette
        self.status_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.status_label.setStyleSheet(f"color:{pal.muted};background:transparent;")

    def _on_search(self, text: str):
        self._query = (text or "").strip().lower()
        self._rebuild()

    def _visible_for(self, kind: str, message: str = "") -> bool:
        mode = self.filter_seg.value
        if mode == "ok":
            level_ok = kind in ("ok", "accent")
        elif mode == "error":
            level_ok = kind in ("error", "warn")
        else:
            level_ok = True
        if not level_ok:
            return False
        if self._query and self._query not in (message or "").lower():
            return False
        return True

    def _rebuild(self):
        self.view.clear()
        shown = 0
        for _ts, message, kind in self.history:
            if self._visible_for(kind, message):
                self.view.append(message, kind)
                shown += 1
        total = len(self.history)
        self.status_label.setText(f"Показано {shown} из {total}" if total else "")

    def _copy(self):
        from PySide6.QtWidgets import QApplication

        text = "\n".join(f"[{time.strftime('%H:%M:%S', time.localtime(ts))}] {msg}"
                         for ts, msg, _k in self.history)
        QApplication.clipboard().setText(text)

    def _export(self):
        from .. import app_dir

        stamp = time.strftime("%Y%m%d-%H%M")
        path = app_dir() / f"zapret-journal-{stamp}.txt"
        try:
            path.write_text("\n".join(
                f"[{time.strftime('%H:%M:%S', time.localtime(ts))}] {msg}"
                for ts, msg, _k in self.history), encoding="utf-8")
        except OSError as exc:
            self.status_label.setText(f"Не удалось сохранить: {exc}")
            return
        self.status_label.setText(f"Сохранено: {path}")

    def _clear(self):
        self.history.clear()
        self.view.clear()
        self.status_label.setText("")


# ---------------------------------------------------------------------------
# Диагностика
# ---------------------------------------------------------------------------

class DiagnosticsSheet(Sheet):
    """Техническая сводка: что установлено, чем запускается, где лежит."""

    def __init__(self, theme, controller, parent=None):
        super().__init__(theme, parent)
        self.theme = theme
        self.controller = controller
        self.set_header(SheetHeader(theme, "Диагностика", "Технические подробности", "flask",
                                    on_close=self.close))
        self.rows: dict[str, QLabel] = {}
        self.rows_box = QWidget()
        layout = QVBoxLayout(self.rows_box)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(SectionTitle(theme, "Состояние", "обновляется автоматически", "gauge"))
        for key, title in (("run", "Помощник DPI (nfqws)"), ("fw", "Правила файрвола"),
                           ("deps", "Зависимости"), ("svc", "Автозапуск системы"),
                           ("sudo", "Права без пароля"), ("shortcut", "Ярлык на столе"),
                           ("update", "Обновление"), ("backend", "Бэкенд файрвола"),
                           ("iface", "Сетевой интерфейс"), ("strategy", "Стратегия"),
                           ("strategies", "Стратегий"), ("nfqws", "Версия nfqws"),
                           ("session", "Сессия"), ("appdir", "Каталог данных")):
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(10, 6, 10, 6)
            label = QLabel(title)
            value = QLabel("—")
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            value.setMinimumWidth(120)
            value.setMaximumWidth(210)
            row_layout.addWidget(label, 2)
            row_layout.addStretch(1)
            row_layout.addWidget(value, 3)
            self.rows[key] = value
            self._row_labels = getattr(self, "_row_labels", [])
            self._row_labels.append((label, value))
            layout.addWidget(row)

        self.refresh_btn = GlassButton(theme, "Обновить", "refresh", "secondary",
                                       compact=True)
        self.refresh_btn.clicked.connect(controller.refresh_status)
        self.check_btn = GlassButton(theme, "Проверить сервисы", "target", "accent-soft",
                                     compact=True)
        self.check_btn.clicked.connect(lambda: controller.check_services())
        self.autopilot_btn = GlassButton(theme, "Запустить автоподбор", "rocket", "primary",
                                         compact=True)
        self.autopilot_btn.clicked.connect(controller.run_autopilot)
        self.rules_btn = GlassButton(theme, "Правила файрвола", "list", "secondary",
                                     compact=True)
        self.rules_btn.clicked.connect(self._show_rules)
        self.ping_btn = GlassButton(theme, "Пинг", "activity", "ghost", compact=True)
        self.ping_btn.clicked.connect(lambda: controller.run_ping("1.1.1.1"))
        self.speed_btn = GlassButton(theme, "Скорость", "gauge", "ghost", compact=True)
        self.speed_btn.clicked.connect(controller.run_speedtest)
        self.copy_report_btn = GlassButton(theme, "Скопировать отчёт", "copy", "ghost",
                                           compact=True)
        self.copy_report_btn.clicked.connect(self._copy_report)
        self.save_report_btn = GlassButton(theme, "Сохранить отчёт", "save", "ghost",
                                           compact=True)
        self.save_report_btn.clicked.connect(self._save_report)

        buttons = QWidget()
        buttons_layout = QVBoxLayout(buttons)
        buttons_layout.setContentsMargins(0, 0, 0, 0)
        buttons_layout.setSpacing(8)
        top_row = QHBoxLayout()
        top_row.setSpacing(8)
        top_row.addWidget(self.refresh_btn)
        top_row.addWidget(self.check_btn)
        buttons_layout.addLayout(top_row)
        buttons_layout.addWidget(self.autopilot_btn)
        mid_row = QHBoxLayout()
        mid_row.setSpacing(8)
        mid_row.addWidget(self.rules_btn)
        mid_row.addWidget(self.ping_btn)
        mid_row.addWidget(self.speed_btn)
        buttons_layout.addLayout(mid_row)
        bot_row = QHBoxLayout()
        bot_row.setSpacing(8)
        bot_row.addWidget(self.copy_report_btn)
        bot_row.addWidget(self.save_report_btn)
        buttons_layout.addLayout(bot_row)

        self.set_content([self.rows_box, buttons])
        theme.changed.connect(self._restyle)
        controller.status_ready.connect(lambda _s: self.refresh())
        controller.update_info_ready.connect(lambda _i: self.refresh())
        self._restyle()
        self.refresh()

    def _restyle(self):
        pal = self.theme.palette
        for label, value in getattr(self, "_row_labels", []):
            label.setFont(font(self.theme.font_family, pal.font_sm))
            label.setStyleSheet(f"color:{pal.muted};background:transparent;")
            value.setFont(font(self.theme.font_family, pal.font_sm, QFont.Weight.DemiBold))
            value.setStyleSheet(f"color:{pal.text};background:transparent;")

    def refresh(self):
        pal = self.theme.palette
        status = self.controller.status or {}
        running = bool(status.get("running"))
        firewall = bool(status.get("firewall"))
        values = {
            "run": ("работает" if running else "остановлен", running),
            "fw": ("активны" if firewall else "нет правил", firewall),
            "deps": ("готовы" if status.get("deps_ready") else "не скачаны",
                     bool(status.get("deps_ready"))),
            "svc": ("установлен" if status.get("service_installed") else "нет",
                    bool(status.get("service_installed"))),
            "sudo": ("настроены" if status.get("sudo_ok") else "нужен пароль",
                     bool(status.get("sudo_ok"))),
            "backend": (status.get("backend") or "—", bool(status.get("backend"))),
            "iface": (self.controller.cfg.get("interface", "any"), True),
            "strategy": (self.controller.cfg.get("strategy", "—"), True),
            "strategies": (str(status.get("strategies", "—")), True),
            "nfqws": ((status.get("nfqws_version") or "—")[:32], True),
            "appdir": (status.get("app_dir", "—"), True),
        }
        try:
            from .. import session as _session

            values["session"] = (
                "Wayland" if _session.is_wayland() else "X11/другая", True)
        except Exception:  # noqa: BLE001 — диагностика не должна ронять панель
            values["session"] = ("—", True)
        # ярлыки и обновления: именно по ним обычно видно «установил через sudo — и пусто»
        try:
            from .. import integration

            shortcuts = integration.shortcut_status()
            values["shortcut"] = (
                "меню + стол" if shortcuts.get("menu") and shortcuts.get("desktop")
                else ("только меню" if shortcuts.get("menu") else "нет"),
                bool(shortcuts.get("menu") and shortcuts.get("desktop")))
        except Exception:  # noqa: BLE001 — диагностика не должна ронять панель
            values["shortcut"] = ("не проверить", False)
        info = self.controller.update_info or {}
        available = info.get("available")
        if available not in (True, False, None):
            available = None
        texts = {True: "доступно " + str(info.get("remote", ""))[:8],
                 False: "версия актуальна",
                 None: info.get("message") or "ещё не проверялось"}
        values["update"] = (texts[available], True)
        for key, (text_value, good) in values.items():
            label = self.rows.get(key)
            if label is None:
                continue
            label.setText(str(text_value))
            color = pal.good if good else pal.warn
            if key in ("appdir", "iface", "strategy", "strategies", "nfqws",
                       "session", "backend", "update", "shortcut"):
                color = pal.text
            label.setStyleSheet(f"color:{color};background:transparent;")

    def _viewer(self):
        window = self.parentWidget()
        if window is None:
            return None
        return getattr(window, "sheets", {}).get("text")

    def _show_rules(self):
        viewer = self._viewer()
        rules = self.controller.firewall_rules()
        if viewer is None:
            self.controller.log_now(rules, "info")
            return
        window = self.parentWidget()
        viewer.show_text("Правила файрвола", "текущие правила обхода", rules)
        viewer.setGeometry(window.centralWidget().rect())
        self.close()
        viewer.open()

    def _copy_report(self):
        from PySide6.QtWidgets import QApplication

        QApplication.clipboard().setText(self.controller.report_text())
        self.controller.log_now("Отчёт скопирован в буфер обмена.", "ok")

    def _save_report(self):
        self.controller.export_report()


# ---------------------------------------------------------------------------
# Справка
# ---------------------------------------------------------------------------

class HelpSheet(Sheet):
    """Короткая справка для человека, который не хочет читать README."""

    def __init__(self, theme, controller, parent=None):
        super().__init__(theme, parent)
        self.theme = theme
        self.controller = controller
        self.set_header(SheetHeader(theme, "Справка", "Три шага и всё работает", "help",
                                    on_close=self.close))

        self.perm_btn = GlassButton(theme, "Настроить права без пароля", "key", "primary")
        self.open_btn = GlassButton(theme, "Открыть README", "external", "ghost")

        blocks = [
            TextBlock(theme, "1. Большая кнопка", "Один клик включает обход DPI, "
                      "повторный — выключает. Больше ничего нажимать не нужно.",
                      "power"),
            TextBlock(theme, "2. Автопилот", "При включении приложение само перебирает "
                      "стратегии Flowseal и оставляет ту, где YouTube, Discord и Telegram "
                      "работают быстрее всего.", "rocket"),
            TextBlock(theme, "3. Автономность", "Автозапуск, автовосстановление и проверка "
                      "сервисов по расписанию включаются в разделе «Оформление» → "
                      "«Автономика».", "shield-check"),
            TextBlock(theme, "Права без пароля", "Чтобы кнопка работала без запроса пароля, "
                      "настройте NOPASSWD (один раз, потребуется пароль sudo).", "key"),
            TextBlock(theme, "Если что-то не так", "Откройте «Диагностику»: там видно, "
                      "запущен ли nfqws, активны ли правила и настроены ли права.", "flask"),
            TextBlock(theme, "Защита живёт отдельно", "Обход работает в отдельном "
                      "процессе, поэтому закрытие окна или выход из приложения его не "
                      "выключают. Выключить обход можно кнопкой или из меню трея.",
                      "plug"),
            TextBlock(theme, "Важно", "Приложение обходит искусственное замедление "
                      "легитимных сервисов. Используйте там, где это не запрещено законом.",
                      "info"),
            TextBlock(theme, "Горячие клавиши",
                      "Ctrl+P — вкл/выкл · Ctrl+R — проверка · Ctrl+T — подбор под сайт · "
                      "Ctrl+B — стратегии · Ctrl+E — сеть · Ctrl+, — оформление · "
                      "Ctrl+L — журнал · Ctrl+D — диагностика · Esc — закрыть панель",
                      "zap"),
            self.perm_btn,
            self.open_btn,
        ]
        self.set_content(blocks)

    def set_actions(self, on_permissions, on_readme):
        self.perm_btn.clicked.connect(on_permissions)
        self.open_btn.clicked.connect(on_readme)


# ---------------------------------------------------------------------------
# Подбор стратегии под конкретный сайт или группу сайтов
# ---------------------------------------------------------------------------

class PresetCard(Card):
    """Карточка пресета: домены и стратегия, подобранная именно под них."""

    def __init__(self, theme, preset: dict, on_apply, on_check, on_remove, parent=None):
        super().__init__(theme, preset["name"], "", "star", parent)
        self.preset = preset
        self.setToolTip(", ".join(preset["hosts"]))

        self.apply_btn = GlassButton(theme, "Применить", "zap", "secondary", compact=True)
        self.apply_btn.clicked.connect(lambda: on_apply(preset["name"]))
        self.check_btn = GlassButton(theme, "", "refresh", "ghost", compact=True,
                                     icon_only=True)
        self.check_btn.setToolTip("Проверить пресет заново")
        self.check_btn.clicked.connect(lambda: on_check(preset["name"]))
        self.remove_btn = GlassButton(theme, "", "trash", "ghost", compact=True,
                                      icon_only=True)
        self.remove_btn.setToolTip("Удалить пресет")
        self.remove_btn.clicked.connect(lambda: on_remove(preset["name"]))
        if hasattr(self, "header_row"):
            self.header_row.addSpacing(6)
            self.header_row.addWidget(self.apply_btn)
            self.header_row.addWidget(self.check_btn)
            self.header_row.addWidget(self.remove_btn)

        strategy = preset.get("strategy") or "стратегия ещё не подобрана"
        ok, total = preset.get("ok", 0), preset.get("total", 0) or len(preset["hosts"])
        state = "ok" if total and ok >= total else ("warn" if ok else "idle")
        when = _ago(preset.get("checked_at") or 0)
        if hasattr(self, "subtitle_label"):
            self.subtitle_label.setText(f"{strategy} · сайтов ок {ok}/{total} · "
                                        f"{preset.get('avg_ms', 0):.0f} мс" +
                                        (f" · проверено {when}" if when else ""))
            self.subtitle_label.setStyleSheet(
                f"color:{theme.palette.ok if state == 'ok' else theme.palette.muted};"
                "background:transparent;")
        hosts = preset["hosts"]
        shown = ", ".join(hosts[:3]) + (f" и ещё {len(hosts) - 3}" if len(hosts) > 3 else "")
        self.hosts_label = QLabel(shown, self)
        self.hosts_label.setWordWrap(True)
        self.body.addWidget(self.hosts_label)
        self._restyle_hosts()
        theme.changed.connect(self._restyle_hosts)

    def _restyle_hosts(self):
        pal = self.theme.palette
        self.hosts_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.hosts_label.setStyleSheet(f"color:{pal.muted};background:transparent;")


def _ago(stamp: float) -> str:
    """«5 минут назад» — человеческое время последней проверки."""
    if not stamp:
        return ""
    delta = max(0.0, time.time() - stamp)
    if delta < 90:
        return "только что"
    if delta < 3600:
        return f"{int(delta // 60)} мин назад"
    if delta < 86400:
        return f"{int(delta // 3600)} ч назад"
    return f"{int(delta // 86400)} дн назад"


class TargetSheet(Sheet):
    """
    «Подбор под сайт»: стратегия под домен, под готовую группу сервисов
    (YouTube, Discord, Telegram…) или сразу под несколько групп.

    Найденный результат можно сохранить пресетом — тогда в следующий раз
    достаточно нажать «Применить», без повторного перебора.
    """

    def __init__(self, theme, controller, parent=None):
        super().__init__(theme, parent, width=520)
        self.theme = theme
        self.controller = controller
        self.result_rows: list[ServiceRow] = []
        self.site_rows: list[ServiceRow] = []
        self.preset_cards: list[PresetCard] = []
        self.group_chips: dict[str, Chip] = {}
        self.custom_rows: list[tuple[Chip, object]] = []
        self._running = False
        self.last_report: dict | None = None

        self.set_header(SheetHeader(theme, "Подбор под сайт",
                                    "стратегия под сайт или группу сервисов", "target",
                                    on_close=self.close))
        self._build()
        controller.targets_started.connect(self._on_started)
        controller.targets_step.connect(self._on_step)
        controller.targets_done.connect(self._on_done)
        controller.finished.connect(self._on_finished)
        controller.presets_changed.connect(lambda _items: self._rebuild_presets())
        theme.changed.connect(self._restyle)

    # -- сборка ------------------------------------------------------------

    def _build(self):
        theme = self.theme

        self.input = QLineEdit()
        self.input.setPlaceholderText("rutracker.org, https://site.ru/page или несколько "
                                      "через запятую")
        self.input.setClearButtonEnabled(True)
        self.input.returnPressed.connect(self._run)
        self.input.setMinimumHeight(40)
        self.input.textChanged.connect(lambda _text: self._refresh_preset_name())

        self.run_btn = GlassButton(theme, "Проверить стратегии", "rocket", "primary")
        self.run_btn.clicked.connect(self._run)
        self.stop_btn = GlassButton(theme, "Остановить", "stop", "ghost")
        self.stop_btn.clicked.connect(self._stop)
        self.stop_btn.setVisible(False)
        self.progress = ThinProgress(theme)

        self.apply_switch = Switch(theme, True)
        self.apply_switch.setToolTip("Сразу применять лучшую найденную стратегию")

        self.groups_label = QLabel("Готовые группы — выберите одну или несколько")
        self.groups_grid = QGridLayout()
        self.groups_grid.setSpacing(6)

        self.selection_label = QLabel("")
        self.btn_run_groups = GlassButton(theme, "Проверить выбранные группы", "target",
                                         "secondary")
        self.btn_run_groups.setEnabled(False)
        self.btn_run_groups.clicked.connect(self._run_groups)

        # Своя группа: название + домены из поля ввода
        self.group_name = QLineEdit()
        self.group_name.setPlaceholderText("Название своей группы, например «Работа»")
        self.group_name.setMinimumHeight(36)
        self.group_name.returnPressed.connect(self._save_group)
        self.btn_save_group = GlassButton(theme, "Сохранить группу", "star", "ghost",
                                          compact=True)
        self.btn_save_group.clicked.connect(self._save_group)
        self.custom_box = QWidget()
        self.custom_layout = QVBoxLayout(self.custom_box)
        self.custom_layout.setContentsMargins(0, 0, 0, 0)
        self.custom_layout.setSpacing(6)
        self.my_groups_label = QLabel("Мои группы")
        self.custom_title = SectionTitle(theme, "Мои группы", "свой список сайтов", "users")

        self.status_label = QLabel("Введите сайт, выберите группу или несколько групп.")
        self.status_label.setWordWrap(True)

        # Сохранение результата пресетом
        self.preset_name = QLineEdit()
        self.preset_name.setPlaceholderText("Название пресета, например «Видео и чат»")
        self.preset_name.setMinimumHeight(36)
        self.preset_name.returnPressed.connect(self._save_preset)
        self._preset_auto = True
        self.preset_name.textEdited.connect(lambda _text: setattr(self, "_preset_auto", False))
        self.btn_save_preset = GlassButton(theme, "Сохранить пресет", "star", "secondary",
                                           compact=True)
        self.btn_save_preset.clicked.connect(self._save_preset)

        self.presets_title = SectionTitle(theme, "Пресеты", "домены и стратегия под них",
                                         "star")
        self.presets_box = QWidget()
        self.presets_layout = QVBoxLayout(self.presets_box)
        self.presets_layout.setContentsMargins(0, 0, 0, 0)
        self.presets_layout.setSpacing(8)
        self.presets_empty = QLabel("Пока пусто: подберите стратегию и сохраните её "
                                    "пресетом — вернуться к ней можно одним нажатием.")
        self.presets_empty.setWordWrap(True)
        self.presets_layout.addWidget(self.presets_empty)

        self.results_box = QWidget()
        self.results_layout = QVBoxLayout(self.results_box)
        self.results_layout.setContentsMargins(0, 0, 0, 0)
        self.results_layout.setSpacing(6)
        self.results_title = SectionTitle(theme, "Стратегии", "от лучшей к худшей", "layers")

        self.sites_box = QWidget()
        self.sites_layout = QVBoxLayout(self.sites_box)
        self.sites_layout.setContentsMargins(0, 0, 0, 0)
        self.sites_layout.setSpacing(6)
        self.sites_title = SectionTitle(theme, "Сайты", "что ответило и как быстро", "globe")

        self.history_grid = QGridLayout()
        self.history_grid.setSpacing(6)
        self.history_label = QLabel("Недавние запросы")

        group_holder = _label_holder(theme, self.groups_label, self.groups_grid)
        self.set_content([
            section(theme, "Что проверяем", "домен или группа сайтов", "compass",
                    [self.input,
                     SettingRow(theme, "Применять лучшую", "сразу переключить обход на неё",
                                _switch_holder(theme, self.apply_switch)),
                     self.run_btn,
                     self.stop_btn,
                     self.progress,
                     group_holder,
                     self.selection_label,
                     self.btn_run_groups,
                     _label_holder(theme, self.custom_title, self.custom_row_layout()),
                     ]),
            self.status_label,
            section(theme, "Свой пресет", "сохранить результат под своим именем", "star",
                    [self.preset_name, self.btn_save_preset]),
            self.presets_title,
            self.presets_box,
            self.results_title,
            self.results_box,
            self.sites_title,
            self.sites_box,
            _label_holder(theme, self.history_label, self.history_grid),
        ])
        self._build_groups()
        self._rebuild_presets()
        self._rebuild_history()
        self._update_selection()

    def custom_row_layout(self) -> QVBoxLayout:
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        row = QHBoxLayout()
        row.setSpacing(6)
        row.addWidget(self.group_name, 1)
        row.addWidget(self.btn_save_group)
        layout.addLayout(row)
        layout.addWidget(self.custom_box)
        return layout

    def _build_groups(self):
        """Чипы групп: встроенные сервисы и группы пользователя."""
        kept = {key for key, chip in self.group_chips.items() if chip.isChecked()}
        while self.groups_grid.count():
            item = self.groups_grid.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
        self.group_chips = {}
        self.custom_rows = []
        while self.custom_layout.count():
            item = self.custom_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)

        groups = self.controller.groups()
        builtin = [g for g in groups if not g.get("custom")]
        custom = [g for g in groups if g.get("custom")]
        for index, group in enumerate(builtin):
            chip = self._make_group_chip(group)
            self.groups_grid.addWidget(chip, index // 2, index % 2)
        for group in custom:
            row = QWidget()
            layout = QHBoxLayout(row)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(6)
            chip = self._make_group_chip(group)
            chip.clicked.connect(lambda _checked=False, k=group["key"]: self._update_selection())
            trash = GlassButton(self.theme, "", "trash", "ghost", compact=True,
                                icon_only=True)
            trash.setToolTip("Удалить группу")
            trash.clicked.connect(lambda _checked=False, k=group["key"]: self._remove_group(k))
            layout.addWidget(chip)
            layout.addStretch(1)
            layout.addWidget(trash)
            self.custom_layout.addWidget(row)
            self.custom_rows.append((chip, trash))
        # Отмеченные группы остаются выбранными после пересборки списка.
        for key in kept:
            chip = self.group_chips.get(key)
            if chip is not None:
                chip.setChecked(True)
        self.custom_title.setVisible(bool(custom))

    def _make_group_chip(self, group: dict) -> Chip:
        chip = Chip(self.theme, group["title"], group["icon"], "ghost", checkable=True)
        chip.setToolTip(", ".join(group["hosts"][:5])
                        + ("…" if len(group["hosts"]) > 5 else ""))
        chip.toggled.connect(lambda _checked: self._update_selection())
        self.group_chips[group["key"]] = chip
        return chip

    def _rebuild_presets(self):
        for card in self.preset_cards:
            self.presets_layout.removeWidget(card)
            card.setParent(None)
        self.preset_cards = []
        presets = self.controller.site_presets()
        for preset in presets:
            card = PresetCard(self.theme, preset,
                              on_apply=self._apply_preset,
                              on_check=self._check_preset,
                              on_remove=self._remove_preset)
            self.presets_layout.addWidget(card)
            self.preset_cards.append(card)
        self.presets_empty.setVisible(not presets)

    def _rebuild_history(self):
        while self.history_grid.count():
            item = self.history_grid.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
        history = self.controller.target_history()
        self.history_label.setVisible(bool(history))
        for index, query in enumerate(history[:6]):
            chip = Chip(self.theme, query if len(query) <= 28 else query[:26] + "…",
                        "clock")
            chip.setToolTip(query)
            chip.clicked.connect(lambda _checked=False, q=query: self._run_query(q))
            self.history_grid.addWidget(chip, index // 2, index % 2)

    # -- выбор групп -------------------------------------------------------

    def selected_groups(self) -> list[str]:
        return [key for key, chip in self.group_chips.items() if chip.isChecked()]

    def _update_selection(self):
        keys = self.selected_groups()
        hosts = self.controller.group_hosts(keys)
        self.btn_run_groups.setEnabled(bool(hosts) and not self._running)
        if keys:
            titles = [chip.text() for chip in self.group_chips.values() if chip.isChecked()]
            text = f"Выбрано: {' + '.join(titles)} · доменов: {len(hosts)}"
        else:
            text = "Можно выбрать сразу несколько групп — стратегия подберётся под все."
        self.selection_label.setText(text)
        self._refresh_preset_name()

    def _refresh_preset_name(self):
        """Подсказывает имя пресета, пока пользователь не ввёл своё."""
        if not self._preset_auto:
            return
        keys = self.selected_groups()
        hosts = self._targets_for_saving()
        if not keys and not hosts:
            self.preset_name.clear()
            return
        self.preset_name.setText(self._suggested_name(keys, hosts))

    def _suggested_name(self, keys: list[str], hosts: list[str]) -> str:
        from .. import presets as presets_mod

        return presets_mod.suggested_preset_name(self.controller.cfg, keys, hosts)

    def _typed_hosts(self) -> list[str]:
        from .. import checks

        return checks.parse_targets(self.input.text())

    def _current_targets(self) -> list[str]:
        """Домены из поля ввода + домены выбранных групп."""
        hosts = list(self._typed_hosts())
        for host in self.controller.group_hosts(self.selected_groups()):
            if host not in hosts:
                hosts.append(host)
        return hosts

    def _targets_for_saving(self) -> list[str]:
        """Что положить в свою группу: введённые сайты, иначе — выбранные группы."""
        typed = self._typed_hosts()
        return typed or self.controller.group_hosts(self.selected_groups())

    # -- запуск ------------------------------------------------------------

    def _run(self):
        self._run_query(self.input.text())

    def _run_groups(self):
        keys = self.selected_groups()
        if not keys:
            self.status_label.setText("Сначала отметьте группу — например YouTube или Discord.")
            return
        self._run_query("", groups=keys)

    def _run_query(self, query: str, groups: list[str] | None = None):
        if self._running:
            self.controller.log_now("Подбор уже идёт — дождитесь результата.", "warn")
            return
        hosts = self._current_targets() if groups is None else (
            self.controller.group_hosts(groups))
        if not hosts:
            self.status_label.setText("Сначала введите сайт, например rutracker.org, "
                                      "или выберите группу.")
            return
        # Если для этих доменов уже есть пресет — проверяем его же стратегией
        # и обновляем результат, а не начинаем перебор с нуля.
        existing = self._matching_preset(hosts)
        self.controller.test_targets(query, apply_best=self.apply_switch.isChecked(),
                                     hosts=hosts if groups else None,
                                     groups=groups,
                                     prefer=existing["strategy"] if existing else "",
                                     preset_name=existing["name"] if existing else "")

    def _matching_preset(self, hosts: list[str]) -> dict | None:
        wanted = set(hosts)
        for preset in self.controller.site_presets():
            if set(preset["hosts"]) == wanted:
                return preset
        return None

    def _save_group(self):
        title = self.group_name.text().strip()
        hosts = self._targets_for_saving()
        if not title:
            self.status_label.setText("Введите название группы — например «Работа».")
            self.group_name.setFocus()
            return
        if not hosts:
            self.status_label.setText("Сначала введите домены для группы — хотя бы один.")
            return
        group = self.controller.add_group(title, hosts)
        if group is None:
            self.status_label.setText("Не удалось сохранить группу — смотрите журнал.")
            return
        self.group_name.clear()
        self._build_groups()
        self.status_label.setText(f"Группа «{group['title']}» сохранена: "
                                  f"{len(group['hosts'])} домен(ов). "
                                  f"Отметьте её чипом выше.")

    def _remove_group(self, key: str):
        if self.controller.remove_group(key):
            self._build_groups()
            self._update_selection()

    def _save_preset(self):
        name = self.preset_name.text().strip()
        if not name:
            self.status_label.setText("Введите название пресета, например «Видео и чат».")
            self.preset_name.setFocus()
            return
        report = self.last_report or {}
        hosts = self._targets_for_saving()
        if report.get("results"):
            hosts = [item.get("host", "") for item in report["results"]] or hosts
        if not hosts:
            self.status_label.setText("Нечего сохранять: сначала подберите стратегию "
                                      "под сайты или группы.")
            return
        preset = self.controller.save_preset(
            name, hosts, report.get("strategy", ""), report.get("ok", 0),
            report.get("total", len(hosts)), report.get("avg_ms", 0.0))
        if preset:
            self.status_label.setText(f"Пресет «{preset['name']}» сохранён: "
                                      f"{preset['strategy'] or 'без стратегии'}.")

    def _apply_preset(self, name: str):
        if self.controller.apply_preset(name):
            self.status_label.setText(f"Пресет «{name}» применён.")

    def _check_preset(self, name: str):
        if self._running:
            self.controller.log_now("Подбор уже идёт — дождитесь результата.", "warn")
            return
        self.controller.check_preset(name)

    def _remove_preset(self, name: str):
        if self.controller.remove_preset(name):
            self.status_label.setText(f"Пресет «{name}» удалён.")

    # -- реакция контроллера -----------------------------------------------

    def _stop(self):
        self.controller.cancel_operation()

    def _on_started(self, hosts: list):
        self._running = True
        self.run_btn.setEnabled(False)
        self.run_btn.setText("Подбираю…")
        self.stop_btn.setVisible(True)
        self.progress.set_value(0.02)
        self.btn_run_groups.setEnabled(False)
        self.status_label.setText(
            f"Проверяю {len(hosts)} домен(ов): {', '.join(hosts[:4])}"
            + ("…" if len(hosts) > 4 else ""))
        self._clear_rows(self.results_layout, self.result_rows)
        self.result_rows = []
        self._clear_rows(self.sites_layout, self.site_rows)
        self.site_rows = []
        self.results_title.setVisible(False)
        self.sites_title.setVisible(False)

    def _on_step(self, step: dict):
        self.status_label.setText(
            f"Стратегия {step.get('index')} из {step.get('total')}: "
            f"{step.get('strategy')} — проверяю сайты…")
        try:
            self.progress.set_value(float(step.get("index", 0)) / max(1, int(step.get("total", 1))))
        except (TypeError, ValueError):
            pass

    def _on_done(self, report: dict):
        self._running = False
        self.last_report = report
        self.run_btn.setEnabled(True)
        self.run_btn.setText("Проверить стратегии")
        self.stop_btn.setVisible(False)
        self.progress.set_value(1.0)
        self.btn_run_groups.setEnabled(bool(self.selected_groups()))
        best = report.get("strategy", "")
        self.status_label.setText(
            f"Лучшая стратегия: {best} · успешно {report.get('ok')} из "
            f"{report.get('total')} · средняя задержка {report.get('avg_ms', 0):.0f} мс"
            + (" · применена" if report.get("applied") else " · не применялась"))

        tries = [t for t in report.get("tries", []) if "error" not in t]
        tries.sort(key=lambda t: (-t.get("ok", 0), t.get("avg_ms", 99999)))
        best_try = next((t for t in tries if t.get("strategy") == best), None)
        self.results_title.setVisible(bool(tries))
        for item in tries:
            row = ServiceRow(self.theme, item["strategy"], item["strategy"], "layers")
            state = "ok" if item.get("ok") == report.get("total") else (
                "warn" if item.get("ok", 0) else "bad")
            row.set_status(state, f"сайтов ок: {item.get('ok')}/{report.get('total')} · "
                                  f"средняя задержка {item.get('avg_ms', 0):.0f} мс")
            if item["strategy"] == best:
                row.set_badge("ЛУЧШАЯ")
            self.results_layout.addWidget(row)
            self.result_rows.append(row)

        sites = (best_try or {}).get("results") or report.get("results", [])
        self.sites_title.setVisible(bool(sites))
        for site in sites:
            row = ServiceRow(self.theme, site.get("host", ""), site.get("host", ""), "globe")
            row.set_status(site.get("state", "idle"), site.get("detail", ""))
            self.sites_layout.addWidget(row)
            self.site_rows.append(row)
        self._rebuild_history()
        if not self.preset_name.text().strip() and report.get("what"):
            self.preset_name.setText(report["what"])
        if self.controller.site_presets():
            self._rebuild_presets()

    def _on_finished(self, key: str, success: bool, message: str):
        if key != "targets":
            return
        self._running = False
        self.run_btn.setEnabled(True)
        self.run_btn.setText("Проверить стратегии")
        self.stop_btn.setVisible(False)
        if success:
            self.progress.set_value(1.0)
        self.btn_run_groups.setEnabled(bool(self.selected_groups()))
        if not success:
            self.status_label.setText(message or "Подбор не удался — смотрите журнал.")
        self.last_report = self.controller.target_report if success else None

    # -- вспомогательное ---------------------------------------------------

    def _clear_rows(self, layout, rows: list):
        for row in rows:
            layout.removeWidget(row)
            row.setParent(None)
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)

    def _restyle(self):
        pal = self.theme.palette
        self.status_label.setFont(font(self.theme.font_family, pal.font_sm))
        self.status_label.setStyleSheet(f"color:{pal.muted};background:transparent;")
        self.selection_label.setFont(font(self.theme.font_family, pal.font_xs,
                                          QFont.Weight.DemiBold))
        self.selection_label.setStyleSheet(f"color:{pal.text};background:transparent;")
        self.history_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.history_label.setStyleSheet(f"color:{pal.muted};background:transparent;")
        self.presets_empty.setFont(font(self.theme.font_family, pal.font_xs))
        self.presets_empty.setStyleSheet(f"color:{pal.muted};background:transparent;")


def _switch_holder(theme, switch: Switch) -> QWidget:
    holder = QWidget()
    layout = QHBoxLayout(holder)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.addWidget(switch)
    holder.setFixedSize(switch.sizeHint())
    return holder


def _label_holder(theme, label: QLabel, grid: QGridLayout) -> QWidget:
    box = QWidget()
    layout = QVBoxLayout(box)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(6)
    label.setFont(font(theme.font_family, theme.palette.font_xs))
    label.setStyleSheet(f"color:{theme.palette.muted};background:transparent;")
    layout.addWidget(label)
    layout.addLayout(grid)
    return box


# ---------------------------------------------------------------------------
# Таблица результатов подбора
# ---------------------------------------------------------------------------

from PySide6.QtWidgets import QTableWidget, QTableWidgetItem, QHeaderView


class ResultsSheet(Sheet):
    """Красивое окно-таблица с результатами автоподбора или подбора под сайт."""

    def __init__(self, theme, parent=None):
        super().__init__(theme, parent, width=640)
        self.theme = theme
        self.set_header(SheetHeader(theme, "Результаты подбора",
                                    "что работает, что нет — в одной таблице", "layers",
                                    on_close=self.close))
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(["Стратегия", "ОК", "Всего", "Задержка", "Статус"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setMinimumHeight(340)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.ResizeToContents)
        for col in (1, 2, 3):
            header.setSectionResizeMode(col, QHeaderView.ResizeMode.Fixed)
        self.table.setColumnWidth(1, 50)
        self.table.setColumnWidth(2, 50)
        self.table.setColumnWidth(3, 90)
        self.table.setColumnWidth(4, 90)
        self.summary_label = QLabel("")
        self.summary_label.setWordWrap(True)
        self.apply_btn = GlassButton(theme, "Применить лучшую", "zap", "primary",
                                     compact=True)
        self.apply_btn.clicked.connect(self._apply_best)
        self.copy_btn = GlassButton(theme, "Скопировать", "copy", "ghost", compact=True)
        self.copy_btn.clicked.connect(self._copy_table)
        buttons = QWidget()
        buttons_layout = QHBoxLayout(buttons)
        buttons_layout.setContentsMargins(0, 0, 0, 0)
        buttons_layout.setSpacing(8)
        buttons_layout.addWidget(self.apply_btn, 1)
        buttons_layout.addWidget(self.copy_btn, 1)
        self.set_content([self.summary_label, self.table, buttons])
        self._report: dict = {}
        theme.changed.connect(self._restyle)
        self._restyle()

    def _controller(self):
        window = self.parentWidget()
        return getattr(window, "controller", None) if window is not None else None

    def _apply_best(self):
        controller = self._controller()
        best = (self._report or {}).get("strategy", "")
        if controller is not None and best:
            controller.set_strategy(best)
        self.close()

    def _copy_table(self):
        from PySide6.QtWidgets import QApplication

        report = self._report or {}
        total = report.get("total", 0)
        lines = [f"Лучшая: {report.get('strategy', '—')} "
                 f"({report.get('ok', 0)}/{total}, {report.get('avg_ms', 0):.0f} мс)"]
        for item in sorted((t for t in report.get("tries", []) if "error" not in t),
                           key=lambda t: (-t.get("ok", 0), t.get("avg_ms", 99999))):
            lines.append(f"{item.get('strategy', '—')}: {item.get('ok', 0)}/{total}, "
                         f"{item.get('avg_ms', 0):.0f} мс")
        QApplication.clipboard().setText("\n".join(lines))

    def fill_report(self, report: dict):
        self._report = report
        self.summary_label.setText(
            f"Лучшая: {report.get('strategy', '—')} · "
            f"успешно {report.get('ok', 0)} из {report.get('total', 0)} · "
            f"средняя задержка {report.get('avg_ms', 0):.0f} мс"
            + (" · применена" if report.get("applied") else ""))
        tries = [t for t in report.get("tries", []) if "error" not in t]
        tries.sort(key=lambda t: (-t.get("ok", 0), t.get("avg_ms", 99999)))
        best = report.get("strategy", "")
        total = report.get("total", 0) or len(tries)
        self.table.setRowCount(len(tries))
        pal = self.theme.palette
        for row_idx, item in enumerate(tries):
            strategy = item.get("strategy", "—")
            ok = item.get("ok", 0)
            avg = item.get("avg_ms", 0.0)
            status_text = "ОК" if ok == total else ("Частично" if ok > 0 else "Нет")
            status_color = pal.good if ok == total else (pal.warn if ok > 0 else pal.bad)
            # Стратегия
            cell = QTableWidgetItem(strategy)
            cell.setFont(font(self.theme.font_family, pal.font_sm,
                              QFont.Weight.DemiBold))
            cell.setForeground(QColor(pal.text))
            if strategy == best:
                cell.setBackground(QColor(pal.accent))
                cell.setForeground(QColor(pal.on_accent))
            self.table.setItem(row_idx, 0, cell)
            # ОК
            cell_ok = QTableWidgetItem(str(ok))
            cell_ok.setFont(font(self.theme.font_family, pal.font_sm))
            cell_ok.setForeground(QColor(pal.good))
            cell_ok.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(row_idx, 1, cell_ok)
            # Всего
            cell_total = QTableWidgetItem(str(total))
            cell_total.setFont(font(self.theme.font_family, pal.font_sm))
            cell_total.setForeground(QColor(pal.muted))
            cell_total.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(row_idx, 2, cell_total)
            # Задержка
            cell_avg = QTableWidgetItem(f"{avg:.0f} мс")
            cell_avg.setFont(font(self.theme.mono_family, pal.font_sm))
            cell_avg.setForeground(QColor(pal.accent_text))
            cell_avg.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(row_idx, 3, cell_avg)
            # Статус
            cell_status = QTableWidgetItem(status_text)
            cell_status.setFont(font(self.theme.font_family, pal.font_sm, QFont.Weight.DemiBold))
            cell_status.setForeground(QColor(status_color))
            cell_status.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(row_idx, 4, cell_status)
        self.table.resizeRowsToContents()

    def _restyle(self):
        pal = self.theme.palette
        self.summary_label.setFont(font(self.theme.font_family, pal.font_md,
                                        QFont.Weight.DemiBold))
        self.summary_label.setStyleSheet(f"color:{pal.text};background:transparent;")
        self.table.setStyleSheet(
            f"QTableWidget{{background:transparent;color:{pal.text};"
            f"gridline-color:{pal.line_strong};border:none;border-radius:10px;}}"
            f"QHeaderView::section{{background:{pal.surface_2};color:{pal.text};"
            f"font-weight:bold;padding:8px;border:none;}}"
            f"QTableWidget::item{{padding:6px;border-bottom:1px solid {pal.line_strong};}}"
        )
        self.table.setAlternatingRowColors(True)
        alt = QColor(pal.surface_2).lighter(105) if not pal.dark else QColor(pal.surface_3).darker(105)
        alt.setAlphaF(0.5)
        self.table.setPalette(type(self.table)().palette())
        # Для простоты не меняем палитру таблицы отдельно

    def open(self):
        super().open()
        self.raise_()


# ---------------------------------------------------------------------------
# Универсальный просмотр текста (правила файрвола, превью стратегии, чейнджлог)
# ---------------------------------------------------------------------------

class TextSheet(Sheet):
    """Панель с моноширинным текстом и кнопкой копирования."""

    def __init__(self, theme, parent=None):
        super().__init__(theme, parent, width=560)
        self.theme = theme
        self.header_widget = SheetHeader(theme, "Просмотр", "", "file",
                                         on_close=self.close)
        self.set_header(self.header_widget)
        self.view = QTextEdit()
        self.view.setReadOnly(True)
        self.view.setMinimumHeight(420)
        self.copy_btn = GlassButton(theme, "Скопировать", "copy", "secondary",
                                    compact=True)
        self.copy_btn.clicked.connect(self._copy)
        self.set_content([self.view, self.copy_btn])
        theme.changed.connect(self._restyle)
        self._restyle()

    def show_text(self, title: str, subtitle: str, body: str):
        self.header_widget.title_label.setText(title)
        self.header_widget.subtitle_label.setText(subtitle)
        self.view.setPlainText(body or "(пусто)")

    def _copy(self):
        from PySide6.QtWidgets import QApplication

        QApplication.clipboard().setText(self.view.toPlainText())

    def _restyle(self):
        pal = self.theme.palette
        self.view.setFont(font(self.theme.mono_family, pal.font_sm))
        self.view.setStyleSheet(
            f"QTextEdit{{background:{pal.surface_2};color:{pal.text};"
            f"border:1px solid {pal.line};border-radius:12px;padding:10px;}}")


# ---------------------------------------------------------------------------
# Стратегии: список, превью, применение в один клик
# ---------------------------------------------------------------------------

class StrategiesSheet(Sheet):
    """Все стратегии Flowseal: что есть, что внутри, какая активна."""

    def __init__(self, theme, controller, parent=None):
        super().__init__(theme, parent, width=560)
        self.theme = theme
        self.controller = controller
        self.set_header(SheetHeader(theme, "Стратегии",
                                    "список, превью и применение", "layers",
                                    on_close=self.close))
        self.search = SearchField(theme, "Найти стратегию…")
        self.search.textChanged.connect(lambda _t: self.refresh())
        self.list_box = QWidget()
        self.list_layout = QVBoxLayout(self.list_box)
        self.list_layout.setContentsMargins(0, 0, 0, 0)
        self.list_layout.setSpacing(6)
        self.rows: list[ServiceRow] = []
        self.preview = QTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setMinimumHeight(140)
        self.preview.setMaximumHeight(220)
        self.apply_btn = GlassButton(theme, "Применить выбранную", "zap", "primary",
                                     compact=True)
        self.apply_btn.clicked.connect(self._apply_selected)
        self.test_btn = GlassButton(theme, "Проверить", "flask", "secondary",
                                    compact=True)
        self.test_btn.clicked.connect(self._test_selected)
        self.refresh_btn = GlassButton(theme, "Обновить", "refresh", "ghost",
                                       compact=True)
        self.refresh_btn.clicked.connect(self.refresh)
        buttons = QWidget()
        buttons_layout = QHBoxLayout(buttons)
        buttons_layout.setContentsMargins(0, 0, 0, 0)
        buttons_layout.setSpacing(8)
        buttons_layout.addWidget(self.apply_btn, 1)
        buttons_layout.addWidget(self.test_btn, 1)
        buttons_layout.addWidget(self.refresh_btn, 1)
        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)
        self.set_content([
            section(theme, "Доступные", "клик — посмотреть содержимое", "layers",
                    [self.search, self.list_box]),
            self.preview,
            buttons,
            self.status_label,
        ])
        self._selected = ""
        controller.finished.connect(self._on_finished)
        theme.changed.connect(self._restyle)
        self._restyle()
        self.refresh()

    def _restyle(self):
        pal = self.theme.palette
        self.preview.setFont(font(self.theme.mono_family, pal.font_xs))
        self.preview.setStyleSheet(
            f"QTextEdit{{background:{pal.surface_2};color:{pal.muted};"
            f"border:1px solid {pal.line};border-radius:12px;padding:10px;}}")
        self.status_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.status_label.setStyleSheet(f"color:{pal.muted};background:transparent;")

    def refresh(self):
        for row in self.rows:
            self.list_layout.removeWidget(row)
            row.setParent(None)
        self.rows = []
        query = self.search.text().strip().lower()
        current = (self.controller.cfg.get("strategy") or "")
        items = self.controller.strategies()
        shown = 0
        for item in items:
            name = item.get("name", "")
            if query and query not in name.lower():
                continue
            detail = self._detail(item)
            row = ServiceRow(self.theme, name, name, "layers")
            row.set_status("ok" if name == current else "idle", detail)
            if name == current:
                row.set_badge("АКТИВНА")
            row.clicked.connect(self._on_row_clicked)
            self.list_layout.addWidget(row)
            self.rows.append(row)
            shown += 1
        if not self.rows:
            empty = QLabel("Ничего не найдено. Обновите зависимости — "
                           "стратегии скачиваются вместе с ними.")
            empty.setWordWrap(True)
            self.list_layout.addWidget(empty)
            self.rows.append(empty)  # type: ignore[arg-type]
        self.status_label.setText(f"Всего стратегий: {len(items)}" +
                                  (f" · показано {shown}" if query else ""))
        if self._selected:
            self._show_preview(self._selected)

    @staticmethod
    def _detail(item: dict) -> str:
        parts = []
        if item.get("tcp"):
            parts.append(f"TCP {item['tcp']}")
        if item.get("udp"):
            parts.append(f"UDP {item['udp']}")
        if item.get("filters"):
            parts.append(f"фильтров: {item['filters']}")
        size = item.get("size") or 0
        if size:
            parts.append(f"{size // 1024 + 1} КБ")
        return " · ".join(parts) or "—"

    def _on_row_clicked(self, name: str):
        if not isinstance(name, str) or not name.endswith(".bat"):
            return
        self._selected = name
        self._show_preview(name)
        for row in self.rows:
            if isinstance(row, ServiceRow):
                current = (self.controller.cfg.get("strategy") or "")
                row.set_badge("АКТИВНА" if row.key == current else "")

    def _show_preview(self, name: str):
        self.preview.setPlainText(self.controller.strategy_preview(name))

    def _apply_selected(self):
        if not self._selected:
            self.status_label.setText("Сначала выберите стратегию из списка.")
            return
        if self.controller.set_strategy(self._selected):
            self.refresh()

    def _test_selected(self):
        if not self._selected:
            self.status_label.setText("Сначала выберите стратегию из списка.")
            return
        self.controller.test_strategy(self._selected)

    def _on_finished(self, key: str, success: bool, _message: str):
        if key in ("apply", "strategy_test", "deps") and self.isVisible():
            self.refresh()


# ---------------------------------------------------------------------------
# Сеть и файрвол: интерфейс, бэкенд, версии зависимостей
# ---------------------------------------------------------------------------

class NetworkSheet(Sheet):
    """Сетевые настройки: интерфейс, файрвол, GameFilter, версии."""

    def __init__(self, theme, controller, parent=None):
        super().__init__(theme, parent)
        self.theme = theme
        self.controller = controller
        self.set_header(SheetHeader(theme, "Сеть и файрвол",
                                    "интерфейс, бэкенд и версии", "server",
                                    on_close=self.close))

        self.iface_combo = QComboBox()
        self.iface_combo.setMinimumHeight(38)
        self.backend_seg = SegmentedControl(
            theme, [("auto", "Авто"), ("nftables", "nftables"), ("iptables", "iptables")],
            "auto")
        self.nfqws_input = QLineEdit()
        self.nfqws_input.setPlaceholderText("latest или тег, например v1.7.2")
        self.nfqws_input.setMinimumHeight(38)
        self.rev_input = QLineEdit()
        self.rev_input.setPlaceholderText("пусто — рекомендованный коммит")
        self.rev_input.setMinimumHeight(38)
        self.game_tcp = Switch(theme, False)
        self.game_udp = Switch(theme, False)
        self.telegram_sw = Switch(theme, True)

        self.apply_btn = GlassButton(theme, "Применить и перезапустить", "zap",
                                     "primary")
        self.apply_btn.clicked.connect(self._apply)
        self.ping_btn = GlassButton(theme, "Пинг 1.1.1.1", "activity", "secondary",
                                    compact=True)
        self.ping_btn.clicked.connect(lambda: controller.run_ping("1.1.1.1"))
        self.speed_btn = GlassButton(theme, "Замер скорости", "gauge", "secondary",
                                     compact=True)
        self.speed_btn.clicked.connect(controller.run_speedtest)
        self.dns_btn = GlassButton(theme, "Проверка DNS", "globe", "ghost",
                                   compact=True)
        self.dns_btn.clicked.connect(self._dns_check)
        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)

        tools = QWidget()
        tools_layout = QHBoxLayout(tools)
        tools_layout.setContentsMargins(0, 0, 0, 0)
        tools_layout.setSpacing(8)
        tools_layout.addWidget(self.ping_btn, 1)
        tools_layout.addWidget(self.speed_btn, 1)
        tools_layout.addWidget(self.dns_btn, 1)

        self.set_content([
            section(theme, "Подключение", "куда применять правила", "wifi",
                    [SettingRow(theme, "Сетевой интерфейс", "any — все сразу",
                                self.iface_combo),
                     SettingRow(theme, "Файрвол", "бэкенд правил", self.backend_seg)]),
            section(theme, "Протоколы", "что обходить", "filter",
                    [SettingRow(theme, "Telegram", "MTProto, веб и звонки",
                                _switch_holder(theme, self.telegram_sw)),
                     SettingRow(theme, "GameFilter TCP", "порты игр",
                                _switch_holder(theme, self.game_tcp)),
                     SettingRow(theme, "GameFilter UDP", "порты игр",
                                _switch_holder(theme, self.game_udp))]),
            section(theme, "Версии", "что качать при обновлении", "download",
                    [SettingRow(theme, "nfqws", "latest — свежий релиз",
                                self.nfqws_input),
                     SettingRow(theme, "Коммит стратегий", "пусто — проверенный",
                                self.rev_input)]),
            self.apply_btn,
            tools,
            self.status_label,
        ])
        theme.changed.connect(self._restyle)
        self._restyle()
        self.reload()

    def _restyle(self):
        pal = self.theme.palette
        self.status_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.status_label.setStyleSheet(f"color:{pal.muted};background:transparent;")

    def reload(self):
        """Подтягивает значения из конфига в поля."""
        from .. import checks as _checks

        cfg = self.controller.cfg
        self.iface_combo.blockSignals(True)
        self.iface_combo.clear()
        ifaces = ["any"] + [i for i in _checks.list_interfaces()
                            if i != "lo"][:12]
        self.iface_combo.addItems(ifaces)
        current = cfg.get("interface", "any") or "any"
        self.iface_combo.setCurrentText(current if current in ifaces else "any")
        self.iface_combo.blockSignals(False)
        self.backend_seg.set_value(cfg.get("firewall_backend", "auto") or "auto")
        self.nfqws_input.setText(cfg.get("nfqws_version", "latest") or "latest")
        self.rev_input.setText(cfg.get("strategy_rev", "") or "")
        self.game_tcp.setChecked(bool(cfg.get("gamefilter_tcp")), animate_value=False)
        self.game_udp.setChecked(bool(cfg.get("gamefilter_udp")), animate_value=False)
        self.telegram_sw.setChecked(bool(cfg.get("telegram", True)), animate_value=False)

    def open(self):
        self.reload()
        super().open()

    def _apply(self):
        from .. import config as _config
        from .. import core as _core

        cfg = self.controller.cfg
        cfg["interface"] = self.iface_combo.currentText() or "any"
        cfg["firewall_backend"] = self.backend_seg.value or "auto"
        cfg["nfqws_version"] = self.nfqws_input.text().strip() or "latest"
        cfg["strategy_rev"] = self.rev_input.text().strip()
        cfg["gamefilter_tcp"] = self.game_tcp.isChecked()
        cfg["gamefilter_udp"] = self.game_udp.isChecked()
        cfg["telegram"] = self.telegram_sw.isChecked()
        _config.save(cfg)
        self.controller.log_now("Сетевые настройки сохранены.", "ok")
        if _core.nfqws_running():
            self.controller.restart_with_current()
        else:
            self.status_label.setText("Сохранено. Обход выключен — применится при включении.")
        self.controller.changed.emit()

    def _dns_check(self):
        from .. import checks as _checks

        result = _checks.dns_check()
        self.status_label.setText(f"DNS {result['host']}: {result['detail']}")
        self.controller.log_now(f"DNS {result['host']}: {result['detail']}.",
                                "ok" if result["ok"] else "warn")


# ---------------------------------------------------------------------------
# О программе: версия, ссылки, чейнджлог
# ---------------------------------------------------------------------------

class AboutSheet(Sheet):
    """О приложении: версия, чейнджлог, ссылки."""

    changelog_ready = Signal(list)

    def __init__(self, theme, controller, parent=None):
        super().__init__(theme, parent)
        self.theme = theme
        self.controller = controller
        self.set_header(SheetHeader(theme, "О программе",
                                    "версия, новости и ссылки", "info",
                                    on_close=self.close))
        self.title_label = QLabel("")
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.version_label = QLabel("")
        self.version_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.desc_label = QLabel(
            "Обход DPI-замедлений без VPN: YouTube, Discord, Telegram и другие "
            "сервисы открываются напрямую через nfqws и стратегии Flowseal.")
        self.desc_label.setWordWrap(True)
        self.desc_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.update_btn = GlassButton(theme, "Проверить обновления", "refresh",
                                      "accent-soft")
        self.update_btn.clicked.connect(lambda: controller.update_and_restart(True))
        self.changelog_btn = GlassButton(theme, "Что нового", "book", "secondary")
        self.changelog_btn.clicked.connect(self._load_changelog)
        self.github_btn = GlassButton(theme, "GitHub проекта", "github", "ghost")
        self.github_btn.clicked.connect(
            lambda: self._open_url("https://github.com/danilka-revin/Zapret"))
        self.copy_btn = GlassButton(theme, "Скопировать версию", "copy", "ghost",
                                    compact=True)
        self.copy_btn.clicked.connect(self._copy_version)

        self.changelog_label = QLabel("")
        self.changelog_label.setWordWrap(True)
        self.changelog_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)

        self.set_content([
            self.title_label,
            self.version_label,
            self.desc_label,
            Divider(theme),
            self.update_btn,
            self.changelog_btn,
            self.github_btn,
            self.copy_btn,
            self.changelog_label,
        ])
        self.changelog_ready.connect(self._fill_changelog)
        theme.changed.connect(self._restyle)
        self._restyle()

    def _restyle(self):
        from .. import APP_NAME, APP_VERSION

        pal = self.theme.palette
        self.title_label.setText(APP_NAME)
        self.title_label.setFont(font(self.theme.font_family, pal.font_xl,
                                      QFont.Weight.Bold))
        self.title_label.setStyleSheet(f"color:{pal.text};background:transparent;")
        self.version_label.setText(f"версия {APP_VERSION}")
        self.version_label.setFont(font(self.theme.font_family, pal.font_md))
        self.version_label.setStyleSheet(
            f"color:{pal.accent_text};background:transparent;")
        self.desc_label.setFont(font(self.theme.font_family, pal.font_sm))
        self.desc_label.setStyleSheet(f"color:{pal.muted};background:transparent;")
        self.changelog_label.setFont(font(self.theme.font_family, pal.font_sm))
        self.changelog_label.setStyleSheet(
            f"color:{pal.text};background:transparent;")

    def _copy_version(self):
        from PySide6.QtWidgets import QApplication

        from .. import APP_NAME, APP_VERSION

        QApplication.clipboard().setText(f"{APP_NAME} {APP_VERSION}")

    def _open_url(self, url: str):
        import shutil as _shutil
        import subprocess as _subprocess

        opener = _shutil.which("xdg-open")
        if opener:
            try:
                _subprocess.Popen([opener, url], stdout=_subprocess.DEVNULL,
                                  stderr=_subprocess.DEVNULL)
                return
            except OSError:
                pass
        self.controller.log_now(url, "info")

    def _load_changelog(self):
        import threading as _threading

        self.changelog_label.setText("Загружаю список изменений…")
        self.changelog_btn.setEnabled(False)

        def worker():
            from .. import update as _update

            try:
                entries = _update.fetch_changelog(limit=4)
            except Exception:  # noqa: BLE001
                entries = []
            self.changelog_ready.emit(entries)

        _threading.Thread(target=worker, daemon=True).start()

    def _fill_changelog(self, entries: list):
        self.changelog_btn.setEnabled(True)
        if not entries:
            self.changelog_label.setText("Не удалось загрузить (нет сети?).")
            return
        parts = []
        for entry in entries:
            body = (entry.get("body") or "").strip().replace("\r", "")
            lines = [line.strip() for line in body.splitlines() if line.strip()][:6]
            parts.append(f"● {entry.get('tag', '')} — {entry.get('name', '')}\n" +
                         "\n".join(f"  {line}" for line in lines))
        self.changelog_label.setText("\n\n".join(parts))
