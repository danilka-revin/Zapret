"""
Выезжающие панели Qt-интерфейса: оформление, журнал, диагностика и справка.

Кастомизация повторяет идею zmk-videoanalytics: тема, акцент, стекло, плотность,
углы, анимации и размер текста меняются на лету и сохраняются в config.json.
"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (QGridLayout, QHBoxLayout, QLabel, QLineEdit,
                               QVBoxLayout, QWidget)

from . import icons
from .theme import ACCENTS, PRESETS
from .widgets import (AccentPicker, Chip, ChoiceCard, GlassButton, LabeledSlider,
                      LogView, SectionTitle, SegmentedControl, ServiceRow, SettingRow,
                      Sheet, SheetHeader, Switch, font)


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
        icons_for = {"glass": "droplet", "lime": "zap", "night": "moon", "contrast": "chart"}
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
        ])

    # -- синхронизация -----------------------------------------------------

    def _apply_preset(self, key: str):
        self.theme.apply_preset(key)
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
        self.copy_btn = GlassButton(theme, "Скопировать", "copy", "secondary", compact=True)
        self.copy_btn.clicked.connect(self._copy)
        self.clear_btn = GlassButton(theme, "Очистить", "trash", "ghost", compact=True)
        self.clear_btn.clicked.connect(self._clear)

        self.view = LogView(theme)
        self.view.setMinimumHeight(420)
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(8)
        row_layout.addWidget(self.copy_btn)
        row_layout.addWidget(self.clear_btn)
        row_layout.addStretch(1)

        self.set_content([self.filter_seg, row, self.view])
        self.history: list[tuple[float, str, str]] = []

    def fill(self, history: list[tuple[float, str, str]]):
        self.history = list(history)
        self._rebuild()

    def append(self, message: str, kind: str = "info"):
        self.history.append((time.time(), message, kind))
        del self.history[:-2000]
        if self._visible_for(kind):
            self.view.append(message, kind)

    def _visible_for(self, kind: str) -> bool:
        mode = self.filter_seg.value
        if mode == "all":
            return True
        if mode == "ok":
            return kind in ("ok", "accent")
        return kind in ("error", "warn")

    def _rebuild(self):
        self.view.clear()
        for _ts, message, kind in self.history:
            if self._visible_for(kind):
                self.view.append(message, kind)

    def _copy(self):
        from PySide6.QtWidgets import QApplication

        text = "\n".join(f"[{time.strftime('%H:%M:%S', time.localtime(ts))}] {msg}"
                         for ts, msg, _k in self.history)
        QApplication.clipboard().setText(text)

    def _clear(self):
        self.history.clear()
        self.view.clear()


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
                           ("sudo", "Права без пароля"), ("backend", "Бэкенд файрвола"),
                           ("iface", "Сетевой интерфейс"), ("strategy", "Стратегия"),
                           ("appdir", "Каталог данных")):
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

        self.set_content([self.rows_box, buttons])
        theme.changed.connect(self._restyle)
        controller.status_ready.connect(lambda _s: self.refresh())
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
            "appdir": (status.get("app_dir", "—"), True),
        }
        for key, (text, good) in values.items():
            label = self.rows.get(key)
            if label is None:
                continue
            label.setText(text)
            color = pal.good if good else pal.warn
            if key in ("appdir", "iface", "strategy", "backend"):
                color = pal.text
            label.setStyleSheet(f"color:{color};background:transparent;")



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

class TargetSheet(Sheet):
    """
    «Подбор под сайт»: пользователь вводит домен (или выбирает группу), приложение
    перебирает все стратегии Flowseal и показывает, какая открывает именно этот сайт.
    """

    def __init__(self, theme, controller, parent=None):
        super().__init__(theme, parent, width=470)
        self.theme = theme
        self.controller = controller
        self.result_rows: list[ServiceRow] = []
        self.site_rows: list[ServiceRow] = []
        self._running = False

        self.set_header(SheetHeader(theme, "Подбор под сайт",
                                    "какая стратегия открывает именно его", "target",
                                    on_close=self.close))
        self._build()
        controller.targets_started.connect(self._on_started)
        controller.targets_done.connect(self._on_done)
        controller.finished.connect(self._on_finished)
        theme.changed.connect(self._restyle)

    # -- сборка ------------------------------------------------------------

    def _build(self):
        theme = self.theme

        self.input = QLineEdit()
        self.input.setPlaceholderText("rutracker.org, https://site.ru/page или несколько через запятую")
        self.input.setClearButtonEnabled(True)
        self.input.returnPressed.connect(self._run)
        self.input.setMinimumHeight(40)

        self.run_btn = GlassButton(theme, "Проверить стратегии", "rocket", "primary")
        self.run_btn.clicked.connect(self._run)

        self.apply_switch = Switch(theme, True)
        self.apply_switch.setToolTip("Сразу применять лучшую найденную стратегию")

        groups_label = QLabel("Готовые группы — один клик")
        self.groups_grid = QGridLayout()
        self.groups_grid.setSpacing(6)
        self._build_groups()

        self.history_grid = QGridLayout()
        self.history_grid.setSpacing(6)
        self.history_label = QLabel("Недавние запросы")

        self.status_label = QLabel("Введите сайт и нажмите «Проверить стратегии».")
        self.status_label.setWordWrap(True)

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

        self.set_content([
            section(theme, "Что проверяем", "домен или группа сайтов", "compass",
                    [self.input,
                     SettingRow(theme, "Применять лучшую", "сразу переключить обход на неё",
                                _switch_holder(theme, self.apply_switch)),
                     self.run_btn,
                     _label_holder(theme, groups_label, self.groups_grid),
                     _label_holder(theme, self.history_label, self.history_grid)]),
            self.status_label,
            self.results_title,
            self.results_box,
            self.sites_title,
            self.sites_box,
        ])
        self._rebuild_history()

    def _build_groups(self):
        from ..checks import SITE_GROUPS

        for index, (key, title, icon, hosts) in enumerate(SITE_GROUPS):
            chip = Chip(self.theme, title, icon)
            chip.setToolTip(", ".join(hosts[:3]) + ("…" if len(hosts) > 3 else ""))
            chip.clicked.connect(lambda _checked=False, k=key: self._run_group(k))
            self.groups_grid.addWidget(chip, index // 2, index % 2)

    def _rebuild_history(self):
        while self.history_grid.count():
            item = self.history_grid.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
        history = self.controller.target_history()
        self.history_label.setVisible(bool(history))
        for index, query in enumerate(history[:6]):
            chip = Chip(self.theme, query if len(query) <= 28 else query[:26] + "…", "clock")
            chip.setToolTip(query)
            chip.clicked.connect(lambda _checked=False, q=query: self._run_query(q))
            self.history_grid.addWidget(chip, index // 2, index % 2)

    # -- запуск ------------------------------------------------------------

    def _run_group(self, key: str):
        from ..checks import SITE_GROUPS_BY_KEY

        hosts = SITE_GROUPS_BY_KEY[key][2]
        self.input.setText(hosts[0])
        self.controller.log_now(f"Группа «{SITE_GROUPS_BY_KEY[key][0]}»: "
                                f"{len(hosts)} доменов", "info")
        self._run_query(", ".join(hosts))

    def _run(self):
        self._run_query(self.input.text())

    def _run_query(self, query: str):
        if self._running:
            self.controller.log_now("Подбор уже идёт — дождитесь результата.", "warn")
            return
        if not query.strip():
            self.status_label.setText("Сначала введите сайт, например rutracker.org")
            return
        self.controller.test_targets(query, apply_best=self.apply_switch.isChecked())

    # -- реакция контроллера -----------------------------------------------

    def _on_started(self, hosts: list):
        self._running = True
        self.run_btn.setEnabled(False)
        self.run_btn.setText("Подбираю…")
        self.status_label.setText(
            f"Проверяю {len(hosts)} домен(ов): {', '.join(hosts[:4])}"
            + ("…" if len(hosts) > 4 else ""))
        self._clear_rows(self.results_layout, self.result_rows)
        self.result_rows = []
        self._clear_rows(self.sites_layout, self.site_rows)
        self.site_rows = []
        self.results_title.setVisible(False)
        self.sites_title.setVisible(False)

    def _on_done(self, report: dict):
        self._running = False
        self.run_btn.setEnabled(True)
        self.run_btn.setText("Проверить стратегии")
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

    def _on_finished(self, key: str, success: bool, message: str):
        if key != "targets":
            return
        self._running = False
        self.run_btn.setEnabled(True)
        self.run_btn.setText("Проверить стратегии")
        if not success:
            self.status_label.setText(message or "Подбор не удался — смотрите журнал.")

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
        self.history_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.history_label.setStyleSheet(f"color:{pal.muted};background:transparent;")


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
