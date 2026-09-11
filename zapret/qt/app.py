"""
Главное окно Zapret Control на Qt6 (PySide6).

Философия интерфейса: минимум настроек — максимум автономики. На экране одна
большая кнопка, живая проверка сервисов, график трафика и пара переключателей,
а всё остальное (выбор стратегии, перезапуски, восстановление) приложение берёт
на себя. Оформление настраивается отдельной выезжающей панелью.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from PySide6.QtCore import QSize, Qt, QTimer
from PySide6.QtGui import QAction, QFont, QIcon, QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QFrame, QHBoxLayout,
                               QLabel, QMainWindow, QScrollArea, QSystemTrayIcon, QVBoxLayout, QWidget, QMenu)

from .. import APP_NAME, APP_VERSION, app_dir
from . import icons
from .controller import OPERATION_LABELS, Controller
from .sheets import (CustomizerSheet, DiagnosticsSheet, HelpSheet, JournalSheet,
                    TargetSheet)
from .theme import ThemeManager, apply_theme_to_app
from .widgets import (Backdrop, Card, GlassButton, IconButton, LogView, PowerSwitch,
                      ServiceRow, SettingRow, Sparkline, StatTile, StatusPill, Switch,
                      ToastHost, WheelScrollGuard, _Glass, font)

WINDOW_MIN = QSize(1040, 680)
WINDOW_DEFAULT = QSize(1200, 820)
HERO_HEIGHT = 322


def _human_speed(mbps: float) -> str:
    if mbps >= 100:
        return f"{mbps:.0f} Мбит/с"
    if mbps >= 10:
        return f"{mbps:.1f} Мбит/с"
    return f"{mbps:.2f} Мбит/с"


def _short_path(path: str) -> str:
    home = str(Path.home())
    if path.startswith(home):
        return "~" + path[len(home):]
    return path


def _human_bytes(value: int) -> str:
    size = float(value)
    for unit in ("Б", "КБ", "МБ", "ГБ", "ТБ"):
        if size < 1024 or unit == "ТБ":
            return f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} ТБ"


class ZapretWindow(QMainWindow):
    """Окно приложения: главный экран, панели, трей."""

    def __init__(self, theme: ThemeManager, controller: Controller, tray: bool = True):
        super().__init__()
        self.theme = theme
        self.controller = controller
        self.setWindowTitle(f"{APP_NAME} — обход DPI")
        self._fit_to_screen()

        self.log_history: list[tuple[float, str, str]] = []
        self._refresh_queued = False
        self._hide_notice_shown = False
        self._tray_notice_shown = False

        self._build_shell()
        self._build_header()
        self._build_hero()
        self._build_cards()
        self._build_log_card()
        self._build_sheets()

        self.toasts = ToastHost(theme, self)
        self._build_tray(tray)

        self._connect_controller()
        theme.changed.connect(self._restyle)
        self._restyle()
        self._apply_tray_visibility()

        QTimer.singleShot(80, self.controller.refresh_status)
        if os.environ.get("ZAPRET_NO_AUTOSTART") != "1":
            QTimer.singleShot(700, self.controller.bootstrap)
        if self.theme.settings.start_minimized and self.tray_icon is not None:
            QTimer.singleShot(1200, self._maybe_start_hidden)

    def _fit_to_screen(self):
        """Подгоняет окно под экран: на ноутбуках 1366×768 ничего не обрежется."""
        min_w, min_h = WINDOW_MIN.width(), WINDOW_MIN.height()
        width, height = WINDOW_DEFAULT.width(), WINDOW_DEFAULT.height()
        screen = QApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            width = min(width, max(880, avail.width() - 60))
            height = min(height, max(560, avail.height() - 80))
            min_w = min(min_w, max(860, avail.width() - 40))
            min_h = min(min_h, max(520, avail.height() - 60))
        self.setMinimumSize(min_w, min_h)
        self.resize(width, height)
        if screen is not None:
            avail = screen.availableGeometry()
            self.move(avail.x() + max(0, (avail.width() - width) // 2),
                      avail.y() + max(0, (avail.height() - height) // 3))

    # ------------------------------------------------------------------
    # Каркас окна
    # ------------------------------------------------------------------

    def _build_shell(self):
        # Фон и прокрутка живут ВНУТРИ central widget. Если сделать их соседями,
        # Qt ставит central widget поверх: пустой контейнер перекрывает всё окно
        # и съедает каждый клик и колесо мыши — выглядит как «ни одна кнопка не
        # нажимается и прокрутка не работает», хотя интерфейс живой.
        container = QWidget(self)
        container.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.setCentralWidget(container)
        shell = QVBoxLayout(container)      # прокрутка управляется layout'ом:
        shell.setContentsMargins(0, 0, 0, 0)  # размер не зависит от порядка событий
        shell.setSpacing(0)
        self._shell = shell

        self.bg = Backdrop(self.theme, container)
        _Glass.backdrop = self.bg

        self.scroll = QScrollArea(container)
        self.scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        # Прокрутку колесом обрабатывает WheelScrollGuard ниже: он двигает
        # verticalScrollBar напрямую. У QScrollArea нет setVerticalScrollMode()
        # (это метод QAbstractItemView) — такой вызов ронял окно на старте.
        self.scroll.viewport().setAutoFillBackground(False)
        self.scroll.setStyleSheet(
            "QScrollArea, QScrollArea > QWidget#qt_scrollarea_viewport,"
            "QScrollArea > QWidget > QWidget{background:transparent;border:none;}")
        self.scroll.viewport().setAutoFillBackground(False)
        self.scroll.viewport().setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)

        self.content = QWidget()
        self.content.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.content.setMinimumWidth(1000)
        self.root = QVBoxLayout(self.content)
        self.root.setContentsMargins(24, 20, 24, 24)
        self.root.setSpacing(16)
        self.scroll.setWidget(self.content)
        shell.addWidget(self.scroll)
        # колесо крутит окно, даже когда курсор над карточкой или кнопкой
        self._wheel_guard = WheelScrollGuard(self.scroll, self)
        self.scroll.viewport().installEventFilter(self._wheel_guard)

    def resizeEvent(self, event):  # noqa: N802
        rect = self.centralWidget().rect()
        self.bg.setGeometry(rect)          # фон за прокруткой, он вне layout
        if hasattr(self, "toasts"):
            self.toasts.setGeometry(rect)
            self.toasts._layout_toasts()
        for sheet in getattr(self, "sheets", {}).values():
            if sheet.isVisible():
                sheet.setGeometry(rect)
                sheet.overlay.setGeometry(rect)
                sheet._apply_offset(sheet.offset)
        super().resizeEvent(event)

    def showEvent(self, event):  # noqa: N802
        """Qt поднимает central widget при показе окна — возвращаем на место
        тосты и шторки, иначе они окажутся под содержимым."""
        super().showEvent(event)
        self._raise_overlays()
        if not getattr(self, "_input_checked", False):
            self._input_checked = True
            QTimer.singleShot(250, self._selfcheck_input)

    def _selfcheck_input(self):
        """Курсор должен попадать в контент окна, а не в пустой контейнер поверх него.

        Так интерфейс ловит ситуацию, которую не видно ни на скриншотах, ни в
        smoke-тесте (там клики шлются виджету напрямую): окно выглядит целым,
        а клики и колесо не работают ни на одной кнопке.
        """
        try:
            if any(sheet.isVisible() for sheet in getattr(self, "sheets", {}).values()):
                return
            center = self.rect().center()
            hit = self.childAt(center.x(), center.y())
            if hit is None or hit is self.scroll or self.scroll.isAncestorOf(hit):
                return
            self.controller.log_now(
                "Содержимое окна перекрыто виджетом " + hit.metaObject().className()
                + " — клики могут не работать. Обновите приложение: «Обновить и перезапустить» "
                "или bash install.sh update.", "warn")
        except Exception:  # noqa: BLE001 — самопроверка не должна ронять интерфейс
            pass

    def _raise_overlays(self):
        """Порядок важен: тосты — самое верхнее, поверх шторок и содержимого."""
        for widget in [*getattr(self, "sheets", {}).values(), getattr(self, "toasts", None)]:
            if widget is not None:
                widget.raise_()

    # ------------------------------------------------------------------
    # Шапка
    # ------------------------------------------------------------------

    def _build_header(self):
        header = QWidget()
        layout = QHBoxLayout(header)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(14)
        header.setObjectName("header")

        self.mark = QLabel(header)
        self.mark.setFixedSize(42, 42)
        layout.addWidget(self.mark)

        text_box = QVBoxLayout()
        text_box.setSpacing(2)
        self.title_label = QLabel(APP_NAME, header)
        self.subtitle_label = QLabel("", header)
        text_box.addWidget(self.title_label)
        text_box.addWidget(self.subtitle_label)
        layout.addLayout(text_box)
        layout.addStretch(1)

        self.pill = StatusPill(self.theme, "Проверяю…", "idle")
        layout.addWidget(self.pill)

        self.theme_toggle_btn = IconButton(self.theme, "sun", "Переключить тему "
                                           "(тёмная / светлая)", 40)
        self.theme_toggle_btn.clicked.connect(self._toggle_theme)
        self.autopilot_btn = IconButton(self.theme, "sparkles", "Автопилот: включён", 40)
        self.autopilot_btn.clicked.connect(self._toggle_autopilot)
        self.target_btn = IconButton(self.theme, "target",
                                     "Подбор стратегии под сайт (Ctrl+T)", 40)
        self.target_btn.clicked.connect(lambda: self._toggle_sheet("targets"))
        self.palette_btn = IconButton(self.theme, "palette", "Оформление (Ctrl+,)", 40)
        self.palette_btn.clicked.connect(lambda: self._toggle_sheet("customizer"))
        self.journal_btn = IconButton(self.theme, "terminal", "Журнал (Ctrl+L)", 40)
        self.journal_btn.clicked.connect(lambda: self._toggle_sheet("journal"))
        self.help_btn = IconButton(self.theme, "help", "Справка", 40)
        self.help_btn.clicked.connect(lambda: self._toggle_sheet("help"))
        for btn in (self.theme_toggle_btn, self.autopilot_btn, self.target_btn,
                    self.palette_btn, self.journal_btn, self.help_btn):
            layout.addWidget(btn)

        self.header_card = Card(self.theme)
        self.header_card.body.addWidget(header)
        self.root.addWidget(self.header_card)

    # ------------------------------------------------------------------
    # Главный блок с кнопкой
    # ------------------------------------------------------------------

    def _build_hero(self):
        self.hero = Card(self.theme)
        self.hero.accent_edge = True
        self.hero.setMinimumHeight(HERO_HEIGHT)
        row = QHBoxLayout()
        row.setContentsMargins(6, 0, 6, 0)
        row.setSpacing(22)

        self.power = PowerSwitch(self.theme, 250)
        self.power.clicked.connect(self.controller.toggle_power)
        row.addWidget(self.power, 0, Qt.AlignmentFlag.AlignVCenter)

        middle = QVBoxLayout()
        middle.setSpacing(10)
        self.hero_title = QLabel("Защита выключена")
        self.hero_subtitle = QLabel("Нажмите большую кнопку — приложение всё настроит само")
        self.hero_subtitle.setWordWrap(True)
        middle.addWidget(self.hero_title)
        middle.addWidget(self.hero_subtitle)

        tiles = QHBoxLayout()
        tiles.setSpacing(10)
        self.tile_latency = StatTile(self.theme, "Задержка", "—", "activity")
        self.tile_services = StatTile(self.theme, "Сервисы", "—", "check-circle")
        self.tile_uptime = StatTile(self.theme, "Работает", "—", "clock")
        self.tile_mode = StatTile(self.theme, "Режим", "Автопилот", "rocket")
        for tile in (self.tile_latency, self.tile_services, self.tile_uptime, self.tile_mode):
            tile.setMinimumHeight(64)
            tiles.addWidget(tile, 1)
        middle.addLayout(tiles)

        self.hero_hint = QLabel("")
        self.hero_hint.setWordWrap(True)
        middle.addWidget(self.hero_hint)

        hero_buttons = QHBoxLayout()
        hero_buttons.setSpacing(10)
        self.autopilot_quick = GlassButton(self.theme, "Перебрать стратегии", "refresh",
                                           "accent-soft")
        self.autopilot_quick.clicked.connect(self.controller.run_autopilot)
        self.btn_target_test = GlassButton(self.theme, "Подбор под сайт", "target",
                                           "secondary")
        self.btn_target_test.clicked.connect(lambda: self._toggle_sheet("targets"))
        self.btn_setup_rights = GlassButton(self.theme, "Настроить права (1 раз)", "key",
                                            "primary")
        self.btn_setup_rights.clicked.connect(
            lambda: self.controller.setup_permissions(self._open_terminal))
        hero_buttons.addWidget(self.autopilot_quick)
        hero_buttons.addWidget(self.btn_target_test)
        hero_buttons.addWidget(self.btn_setup_rights)
        hero_buttons.addStretch(1)
        middle.addLayout(hero_buttons)
        row.addLayout(middle, 1)

        side = QVBoxLayout()
        side.setSpacing(12)
        side.addWidget(SelfLabel(self.theme, "Автономика"))
        self.sw_autopilot = Switch(self.theme, self.theme.settings.autopilot)
        self.sw_autopilot.toggled.connect(self._on_autopilot_switch)
        self.sw_telegram = Switch(self.theme, bool(self.controller.cfg.get("telegram", True)))
        self.sw_telegram.toggled.connect(
            lambda v: self.controller.set_config_value("telegram", v))
        self.sw_gamefilter = Switch(self.theme,
                                    bool(self.controller.cfg.get("gamefilter_tcp", False)))
        self.sw_gamefilter.toggled.connect(self._on_gamefilter)

        side.addWidget(SettingRow(self.theme, "Автопилот", "сам подбирает стратегию",
                                  _wrap(self.sw_autopilot)))
        side.addWidget(SettingRow(self.theme, "Telegram", "обход MTProto и веб-версии",
                                  _wrap(self.sw_telegram)))
        side.addWidget(SettingRow(self.theme, "GameFilter", "порты игр TCP и UDP",
                                  _wrap(self.sw_gamefilter)))
        side.addStretch(1)
        side_panel = QWidget()
        side_panel.setLayout(side)
        side_panel.setFixedWidth(300)
        row.addWidget(side_panel, 0)

        self.hero.body.addLayout(row)
        self.root.addWidget(self.hero)

    # ------------------------------------------------------------------
    # Карточки: сервисы, трафик, обслуживание
    # ------------------------------------------------------------------

    def _build_cards(self):
        row1 = QHBoxLayout()
        row1.setSpacing(16)

        self.services_card = Card(self.theme, "Проверка сервисов",
                                  "состояние обновляется автоматически", "shield-check")
        self.service_rows: dict[str, ServiceRow] = {}
        from ..checks import SERVICES

        for service in SERVICES:
            row = ServiceRow(self.theme, service.key, service.title, service.icon)
            row.clicked.connect(lambda _key: self.controller.check_services())
            self.service_rows[service.key] = row
            self.services_card.body.addWidget(row)
        footer = QWidget()
        footer_layout = QHBoxLayout(footer)
        footer_layout.setContentsMargins(0, 0, 0, 0)
        self.services_footer = QLabel("проверок пока не было")
        self.recheck_btn = GlassButton(self.theme, "Проверить", "target", "ghost", compact=True)
        self.recheck_btn.clicked.connect(lambda: self.controller.check_services())
        footer_layout.addWidget(self.services_footer, 1)
        footer_layout.addWidget(self.recheck_btn)
        self.services_card.body.addWidget(footer)
        row1.addWidget(self.services_card, 3)

        self.traffic_card = Card(self.theme, "Трафик", "живая скорость соединения", "activity")
        self.spark = Sparkline(self.theme)
        self.spark.setMinimumHeight(96)
        self.traffic_card.body.addWidget(self.spark)
        stats = QHBoxLayout()
        stats.setSpacing(10)
        self.tile_rx = StatTile(self.theme, "Приём", "0 Мбит/с", "download")
        self.tile_tx = StatTile(self.theme, "Отдача", "0 Мбит/с", "upload-cloud")
        self.tile_total = StatTile(self.theme, "Всего", "0 Б", "cloud")
        for tile in (self.tile_rx, self.tile_tx, self.tile_total):
            stats.addWidget(tile, 1)
        self.traffic_card.body.addLayout(stats)
        self.iface_label = QLabel("интерфейс: —")
        self.traffic_card.body.addWidget(self.iface_label)
        row1.addWidget(self.traffic_card, 2)
        self.root.addLayout(row1)

        row2 = QHBoxLayout()
        row2.setSpacing(16)

        self.maintenance_card = Card(self.theme, "Обслуживание",
                                     "редкие действия — обычно не нужны", "sliders")
        # Главная кнопка обновлений: сама тянет код, зависимости, ярлык и права,
        # а затем перезапускает интерфейс — «установил и забыл».
        self.btn_update = GlassButton(self.theme, "Обновить и перезапустить", "refresh",
                                      "accent-soft")
        self.btn_update.clicked.connect(lambda: self.controller.update_and_restart(True))
        self.btn_check_update = GlassButton(self.theme, "Проверить обновления", "search",
                                            "ghost", compact=True)
        self.btn_check_update.clicked.connect(lambda: self.controller.check_update())
        self.update_state_label = QLabel("")
        self.update_state_label.setWordWrap(True)
        self.maintenance_card.body.addWidget(self.btn_update)
        self.maintenance_card.body.addWidget(self.update_state_label)
        self.btn_deps = GlassButton(self.theme, "Обновить зависимости", "download", "secondary")
        self.btn_deps.clicked.connect(self.controller.download_deps)
        self.btn_autostart = GlassButton(self.theme, "Включить автозапуск", "power", "secondary")
        self.btn_autostart.clicked.connect(self.controller.toggle_autostart)
        self.btn_shortcut = GlassButton(self.theme, "Создать ярлык (меню + рабочий стол)",
                                        "external", "ghost")
        self.btn_shortcut.clicked.connect(self.controller.toggle_shortcut)
        self.btn_repair = GlassButton(self.theme, "Починить установку", "settings", "ghost")
        self.btn_repair.setToolTip("Перенести всё из /root, обновить ярлык на столе "
                                   "и права NOPASSWD")
        self.btn_repair.clicked.connect(self.controller.repair_install)
        self.btn_permissions = GlassButton(self.theme, "Права без пароля", "key", "ghost")
        self.btn_permissions.clicked.connect(
            lambda: self.controller.setup_permissions(self._open_terminal))
        self.btn_diagnostics = GlassButton(self.theme, "Диагностика", "flask", "ghost")
        self.btn_diagnostics.clicked.connect(lambda: self._toggle_sheet("diagnostics"))
        for btn in (self.btn_deps, self.btn_autostart, self.btn_shortcut, self.btn_repair,
                    self.btn_permissions):
            self.maintenance_card.body.addWidget(btn)
        footer = QWidget()
        footer_layout = QHBoxLayout(footer)
        footer_layout.setContentsMargins(0, 0, 0, 0)
        footer_layout.setSpacing(8)
        footer_layout.addWidget(self.btn_check_update, 1)
        footer_layout.addWidget(self.btn_diagnostics, 1)
        self.maintenance_card.body.addWidget(footer)
        self.maintenance_card.body.addStretch(1)
        row2.addWidget(self.maintenance_card, 2)

        self.extra_card = Card(self.theme, "Автономность",
                               "что приложение делает само", "rocket")
        self.extra_rows: dict[str, Switch] = {}
        options = [
            ("autostart_protection", "Включать защиту при запуске", "старт вместе с системой"),
            ("auto_recover", "Автовосстановление", "поднимает обход, если он упал"),
            ("tray", "Иконка в трее", "управление без открытия окна"),
            ("notifications", "Уведомления", "сообщения о смене состояния"),
            ("orbs", "Живой фон", "цветные пятна под стеклом"),
        ]
        for key, title, hint in options:
            switch = Switch(self.theme, bool(getattr(self.theme.settings, key)))
            switch.toggled.connect(lambda value, k=key: self.theme.update(**{k: value}))
            self.extra_rows[key] = switch
            self.extra_card.body.addWidget(SettingRow(self.theme, title, hint, _wrap(switch)))
        self.extra_card.body.addStretch(1)
        row2.addWidget(self.extra_card, 3)
        self.root.addLayout(row2)

    def _build_log_card(self):
        self.log_card = Card(self.theme, "Журнал", "все операции приложения", "terminal")
        self.log_view = LogView(self.theme)
        self.log_view.setMinimumHeight(150)
        self.log_view.setMaximumHeight(220)
        self.log_card.body.addWidget(self.log_view)
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        self.btn_log_journal = GlassButton(self.theme, "Подробный журнал", "list", "ghost",
                                           compact=True)
        self.btn_log_journal.clicked.connect(lambda: self._toggle_sheet("journal"))
        self.btn_log_copy = GlassButton(self.theme, "Скопировать", "copy", "ghost", compact=True)
        self.btn_log_copy.clicked.connect(self._copy_log)
        self.btn_log_clear = GlassButton(self.theme, "Очистить", "trash", "ghost", compact=True)
        self.btn_log_clear.clicked.connect(self.log_view.clear)
        layout.addWidget(self.btn_log_journal)
        layout.addStretch(1)
        layout.addWidget(self.btn_log_copy)
        layout.addWidget(self.btn_log_clear)
        self.log_card.body.addWidget(row)
        self.root.addWidget(self.log_card)

    def _build_sheets(self):
        self.sheets = {
            "customizer": CustomizerSheet(self.theme, self),
            "journal": JournalSheet(self.theme, self),
            "targets": TargetSheet(self.theme, self.controller, self),
            "diagnostics": DiagnosticsSheet(self.theme, self.controller, self),
            "help": HelpSheet(self.theme, self.controller, self),
        }
        self.sheets["help"].set_actions(
            lambda: self.controller.setup_permissions(self._open_terminal),
            self._open_readme,
        )
        self.sheets["journal"].closed.connect(self._on_sheet_closed)
        for sheet in self.sheets.values():
            sheet.hide()

    # ------------------------------------------------------------------
    # Трей
    # ------------------------------------------------------------------

    def _build_tray(self, enabled: bool):
        self.tray_icon: QSystemTrayIcon | None = None
        if not enabled or not QSystemTrayIcon.isSystemTrayAvailable():
            return
        self.tray_icon = QSystemTrayIcon(QIcon(icons.app_icon(64, self.theme.palette.accent,
                                                              self.theme.palette.dark)), self)
        self.tray_icon.setToolTip(f"{APP_NAME} — защита выключена")
        menu = QMenu()
        self.act_toggle = QAction("Включить защиту", menu)
        self.act_toggle.triggered.connect(self._tray_toggle)
        self.act_show = QAction("Показать окно", menu)
        self.act_show.triggered.connect(self._show_window)
        self.act_check = QAction("Проверить сервисы", menu)
        self.act_check.triggered.connect(lambda: self.controller.check_services())
        self.act_autostart = QAction("Автозапуск системы", menu)
        self.act_autostart.setCheckable(True)
        self.act_autostart.triggered.connect(self.controller.toggle_autostart)
        self.act_quit = QAction("Выход", menu)
        self.act_quit.triggered.connect(self._quit_app)
        for action in (self.act_toggle, self.act_show, self.act_check, self.act_autostart):
            menu.addAction(action)
        menu.addSeparator()
        menu.addAction(self.act_quit)
        self.tray_menu = menu
        self.tray_icon.setContextMenu(menu)
        self.tray_icon.activated.connect(self._tray_activated)
        self.tray_icon.show()

    def _tray_activated(self, reason):
        if reason in (QSystemTrayIcon.ActivationReason.Trigger,
                      QSystemTrayIcon.ActivationReason.DoubleClick):
            self._toggle_window()

    def _tray_toggle(self):
        self.controller.toggle_power()

    def _toggle_window(self):
        if self.isVisible() and not self.isMinimized():
            self.hide()
        else:
            self._show_window()

    def _show_window(self):
        self.showNormal()
        self.raise_()
        self.activateWindow()

    def _apply_tray_visibility(self):
        want = self.theme.settings.tray and self.tray_icon is not None
        if self.tray_icon is not None:
            self.tray_icon.setVisible(want)

    def _maybe_start_hidden(self):
        if self.theme.settings.start_minimized and self.tray_icon is not None:
            self.hide()
            if self.theme.settings.notifications:
                self.tray_icon.showMessage(
                    APP_NAME, "Приложение работает в трее и следит за защитой.",
                    QIcon(icons.app_icon(64, self.theme.palette.accent, True)), 4000)

    def _quit_app(self):
        if self.tray_icon is not None:
            self.tray_icon.hide()
        if self.controller.status.get("running"):
            # Обход живёт в отдельном процессе (nfqws + правила файрвола), поэтому
            # выход из приложения его не выключает.
            self.controller.log_now("Выход из приложения. Защита остаётся включённой.",
                                    "warn")
        else:
            self.controller.log_now("Выход из приложения.", "info")
        QApplication.quit()

    def closeEvent(self, event):  # noqa: N802
        if (self.tray_icon is not None and self.theme.settings.tray
                and self.tray_icon.isVisible()):
            event.ignore()
            self.hide()
            if not self._hide_notice_shown and self.theme.settings.notifications:
                self._hide_notice_shown = True
                self.tray_icon.showMessage(
                    APP_NAME, "Окно скрыто. Приложение продолжает работать в трее.",
                    QIcon(icons.app_icon(64, self.theme.palette.accent, True)), 4000)
            return
        event.accept()
        self._quit_app()

    # ------------------------------------------------------------------
    # Сигналы контроллера
    # ------------------------------------------------------------------

    def _connect_controller(self):
        c = self.controller
        c.log.connect(self._on_log)
        c.status_ready.connect(self._on_status)
        c.services_ready.connect(self._on_services)
        c.services_checking.connect(self._on_checking)
        c.metrics_ready.connect(self._on_metrics)
        c.busy_changed.connect(self._on_busy)
        c.changed.connect(self._queue_refresh)
        c.finished.connect(self._on_finished)
        c.notify.connect(self._on_notify)
        c.update_info_ready.connect(self._on_update_info)
        c.restart_requested.connect(self._on_restart_requested)

        QShortcut(QKeySequence("Ctrl+P"), self, activated=self.controller.toggle_power)
        QShortcut(QKeySequence("Ctrl+R"), self,
                  activated=lambda: self.controller.check_services())
        QShortcut(QKeySequence("Ctrl+,"), self, activated=lambda: self._toggle_sheet("customizer"))
        QShortcut(QKeySequence("Ctrl+L"), self, activated=lambda: self._toggle_sheet("journal"))
        QShortcut(QKeySequence("Ctrl+D"), self, activated=lambda: self._toggle_sheet("diagnostics"))
        QShortcut(QKeySequence("Ctrl+T"), self, activated=lambda: self._toggle_sheet("targets"))
        QShortcut(QKeySequence("Ctrl+Q"), self, activated=self._quit_app)
        QShortcut(QKeySequence("Escape"), self, activated=self._close_sheets)

    def _on_log(self, message: str, kind: str = "info"):
        self.log_history.append((time.time(), message, kind))
        del self.log_history[:-1200]
        self.log_view.append(message, kind)
        journal = self.sheets.get("journal")
        if journal is not None and journal.isVisible():
            journal.append(message, kind)

    def _on_status(self, status: dict):
        self._queue_refresh()

    def _on_services(self, results: list):
        for result in results:
            row = self.service_rows.get(result["key"])
            if row is not None:
                row.set_status(result["state"], result["detail"])
        self._queue_refresh()

    def _on_checking(self, keys: list):
        for key in keys:
            row = self.service_rows.get(key)
            if row is not None and row.state == "idle":
                row.set_status("checking", "проверяю…")

    def _on_metrics(self, metrics: dict):
        self.spark.push(metrics.get("rx_mbps", 0.0))
        self.tile_rx.set_value(_human_speed(metrics.get("rx_mbps", 0.0)))
        self.tile_tx.set_value(_human_speed(metrics.get("tx_mbps", 0.0)))
        self.tile_total.set_value(_human_bytes(metrics.get("total_rx", 0)
                                               + metrics.get("total_tx", 0)))
        iface = metrics.get("iface") or "—"
        self.iface_label.setText(f"интерфейс: {iface}")

    def _on_busy(self, key: str):
        self._queue_refresh()
        if key:
            self.power.set_state("busy", "ЖДИТЕ", OPERATION_LABELS.get(key, "Выполняю…"))
            self.power.set_progress(0.0)
        self._set_buttons_enabled(not key)

    def _on_finished(self, key: str, success: bool, message: str):
        if success:
            if key in ("power_on", "bootstrap", "autopilot"):
                self.toasts.show_toast("Готово: " + OPERATION_LABELS.get(key, "операция"),
                                       "ok")
            elif key in ("deps", "update"):
                self.toasts.show_toast("Готово: " + OPERATION_LABELS.get(key, "операция"),
                                       "ok")
            elif key == "repair":
                self.toasts.show_toast("Установка починена", "ok")
        else:
            self.toasts.show_toast(f"{OPERATION_LABELS.get(key, 'Операция')}: {message}",
                                   "error", 6000)
        if key in ("update", "repair"):
            self.controller.check_update(notify=False)
        self._queue_refresh()

    def _on_update_info(self, info: dict) -> None:
        """Результат проверки обновлений: подпись под кнопкой и её название."""
        self._queue_refresh()
        if info.get("available"):
            self.toasts.show_toast("Доступна новая версия — нажмите «Обновить и перезапустить»",
                                   "info", 5000)

    def _on_restart_requested(self, message: str) -> None:
        """Мягко закрываем окно: новое поднимет «ждущий» процесс после обновления."""
        if message:
            self.toasts.show_toast(message, "accent", 2500)
        self.controller.log_now(message or "Перезапуск…", "accent")
        QTimer.singleShot(600, self._quit_app)

    def _update_state_text(self) -> str:
        """Строка под кнопкой обновления: версия и что с ней."""
        info = self.controller.update_info or {}
        state = info.get("available")
        version = APP_VERSION
        if state is True:
            return (f"v{version} · доступно обновление "
                    + str(info.get("remote", ""))[:8] + " — нажмите кнопку выше")
        if state is False:
            return f"v{version} · обновление не требуется"
        if info.get("message"):
            return f"v{version} · " + str(info["message"])
        return f"v{version} · проверяю обновления…"

    def _on_notify(self, title: str, message: str):
        if not self.theme.settings.notifications:
            return
        if self.tray_icon is not None and self.tray_icon.isVisible():
            self.tray_icon.showMessage(title, message,
                                       QIcon(icons.app_icon(64, self.theme.palette.accent,
                                                            self.theme.palette.dark)), 3500)
        elif self.isVisible():
            self.toasts.show_toast(message, "info")

    # ------------------------------------------------------------------
    # Обновление вида
    # ------------------------------------------------------------------

    def _queue_refresh(self):
        if self._refresh_queued:
            return
        self._refresh_queued = True
        QTimer.singleShot(160, self._refresh_dashboard)

    def _refresh_dashboard(self):
        self._refresh_queued = False
        pal = self.theme.palette
        c = self.controller
        status = c.status or {}
        state = c.power_state()
        running = bool(status.get("running"))
        firewall = bool(status.get("firewall"))

        # Большая кнопка
        if state == "busy":
            key = c.busy_key or ""
            detail = c.busy_detail or OPERATION_LABELS.get(key, "Выполняю…")
            if c.progress:
                # Автопилот: показываем, сколько стратегий уже проверено
                self.power.set_state("busy", f"ПОДБОР {int(c.progress * 100)}%", detail)
                self.power.set_progress(c.progress)
            else:
                self.power.set_state("busy", "ЖДИТЕ", detail)
                self.power.set_progress(0.0)
        elif state == "on":
            self.power.set_state("on", "ВКЛ", "Обход активен · нажмите, чтобы выключить")
            self.power.set_progress(1.0)
        elif state == "error":
            self.power.set_state("error", "ОШИБКА",
                                 c.error_message or "Что-то пошло не так — откройте журнал")
            self.power.set_progress(0.0)
        else:
            self.power.set_state("off", "ВЫКЛ", "Нажмите, чтобы включить обход")
            self.power.set_progress(0.0)

        # Заголовок и подпись
        avg = c.average_latency()
        if state == "busy":
            self.hero_title.setText(OPERATION_LABELS.get(c.busy_key, "Выполняю…") + "…")
            self.hero_subtitle.setText("Приложение работает автоматически, ничего нажимать "
                                       "не нужно.")
        elif state == "on":
            self.hero_title.setText("Защита включена")
            details = []
            if c.services:
                details.append(f"сервисов в порядке: {c.ok_services_text()}")
            if avg:
                details.append(f"средняя задержка {avg} мс")
            strategy = c.cfg.get("strategy", "")
            if strategy:
                details.append(f"стратегия {strategy}")
            self.hero_subtitle.setText(" · ".join(details) or "Обход DPI активен")
        elif state == "error":
            self.hero_title.setText("Нужно внимание")
            self.hero_subtitle.setText(c.error_message or
                                       "Проверьте журнал и права без пароля.")
        else:
            self.hero_title.setText("Защита выключена")
            self.hero_subtitle.setText("Нажмите большую кнопку — автопилот подберёт рабочую "
                                       "стратегию и включит обход.")

        # Кнопка настройки прав показывается, только когда она действительно нужна
        needs_rights = not status.get("sudo_ok")
        self.btn_setup_rights.setVisible(needs_rights)

        hint_parts = []
        if state == "error" and c.hint:
            hint_parts.append(c.hint)
        if state == "on":
            if not firewall:
                hint_parts.append("Правила файрвола не найдены — включите защиту заново.")
            if not status.get("deps_ready"):
                hint_parts.append("Зависимости не скачаны.")
            if not status.get("sudo_ok"):
                hint_parts.append("Права без пароля не настроены: используйте кнопку "
                                  "«Права без пароля».")
            if not c.services:
                hint_parts.append("Проверка сервисов ещё не проводилась.")
        self.hero_hint.setText(" ".join(hint_parts))

        # Плитки
        self.tile_latency.set_value(f"{avg} мс" if avg else "—", accent=bool(avg and avg < 400))
        self.tile_services.set_value(c.ok_services_text(),
                                     accent=bool(c.services and all(
                                         r["state"] == "ok" for r in c.services)))
        self.tile_uptime.set_value(c.uptime_text())
        self.tile_mode.set_value("Автопилот" if self.theme.settings.autopilot else "Вручную",
                                 accent=self.theme.settings.autopilot)

        # Пилюля статуса
        if state == "busy":
            self.pill.set_status(OPERATION_LABELS.get(c.busy_key, "Работаю…"), "accent")
        elif running and firewall:
            self.pill.set_status("Защита включена", "ok")
        elif running:
            self.pill.set_status("Обход без файрвола", "warn")
        else:
            self.pill.set_status("Защита выключена", "idle")

        # Шапка
        mode_text = "автопилот" if self.theme.settings.autopilot else "ручной режим"
        self.subtitle_label.setText(
            f"v{APP_VERSION} · {mode_text} · {c.last_check_text()}")
        self.theme_toggle_btn.set_icon("sun" if pal.dark else "moon")
        self.theme_toggle_btn.setToolTip("Переключить на светлую тему" if pal.dark
                                         else "Переключить на тёмную тему")
        self.autopilot_btn.kind = "solid" if self.theme.settings.autopilot else "ghost"
        self.autopilot_btn.setToolTip("Автопилот: " +
                                      ("включён" if self.theme.settings.autopilot else "выключен"))
        self.autopilot_btn.update()

        # Кнопки обслуживания
        if status.get("service_installed"):
            self.btn_autostart.setText("Отключить автозапуск")
            self.btn_autostart.set_kind("danger")
        else:
            self.btn_autostart.setText("Включить автозапуск")
            self.btn_autostart.set_kind("secondary")
        self.btn_deps.setText("Обновить зависимости" if status.get("deps_ready")
                              else "Скачать зависимости")
        self.btn_deps.set_kind("secondary" if status.get("deps_ready") else "accent-soft")
        self.btn_permissions.set_kind("ghost" if status.get("sudo_ok") else "accent-soft")
        self.btn_shortcut.setText("Создать ярлык (меню + рабочий стол)")

        # Кнопка обновления: подсвечивается, когда есть новая версия
        available = (self.controller.update_info or {}).get("available")
        busy = bool(c.busy_key)
        if busy and c.busy_key == "update":
            self.btn_update.setText("Обновляю и перезапускаю…")
            self.btn_update.set_kind("primary")
        elif available:
            self.btn_update.setText("Обновить и перезапустить")
            self.btn_update.set_kind("primary")
        else:
            self.btn_update.setText("Обновить и перезапустить")
            self.btn_update.set_kind("accent-soft" if available is None else "secondary")
        self.update_state_label.setText(self._update_state_text())

        self.services_footer.setText(c.last_check_text() +
                                     (f" · {c.average_latency()} мс" if avg else ""))

        # Трей
        if self.tray_icon is not None:
            self.tray_icon.setToolTip(f"{APP_NAME} — " +
                                      ("защита включена" if state == "on" else "защита выключена"))
            self.act_toggle.setText("Выключить защиту" if running else "Включить защиту")
            self.act_autostart.setChecked(bool(status.get("service_installed")))
            self._apply_tray_visibility()
        self.sw_autopilot.setChecked(self.theme.settings.autopilot, animate_value=False)

    # ------------------------------------------------------------------
    # Стили
    # ------------------------------------------------------------------

    def _restyle(self):
        pal = self.theme.palette
        self.mark.setPixmap(icons.app_icon(42, pal.accent, pal.dark))
        self.title_label.setFont(font(self.theme.font_family, pal.font_xl, QFont.Weight.Bold))
        self.title_label.setStyleSheet(f"color:{pal.text};background:transparent;")
        self.subtitle_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.subtitle_label.setStyleSheet(f"color:{pal.muted};background:transparent;")
        self.hero_title.setFont(font(self.theme.font_family, max(20.0, pal.font_hero * 0.62),
                                     QFont.Weight.Bold))
        self.hero_title.setStyleSheet(f"color:{pal.text};background:transparent;")
        self.hero_subtitle.setFont(font(self.theme.font_family, pal.font_md))
        self.hero_subtitle.setStyleSheet(f"color:{pal.muted};background:transparent;")
        self.hero_hint.setFont(font(self.theme.font_family, pal.font_xs))
        self.hero_hint.setStyleSheet(f"color:{pal.warn};background:transparent;")
        self.services_footer.setFont(font(self.theme.font_family, pal.font_xs))
        self.services_footer.setStyleSheet(f"color:{pal.muted};background:transparent;")
        self.iface_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.iface_label.setStyleSheet(f"color:{pal.muted};background:transparent;")
        available = (self.controller.update_info or {}).get("available")
        self.update_state_label.setFont(font(self.theme.font_family, pal.font_xs))
        self.update_state_label.setStyleSheet(
            f"color:{pal.accent if available else pal.muted};background:transparent;")
        if not hasattr(self, "_stat_labels"):
            self._stat_labels = [w for w in self.findChildren(QLabel)]
        for label in self.findChildren(QLabel):
            if label.styleSheet() == "":
                label.setStyleSheet(f"color:{pal.text};background:transparent;")
        if self.tray_icon is not None:
            self.tray_icon.setIcon(QIcon(icons.app_icon(64, pal.accent, pal.dark)))
        self.setWindowIcon(QIcon(icons.app_icon(128, pal.accent, pal.dark)))
        self.content.setMinimumWidth(0)
        self.scroll.setGeometry(self.centralWidget().rect())

    # ------------------------------------------------------------------
    # Действия интерфейса
    # ------------------------------------------------------------------

    def _toggle_sheet(self, name: str):
        sheet = self.sheets.get(name)
        if sheet is None:
            return
        for key, other in self.sheets.items():
            if key != name and other.isVisible():
                other.close()
        if sheet.isVisible():
            sheet.close()
            return
        if name == "journal":
            sheet.fill(self.log_history)
        sheet.setGeometry(self.centralWidget().rect())
        sheet.open()

    def _close_sheets(self):
        for sheet in self.sheets.values():
            if sheet.isVisible():
                sheet.close()

    def _on_sheet_closed(self):
        self._refresh_dashboard()

    def _toggle_theme(self):
        """Один клик: полностью тёмная ↔ полностью светлая тема."""
        current = self.theme.palette.dark
        self.theme.update(mode="light" if current else "dark")
        self.controller.log_now("Тема: " + ("светлая" if current else "тёмная"), "info")

    def _toggle_autopilot(self):
        value = not self.theme.settings.autopilot
        self.theme.update(autopilot=value)
        self.controller.log_now("Автопилот " + ("включён" if value else "выключен"), "ok")
        self.toasts.show_toast("Автопилот " + ("включён" if value else "выключен"), "info")

    def _on_autopilot_switch(self, value: bool):
        self.theme.update(autopilot=bool(value))

    def _on_gamefilter(self, value: bool):
        self.controller.cfg["gamefilter_tcp"] = bool(value)
        self.controller.cfg["gamefilter_udp"] = bool(value)
        self.controller.set_config_value("gamefilter_tcp", bool(value))
        self.controller.set_config_value("gamefilter_udp", bool(value))
        if self.controller.status.get("running"):
            self.controller.restart_with_current()

    def _set_buttons_enabled(self, enabled: bool):
        for btn in (self.autopilot_quick, self.btn_deps, self.btn_update, self.btn_check_update,
                    self.btn_repair, self.btn_autostart, self.btn_shortcut,
                    self.recheck_btn, self.btn_setup_rights, self.btn_target_test):
            btn.setEnabled(enabled)

    def _copy_log(self):
        text = "\n".join(f"[{time.strftime('%H:%M:%S', time.localtime(ts))}] {msg}"
                         for ts, msg, _k in self.log_history)
        QApplication.clipboard().setText(text)
        self.toasts.show_toast("Журнал скопирован в буфер обмена", "ok")

    def _open_readme(self):
        path = app_dir() / "README.md"
        if not path.exists():
            path = Path(__file__).resolve().parent.parent.parent / "README.md"
        try:
            if shutil.which("xdg-open"):
                subprocess.Popen(["xdg-open", str(path)],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                self.controller.log_now(f"README: {path}", "info")
        except OSError as exc:
            self.controller.log_now(f"Не удалось открыть README: {exc}", "warn")

    def _open_terminal(self) -> bool:
        """Открывает терминал для настройки NOPASSWD (пароль вводит пользователь)."""
        command = (f"{sys.executable} {app_dir() / 'run.py'} permissions install; "
                   f"echo; read -p 'Нажмите Enter, чтобы закрыть…'")
        terms = [
            ("x-terminal-emulator", ["-e", "bash", "-c", command]),
            ("gnome-terminal", ["--", "bash", "-c", command]),
            ("konsole", ["-e", "bash", "-c", command]),
            ("xfce4-terminal", ["-e", "bash", "-c", command]),
            ("mate-terminal", ["-e", "bash", "-c", command]),
            ("alacritty", ["-e", "bash", "-c", command]),
            ("kitty", ["bash", "-c", command]),
            ("tilix", ["-e", "bash", "-c", command]),
            ("xterm", ["-e", "bash", "-c", command]),
        ]
        for term, args in terms:
            path = shutil.which(term)
            if path:
                try:
                    subprocess.Popen([path] + args, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                except OSError:
                    continue
                return True
        return False

    # ------------------------------------------------------------------
    # Отладочные помощники
    # ------------------------------------------------------------------

    def demo_mode(self, state: str = "on"):
        """Заполняет интерфейс тестовыми данными (для скриншотов и проверки)."""
        self.controller.demo = True
        if state == "on":
            self.power.set_state("on", "ВКЛ", "Обход активен · нажмите, чтобы выключить")
            self.power.set_progress(1.0)
            self.controller.status = {"running": True, "firewall": True, "deps_ready": True,
                                      "service_installed": True, "sudo_ok": True,
                                      "backend": "nftables",
                                      "app_dir": str(app_dir())}
            self.controller.started_at = time.time() - 8100
            self.controller.services = [
                {"key": "youtube", "state": "ok", "latency_ms": 64, "title": "YouTube"},
                {"key": "discord", "state": "ok", "latency_ms": 88, "title": "Discord"},
                {"key": "telegram", "state": "warn", "latency_ms": 480, "title": "Telegram"},
            ]
            self.controller.last_check_at = time.time() - 12
            self.controller.checks_done = 14
            for service in self.controller.services:
                self.service_rows[service["key"]].set_status(
                    service["state"], {"ok": f"{service['latency_ms']} мс · работает",
                                       "warn": f"{service['latency_ms']} мс · медленно"}[service["state"]])
            for value in (0.4, 0.8, 1.4, 2.2, 3.1, 2.7, 3.6, 4.2, 3.8, 4.6, 5.2, 4.9, 5.6, 6.2,
                          5.8, 6.4, 5.9, 6.8, 7.2, 6.6, 7.4, 8.1, 7.6, 8.4):
                self.spark.push(value)
            self._on_metrics({"iface": "wlan0", "rx_mbps": 8.4, "tx_mbps": 0.9,
                              "total_rx": 3_100_000_000, "total_tx": 320_000_000})
        self._refresh_dashboard()


# ---------------------------------------------------------------------------
# Дополнительные помощники
# ---------------------------------------------------------------------------

class SelfLabel(QLabel):
    """Подпись, которая сама подстраивает цвет под тему."""

    def __init__(self, theme, text: str, parent=None):
        super().__init__(text, parent)
        self.theme = theme
        self._restyle()
        theme.changed.connect(self._restyle)

    def _restyle(self):
        pal = self.theme.palette
        self.setFont(font(self.theme.font_family, pal.font_sm, QFont.Weight.DemiBold))
        self.setStyleSheet(f"color:{pal.muted};background:transparent;")


def _wrap(widget: QWidget) -> QWidget:
    """Обёртка без отступов — чтобы Switch корректно вставал в строку настройки."""
    holder = QWidget()
    layout = QHBoxLayout(holder)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(0)
    layout.addWidget(widget)
    hint = widget.sizeHint()
    if hint.width() > 0 and hint.height() > 0:
        holder.setFixedSize(hint)
    return holder


def run() -> int:
    """Точка входа Qt-интерфейса."""
    app = QApplication.instance() or QApplication(sys.argv)
    # Fusion одинаково выглядит во всех дистрибутивах и слушается нашей палитры —
    # без него на GNOME/KDE в тёмной теме оставались светлые системные подложки.
    app.setStyle("Fusion")
    app.setApplicationName(APP_NAME)
    app.setApplicationVersion(APP_VERSION)
    app.setQuitOnLastWindowClosed(False if QSystemTrayIcon.isSystemTrayAvailable() else True)

    from .. import config as config_mod

    cfg = config_mod.load()
    from .theme import UISettings

    theme = ThemeManager(UISettings.from_dict(cfg.get("ui")))
    apply_theme_to_app(app, theme)
    theme.changed.connect(lambda: apply_theme_to_app(app, theme))
    controller = Controller(theme)

    window = ZapretWindow(theme, controller, tray=bool(theme.settings.tray))
    window.show()
    controller.start()

    if os.environ.get("ZAPRET_DEMO") == "1":
        window.demo_mode()

    return app.exec()
