"""
Мост между ядром zapret и интерфейсом Qt.

Контроллер живёт в GUI-потоке, но все тяжёлые операции (запуск nfqws, скачивание
зависимостей, автоподбор стратегии, проверки сервисов) выполняются в отдельных
потоках и общаются с интерфейсом через сигналы. За счёт этого окно никогда не
замирает — главная проблема прошлой версии на tkinter.
"""

from __future__ import annotations

import threading
import time

from PySide6.QtCore import QObject, QTimer, Signal

from .. import checks, config as config_mod, core, integration
from .. import autopilot

OPERATION_LABELS = {
    "power_on": "Включение защиты",
    "power_off": "Выключение защиты",
    "deps": "Скачивание зависимостей",
    "autopilot": "Автоподбор стратегии",
    "update": "Обновление приложения",
    "service_install": "Установка автозапуска",
    "service_remove": "Удаление автозапуска",
    "shortcut": "Создание ярлыка",
    "bootstrap": "Первый запуск",
    "restart": "Перезапуск с новой стратегией",
}


class Controller(QObject):
    """Состояние приложения и все действия над ним."""

    log = Signal(str, str)                 # сообщение, уровень (info/ok/warn/error/accent)
    changed = Signal()                     # что-то изменилось — обновить экран
    status_ready = Signal(dict)
    services_ready = Signal(list)
    services_checking = Signal(list)       # ключи сервисов, которые сейчас проверяются
    metrics_ready = Signal(dict)
    busy_changed = Signal(str)             # ключ операции или ""
    finished = Signal(str, bool, str)      # операция, успех, сообщение
    notify = Signal(str, str)              # заголовок, текст (трей/уведомления)

    def __init__(self, theme, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.cfg = config_mod.load()
        self.status: dict = {}
        self.services: list[dict] = []
        self.metrics: dict = {}
        self.busy_key: str = ""
        self.progress: float | None = None
        # Пользователь выключил защиту вручную: автовосстановление не должно
        # включать её обратно «за спиной»
        self.user_stopped = False
        # Демонстрационный режим: интерфейс показывает тестовые данные, реальные
        # проверки и автовосстановление отключены (используется для скриншотов)
        self.demo = False
        self.started_at: float | None = None
        self.last_check_at: float | None = None
        self.checks_done = 0
        self.error_message = ""
        self.hint = ""
        self.autopilot_report: dict | None = None

        self._traffic = checks.TrafficMonitor(self.cfg.get("interface", "any")
                                              if self.cfg.get("interface") not in (None, "any") else None)
        self._status_inflight = False
        self._services_inflight = False
        self._stop_flag = False

        self._status_timer = QTimer(self)
        self._status_timer.setInterval(3000)
        self._status_timer.timeout.connect(self.refresh_status)

        self._metrics_timer = QTimer(self)
        self._metrics_timer.setInterval(1500)
        self._metrics_timer.timeout.connect(self._sample_traffic)

        self._services_timer = QTimer(self)
        self._services_timer.timeout.connect(self._periodic_services)

        theme.changed.connect(self._on_theme_changed)
        self._apply_intervals()
        QTimer.singleShot(0, self.refresh_status)
        QTimer.singleShot(50, lambda: self._sample_traffic())

    # ------------------------------------------------------------------
    # Настройки
    # ------------------------------------------------------------------

    @property
    def ui(self):
        return self.theme.settings

    def _on_theme_changed(self):
        data = self.cfg.get("ui") or {}
        data.update(self.theme.settings.to_dict())
        self.cfg["ui"] = data
        config_mod.save(self.cfg)
        self._apply_intervals()

    def _apply_intervals(self):
        ui = self.ui
        self._services_timer.setInterval(int(ui.check_interval * 1000))
        if ui.tray or True:
            self._status_timer.start()
            self._metrics_timer.start()
            if ui.autostart_protection or ui.auto_recover:
                self._services_timer.start()
            else:
                self._services_timer.stop()

    def set_config_value(self, key: str, value):
        self.cfg[key] = value
        config_mod.save(self.cfg)
        self.log.emit(f"Настройка «{key}» = {value}", "info")
        self.changed.emit()

    # ------------------------------------------------------------------
    # Запуск/остановка обслуживания
    # ------------------------------------------------------------------

    def start(self):
        self.log.emit("Zapret Control запущен.", "accent")
        self.refresh_status()
        QTimer.singleShot(600, self._sample_traffic)

    def log_now(self, message: str, level: str = "info"):
        self.log.emit(message, level)

    # ------------------------------------------------------------------
    # Статус
    # ------------------------------------------------------------------

    def refresh_status(self):
        if self.demo or self._status_inflight:
            return
        self._status_inflight = True

        def worker():
            try:
                status = core.status()
            except Exception as exc:  # noqa: BLE001
                status = {"error": str(exc)}
            self._status_inflight = False
            self._apply_status(status)

        threading.Thread(target=worker, daemon=True, name="zapret-status").start()

    def _apply_status(self, status: dict):
        previous = self.status
        self.status = status
        running = bool(status.get("running"))
        firewall = bool(status.get("firewall"))
        power_on = running and firewall

        if power_on and self.started_at is None:
            self.started_at = time.time()
        if not running:
            self.started_at = None

        self.status_ready.emit(status)

        # Автопилот-реакция: защита упала сама — поднимаем
        if (self.ui.auto_recover and not self.user_stopped and not self.demo
                and not self.busy_key and previous.get("running") and not running):
            self.log.emit("Защита остановилась — поднимаю автоматически.", "warn")
            self.notify.emit("Zapret Control", "Защита остановилась, включаю снова.")
            self.turn_on(reason="auto-recover")

        if power_on and not previous.get("running"):
            self.notify.emit("Защита включена", "Обход DPI активен.")
        if previous.get("running") and not running and not self.ui.auto_recover:
            self.notify.emit("Защита выключена", "Обход DPI остановлен.")

        self.changed.emit()

    # ------------------------------------------------------------------
    # Трафик и сервисы
    # ------------------------------------------------------------------

    def _sample_traffic(self):
        try:
            data = self._traffic.sample()
        except Exception:  # noqa: BLE001
            return
        self.metrics = data
        self.metrics_ready.emit(data)
        self.changed.emit()

    def _periodic_services(self):
        if self.demo or not self.status.get("running"):
            return
        self.check_services()

    def check_services(self, silent: bool = False):
        if self._services_inflight:
            return
        self._services_inflight = True
        keys = [s.key for s in checks.SERVICES]
        self.services_checking.emit(keys)

        def worker():
            try:
                results = checks.probe_all(timeout=6.0)
            except Exception as exc:  # noqa: BLE001
                self.log.emit(f"Проверка сервисов не удалась: {exc}", "error")
            else:
                self.services = results
                self.last_check_at = time.time()
                self.checks_done += 1
                self.services_ready.emit(results)
                if not silent:
                    summary = ", ".join(
                        f"{r['title']}: {'ок' if r['state'] == 'ok' else ('медленно' if r['state'] == 'warn' else 'нет')}"
                        for r in results)
                    self.log.emit(f"Проверка сервисов — {summary}", "info")
            finally:
                self._services_inflight = False
                self.changed.emit()

        threading.Thread(target=worker, daemon=True, name="zapret-probe").start()

    # ------------------------------------------------------------------
    # Универсальный запуск операций
    # ------------------------------------------------------------------

    def _set_busy(self, key: str):
        self.busy_key = key
        self.busy_changed.emit(key)
        self.changed.emit()

    def _submit(self, key: str, work, success: str = "", failure: str = "Ошибка"):
        if self.busy_key:
            self.log.emit(f"Сейчас выполняется: "
                          f"{OPERATION_LABELS.get(self.busy_key, self.busy_key)}. "
                          f"Дождитесь завершения.", "warn")
            return
        self._set_busy(key)

        def runner():
            try:
                work()
            except Exception as exc:  # noqa: BLE001
                self.error_message = str(exc)
                self.hint = self._hint_for(exc)
                self.log.emit(f"{failure}: {exc}", "error")
                self.log.emit(self.hint, "warn")
                self.finished.emit(key, False, str(exc))
            else:
                self.error_message = ""
                self.hint = ""
                if success:
                    self.log.emit(success, "ok")
                self.finished.emit(key, True, "")
            finally:
                self.progress = None
                self._set_busy("")
                self.refresh_status()

        threading.Thread(target=runner, daemon=True, name=f"zapret-{key}").start()

    def _hint_for(self, exc: Exception) -> str:
        text = str(exc).lower()
        if "nftables" in text or "iptables" in text:
            return ("Не установлен файрвол. Установите nftables: "
                    "sudo apt install nftables (Debian/Ubuntu), "
                    "sudo pacman -S nftables (Arch), sudo dnf install nftables (Fedora).")
        if "интернет" in text or "соединение" in text:
            return "Проверьте интернет и повторите — автопилоту нужно проверить сервисы."
        if "sudo" in text or "пароль" in text or "permission" in text or "root" in text:
            return ("Не хватает прав. Нажмите «Настроить права (1 раз)» — приложение "
                    "попросит пароль sudo и больше не будет его спрашивать.")
        if "nfqws" in text or "зависимост" in text:
            return "Нажмите «Обновить зависимости» в карточке «Обслуживание»."
        return "Попробуйте ещё раз или откройте журнал — там подробности."

    # ------------------------------------------------------------------
    # Питание
    # ------------------------------------------------------------------

    def toggle_power(self):
        if self.busy_key:
            self.log.emit("Операция уже выполняется, подождите.", "warn")
            return
        if self.status.get("running"):
            self.turn_off()
        else:
            self.turn_on()

    def turn_on(self, reason: str = ""):
        if self.busy_key:
            return
        self.user_stopped = False
        cfg = dict(self.cfg)
        ui = self.ui

        def work():
            if not core.deps_ready():
                self.log.emit("Зависимости не найдены — скачиваю автоматически.", "accent")
                core.ensure_deps(cfg.get("nfqws_version", "latest"),
                                 cfg.get("strategy_rev", ""),
                                 lambda msg: self.log.emit(msg, "info"))
            if ui.autopilot:
                report = autopilot.run(
                    self.cfg,
                    progress_cb=lambda msg: self.log.emit(msg, "info"),
                    stop_flag=lambda: self._stop_flag,
                    limit=6,
                )
                self.autopilot_report = report
            else:
                core.run_zapret(self.cfg)
                self.services = checks.probe_all(timeout=6.0)
                self.services_ready.emit(self.services)
                self.last_check_at = time.time()

        label = {"auto-recover": "Автовосстановление"}
        self._submit("power_on", work,
                     success=f"Защита включена{(' · ' + label[reason]) if reason in label else ''}.",
                     failure="Не удалось включить защиту")

    def turn_off(self):
        self.user_stopped = True
        self._submit("power_off", core.stop_zapret, "Защита выключена.",
                     "Не удалось остановить защиту")

    def restart_with_current(self):
        self._submit("restart", lambda: core.run_zapret(self.cfg),
                     "Настройки применены.", "Не удалось перезапустить")

    def run_autopilot(self):
        def work():
            self.autopilot_report = autopilot.run(
                self.cfg,
                progress_cb=lambda msg: self.log.emit(msg, "info"),
                stop_flag=lambda: self._stop_flag,
                limit=8,
            )
            self.services = self.autopilot_report.get("results") or []
            if self.services:
                self.services_ready.emit(self.services)
                self.last_check_at = time.time()

        self._submit("autopilot", work, "Автоподбор завершён.", "Автоподбор не удался")

    def stop_autopilot(self):
        self._stop_flag = True
        QTimer.singleShot(100, lambda: setattr(self, "_stop_flag", False))

    # ------------------------------------------------------------------
    # Обслуживание
    # ------------------------------------------------------------------

    def download_deps(self):
        def work():
            self.progress = 0.05
            core.ensure_deps(self.cfg.get("nfqws_version", "latest"),
                             self.cfg.get("strategy_rev", ""),
                             lambda msg: self.log.emit(msg, "info"))
            self.progress = 1.0

        self._submit("deps", work, "Зависимости готовы.", "Не удалось скачать зависимости")

    def update_app(self):
        from .. import update as update_mod

        def work():
            result = update_mod.update_app(lambda msg: self.log.emit(msg, "info"))
            if not result.get("code_updated"):
                self.log.emit("Код приложения не обновлялся (нет git-копии или нет изменений).",
                              "warn")

        self._submit("update", work, "Обновление завершено.", "Обновление не удалось")

    def toggle_autostart(self):
        if integration.service_installed():
            self._submit("service_remove", integration.remove_service,
                         "Автозапуск удалён: система больше не поднимает обход сама.",
                         "Не удалось удалить автозапуск")
        else:
            self._submit("service_install", integration.install_service,
                         "Автозапуск установлен: обход будет включаться при загрузке системы.",
                         "Не удалось установить автозапуск")

    def toggle_shortcut(self):
        def work():
            if integration.shortcut_installed():
                integration.remove_shortcut()
            else:
                integration.install_shortcut()

        self._submit("shortcut", work, "Ярлык в меню приложений обновлён.",
                     "Не удалось изменить ярлык")

    def setup_permissions(self, open_terminal) -> None:
        """Настройка NOPASSWD: если есть графический терминал — запускаем в нём."""
        if integration.permissions_ready() and not integration.service_installed():
            self.log.emit("Права sudo уже настроены.", "ok")
            self.changed.emit()
            return
        opened = open_terminal()
        if opened:
            self.log.emit("Открыт терминал — введите пароль sudo один раз.", "accent")
        else:
            self.log.emit("Не найден терминал. Выполните вручную: "
                          "python3 run.py permissions install", "warn")

    # ------------------------------------------------------------------
    # Первый запуск
    # ------------------------------------------------------------------

    def bootstrap(self):
        """Автономный старт: зависимости → защита → проверка сервисов."""
        if self.status.get("running"):
            self.check_services(silent=True)
            return
        ui = self.ui
        if not ui.autostart_protection and core.deps_ready():
            self.log.emit("Автовключение защиты выключено — включаю только проверку.", "info")
            self.check_services(silent=True)
            return

        def work():
            if not core.deps_ready():
                self.log.emit("Первый запуск: скачиваю nfqws и стратегии Flowseal…", "accent")
                core.ensure_deps(self.cfg.get("nfqws_version", "latest"),
                                 self.cfg.get("strategy_rev", ""),
                                 lambda msg: self.log.emit(msg, "info"))
                self.log.emit("Зависимости готовы.", "ok")
            if ui.autopilot:
                self.autopilot_report = autopilot.run(
                    self.cfg,
                    progress_cb=lambda msg: self.log.emit(msg, "info"),
                    stop_flag=lambda: self._stop_flag,
                    limit=5,
                )
            else:
                core.run_zapret(self.cfg)
            self.services = checks.probe_all(timeout=6.0)
            self.services_ready.emit(self.services)
            self.last_check_at = time.time()

        self._submit("bootstrap", work, "Готово: защита включена.",
                     "Автовключение не удалось")

    # ------------------------------------------------------------------
    # Сводка для интерфейса
    # ------------------------------------------------------------------

    def power_state(self) -> str:
        if self.busy_key:
            return "busy"
        if self.error_message and not self.status.get("running"):
            return "error"
        running = bool(self.status.get("running"))
        firewall = bool(self.status.get("firewall"))
        if running and firewall:
            return "on"
        if running:
            return "on"
        return "off"

    def uptime_text(self) -> str:
        if not self.started_at:
            return "—"
        seconds = int(time.time() - self.started_at)
        hours, remainder = divmod(seconds, 3600)
        minutes, secs = divmod(remainder, 60)
        if hours:
            return f"{hours} ч {minutes} м"
        if minutes:
            return f"{minutes} мин {secs} с"
        return f"{secs} с"

    def average_latency(self) -> int | None:
        values = [r["latency_ms"] for r in self.services if r.get("state") in ("ok", "warn")]
        if not values:
            return None
        return int(sum(values) / len(values))

    def ok_services_text(self) -> str:
        if not self.services:
            return "—"
        ok = sum(1 for r in self.services if r["state"] == "ok")
        return f"{ok} из {len(self.services)}"

    def last_check_text(self) -> str:
        if not self.last_check_at:
            return "проверок пока не было"
        seconds = int(time.time() - self.last_check_at)
        if seconds < 60:
            return f"проверено {seconds} с назад"
        return f"проверено {seconds // 60} мин назад"
