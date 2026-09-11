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
from .. import autopilot, presets as presets_mod

OPERATION_LABELS = {
    "power_on": "Включение защиты",
    "power_off": "Выключение защиты",
    "deps": "Скачивание зависимостей",
    "autopilot": "Автоподбор стратегии",
    "update": "Обновление приложения",
    "repair": "Починка установки",
    "service_install": "Установка автозапуска",
    "service_remove": "Удаление автозапуска",
    "shortcut": "Создание ярлыка",
    "bootstrap": "Первый запуск",
    "restart": "Перезапуск с новой стратегией",
    "targets": "Подбор стратегии под сайт",
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
    update_info_ready = Signal(dict)       # результат проверки обновлений
    restart_requested = Signal(str)        # интерфейс должен закрыться и запуститься заново
    targets_started = Signal(list)          # сайты, для которых начат подбор
    targets_step = Signal(dict)             # промежуточный итог по стратегии
    targets_done = Signal(dict)             # итоговый отчёт подбора
    presets_changed = Signal(list)          # сохранённые пресеты сайтов

    def __init__(self, theme, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.cfg = config_mod.load()
        self.status: dict = {}
        self.services: list[dict] = []
        self.metrics: dict = {}
        self.busy_key: str = ""
        self.busy_detail: str = ""
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
        self.target_report: dict | None = None
        # Обновление приложения: результат проверки новой версии и итог обновления
        self.update_info: dict = {}
        self.update_result: dict = {}
        self.repair_result: dict = {}
        self._restart_after_update = False
        self._relaunch_plan: dict | None = None
        self._update_check_running = False
        self.finished.connect(self._on_operation_finished)

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
        try:
            from PySide6.QtWidgets import QApplication

            instance = QApplication.instance()
            if instance is not None:
                self.log.emit(f"Платформа Qt: {instance.platformName()}", "info")
        except Exception:  # noqa: BLE001 — строка диагностики, не критично
            pass
        self.refresh_status()
        QTimer.singleShot(600, self._sample_traffic)
        QTimer.singleShot(1500, self._log_environment)
        if not self.demo:
            # проверка обновлений в фоне: медленный GitHub не должен тормозить окно
            QTimer.singleShot(4000, self.check_update)

    def _log_environment(self):
        """Проверка окружения при старте: сразу говорит, что мешает работе."""
        problems: list[tuple[str, str]] = []
        if not (core.which("nft") or core.which("iptables")):
            problems.append(("Не найден nftables или iptables — без файрвола обход не "
                             "включится. Установите: sudo apt install nftables", "error"))
        if not core.deps_ready():
            problems.append(("Зависимости (nfqws и стратегии) не скачаны — скачаю при "
                             "первом включении автоматически.", "warn"))
        if not self.status.get("sudo_ok"):
            problems.append(("Права без пароля не настроены: нажмите «Настроить права "
                             "(1 раз)» — иначе кнопка будет просить пароль.", "warn"))
        from .. import session

        owner = session.desktop_user()
        if session.is_root() and owner != "root":
            problems.append((f"Приложение запущено от root, а рабочий стол у «{owner}»: "
                             "ярлык и права могут лежать не там. Нажмите «Починить "
                             "установку» — приложение разложит всё по местам.", "error"))
        if not integration.shortcut_status().get("desktop"):
            problems.append(("На рабочем столе нет значка Zapret Control — нажмите "
                             "«Создать ярлык», чтобы он появился.", "warn"))
        if problems:
            self.log.emit("Проверка окружения нашла замечания:", "accent")
            for message, level in problems:
                self.log.emit("  • " + message, level)
        else:
            self.log.emit("Проверка окружения: всё на месте.", "ok")

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
        if not key:
            self.busy_detail = ""
        self.busy_changed.emit(key)
        self.changed.emit()

    def _on_autopilot_step(self, index: int, total: int, strategy: str):
        """Автопилот переходит к следующей стратегии — обновляем прогресс.

        Берём середину шага, чтобы на первой же стратегии кнопка показывала
        движение, а не нулевой прогресс.
        """
        self.progress = (index - 0.5) / max(1, total)
        self.busy_detail = f"Стратегия {index} из {total}: {strategy}"
        self.changed.emit()

    def _on_targets_step(self, index: int, total: int, strategy: str):
        """Подбор под сайт переходит к следующей стратегии."""
        self._on_autopilot_step(index, total, strategy)
        self.targets_step.emit({"index": index, "total": total, "strategy": strategy})

    def _submit(self, key: str, work, success: str = "", failure: str = "Ошибка"):
        # Кнопка питания («ВЫКЛ» / «ОСТАНОВИТЬ») может прервать любую операцию.
        override_stop = (key == "power_off")
        if self.busy_key and not override_stop:
            self.log.emit(f"Сейчас выполняется: "
                          f"{OPERATION_LABELS.get(self.busy_key, self.busy_key)}. "
                          f"Дождитесь завершения.", "warn")
            return
        # При остановке сбрасываем флаг остановки, чтобы автоподбор/автовосстановление
        # не включились «за спиной».
        if override_stop:
            self.user_stopped = True
            self._stop_flag = True
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
        if "репозитори" in text or "github" in text or "обновить код" in text:
            return ("Похоже, GitHub сейчас недоступен. Попробуйте позже или обновитесь "
                    "вручную: curl -fsSL https://raw.githubusercontent.com/danilka-revin/"
                    "Zapret/main/install.sh -o /tmp/zc-install.sh && bash /tmp/zc-install.sh")
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
                    on_step=self._on_autopilot_step,
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
                on_step=self._on_autopilot_step,
            )
            self.services = self.autopilot_report.get("results") or []
            if self.services:
                self.services_ready.emit(self.services)
                self.last_check_at = time.time()

        self._submit("autopilot", work, "Автоподбор завершён.", "Автоподбор не удался")

    # ------------------------------------------------------------------
    # Подбор стратегии под конкретные сайты
    # ------------------------------------------------------------------

    def target_history(self) -> list[str]:
        ui = self.cfg.get("ui") or {}
        history = ui.get("target_history") or []
        return [str(x) for x in history][:8]

    def _remember_targets(self, query: str):
        ui = dict(self.cfg.get("ui") or {})
        history = [query] + [x for x in (ui.get("target_history") or []) if x != query]
        ui["target_history"] = history[:8]
        self.cfg["ui"] = ui
        config_mod.save(self.cfg)

    # -- группы сайтов: готовые и свои ----------------------------------

    def groups(self) -> list[dict]:
        """Готовые группы сервисов (YouTube, Discord, Telegram…) и группы пользователя."""
        return presets_mod.all_groups(self.cfg)

    def group_hosts(self, keys: list[str]) -> list[str]:
        return presets_mod.hosts_for_selection(self.cfg, keys)

    def add_group(self, title: str, hosts: list[str]) -> dict | None:
        try:
            group = presets_mod.add_group(self.cfg, title, hosts)
        except ValueError as exc:
            self.log.emit(str(exc), "warn")
            self.finished.emit("group", False, str(exc))
            return None
        self.log.emit(f"Группа «{group['title']}» сохранена: "
                      f"{len(group['hosts'])} домен(ов).", "ok")
        self.finished.emit("group", True, "")
        return group

    def remove_group(self, key: str) -> bool:
        group = presets_mod.group_by_key(self.cfg, key)
        if not presets_mod.remove_group(self.cfg, key):
            return False
        title = (group or {}).get("title", key)
        self.log.emit(f"Группа «{title}» удалена.", "info")
        self.finished.emit("group", True, "")
        return True

    # -- пресеты: домены + найденная стратегия --------------------------

    def site_presets(self) -> list[dict]:
        return presets_mod.presets(self.cfg)

    def save_preset(self, name: str, hosts: list[str], strategy: str = "",
                    ok: int = 0, total: int = 0, avg_ms: float = 0.0) -> dict | None:
        try:
            preset = presets_mod.save_preset(self.cfg, name, hosts, strategy,
                                             ok=ok, total=total, avg_ms=avg_ms)
        except ValueError as exc:
            self.log.emit(str(exc), "warn")
            self.finished.emit("preset", False, str(exc))
            return None
        self.presets_changed.emit(self.site_presets())
        self.log.emit(f"Пресет «{preset['name']}» сохранён: {preset['strategy']} "
                      f"для {len(preset['hosts'])} домен(ов).", "ok")
        self.finished.emit("preset", True, "")
        return preset

    def remove_preset(self, name: str) -> bool:
        if not presets_mod.remove_preset(self.cfg, name):
            return False
        self.presets_changed.emit(self.site_presets())
        self.log.emit(f"Пресет «{name}» удалён.", "info")
        self.finished.emit("preset", True, "")
        return True

    def check_custom_domain(self, domain: str):
        """Проверяет одну стратегию под конкретный домен — быстрее полного автопилота."""
        if not domain or self.busy_key:
            return
        hosts = checks.parse_targets(domain)
        if not hosts:
            hosts = [checks.clean_host(domain) or domain]
        hosts = [h for h in hosts if h]
        if not hosts:
            self.log.emit(f"Не понял домен: {domain}", "warn")
            self.finished.emit("targets", False, "непонятный домен")
            return
        self.log.emit(f"Проверяю домен: {hosts[0]}", "accent")
        self.test_targets(domain, apply_best=True, hosts=hosts, groups=[])

    def run_deep_scan(self):
        """Глубокий скан — больше стратегий, дольше, но точнее."""
        if self.busy_key:
            self.log.emit("Сейчас уже выполняется операция — дождитесь завершения.", "warn")
            return
        self.run_autopilot()
        # Можно усилить: после обычного автопилота запускаем форсированный перебор
        def work():
            # Просто повторяем с большим лимитом для более глубокого поиска
            self.autopilot_report = autopilot.run(
                self.cfg,
                progress_cb=lambda msg: self.log.emit(msg, "info"),
                stop_flag=lambda: self._stop_flag,
                limit=10,
                on_step=self._on_autopilot_step,
            )
            if self.autopilot_report:
                self.log.emit(f"Глубокий скан: лучшая — "
                              f"{self.autopilot_report.get('strategy', '—')}, "
                              f"результат: {self.autopilot_report.get('ok', 0)}/"
                              f"{self.autopilot_report.get('total', 0)}.", "ok")

        self._submit("autopilot", work, "Глубокий скан завершён.", "Глубокий скан не удался")

    def backup_config(self, path: str = "") -> str:
        import json, shutil
        src = app_dir() / "zapret-config.json"
        dst = Path(path) if path else (app_dir() / f"zapret-config-backup-{time.strftime('%Y%m%d-%H%M')}.json")
        if not src.exists():
            return ""
        shutil.copy(str(src), str(dst))
        return str(dst)

    def restore_config(self, path: str = ""):
        import shutil
        src = Path(path) if path else None
        if not src or not src.exists():
            self.log.emit("Не указан файл для восстановления.", "warn")
            return False
        dst = app_dir() / "zapret-config.json"
        shutil.copy(str(src), str(dst))
        self.cfg = config_mod.load()
        self.log.emit("Настройки восстановлены из резервной копии.", "ok")
        self.changed.emit()
        return True

    def check_preset(self, name: str):
        """Перепроверяет сохранённый пресет: сначала его же стратегией."""
        preset = presets_mod.preset_by_name(self.cfg, name)
        if not preset:
            self.log.emit(f"Пресет «{name}» не найден.", "warn")
            self.finished.emit("targets", False, "пресет не найден")
            return
        self.log.emit(f"Проверяю пресет «{preset['name']}» "
                      f"({len(preset['hosts'])} домен(ов)).", "info")
        self.test_targets("", apply_best=True, prefer=preset["strategy"],
                          preset_name=preset["name"], hosts=preset["hosts"])

    def apply_preset(self, name: str) -> bool:
        """Ставит стратегию из пресета — сразу, вместе с перезапуском обхода."""
        preset = presets_mod.preset_by_name(self.cfg, name)
        if not preset:
            self.log.emit(f"Пресет «{name}» не найден.", "warn")
            return False
        if not preset["strategy"]:
            self.log.emit(f"В пресете «{preset['name']}» ещё нет стратегии — "
                          f"сначала проверьте его.", "warn")
            return False
        self.cfg["strategy"] = preset["strategy"]
        config_mod.save(self.cfg)
        self.log.emit(f"Пресет «{preset['name']}»: стратегия {preset['strategy']}.",
                      "ok")
        if core.nfqws_running():
            self._submit(
                "apply",
                lambda: core.run_zapret(self.cfg),
                f"Обход перезапущен со стратегией {preset['strategy']}.",
                "Не удалось перезапустить обход")
        else:
            self.log.emit("Обход выключен — стратегия применится при включении.", "info")
        self.notify.emit("Пресет применён", f"{preset['name']} → {preset['strategy']}")
        self.changed.emit()
        return True

    # -- подбор ---------------------------------------------------------

    def test_targets(self, raw_query: str, apply_best: bool = True,
                     prefer: str = "", preset_name: str = "",
                     hosts: list[str] | None = None, groups: list[str] | None = None):
        """Перебирает стратегии под указанные сайты (или группы) и выбирает лучшую."""
        targets: list[str] = []
        if groups:
            targets.extend(presets_mod.hosts_for_selection(self.cfg, groups))
        if raw_query.strip():
            targets.extend(checks.parse_targets(raw_query))
        if hosts:
            targets.extend(checks.clean_host(h) or h for h in hosts)
        hosts = []
        for host in targets:
            if host and host not in hosts:
                hosts.append(host)

        if not hosts:
            self.log.emit("Не понял, какие сайты проверять. Введите домен, например "
                          "rutracker.org, или выберите группу.", "warn")
            self.finished.emit("targets", False, "не указаны сайты")
            return
        if not core.deps_ready():
            self.log.emit("Сначала нужны зависимости: nfqws и стратегии.", "warn")
            self._submit("deps", lambda: core.ensure_deps(
                self.cfg.get("nfqws_version", "latest"), self.cfg.get("strategy_rev", ""),
                lambda m: self.log.emit(m, "info")), "Зависимости готовы.",
                "Не удалось скачать зависимости")
            self.finished.emit("targets", False, "нет зависимостей")
            return

        # В историю пишем уже разобранные домены — из ссылки вида
        # https://rutracker.org/forum/index.php остаётся понятное «rutracker.org».
        self._remember_targets(", ".join(hosts[:3]) + ("…" if len(hosts) > 3 else ""))
        self.targets_started.emit(hosts)
        if self.busy_key:
            self.log.emit("Сейчас выполняется другая операция — дождитесь завершения.", "warn")
            self.finished.emit("targets", False, "занято")
            return

        self._set_busy("targets")
        if groups:
            titles = [g["title"] for g in (presets_mod.group_by_key(self.cfg, key)
                                           for key in groups) if g]
            what = " + ".join(titles[:3]) if titles else f"{len(hosts)} сайтов"
        else:
            what = hosts[0] if len(hosts) == 1 else f"{len(hosts)} сайтов"

        def runner():
            try:
                report = autopilot.run_for_targets(
                    self.cfg, hosts,
                    progress_cb=lambda msg: self.log.emit(msg, "info"),
                    stop_flag=lambda: self._stop_flag,
                    on_step=self._on_targets_step,
                    apply_best=apply_best,
                    limit=8,
                    prefer=prefer,
                )
                self.target_report = report
                self.services = [
                    {**result, "key": result.get("host", ""), "icon": "globe"}
                    for result in report.get("results", [])
                ]
                self.targets_done.emit(report)

                # Результат подбора сохраняем в пресет: в следующий раз его можно
                # применить одним нажатием, не перебирая стратегии заново.
                name = (preset_name or "").strip()
                if name:
                    saved = presets_mod.update_preset(
                        self.cfg, name, strategy=report["strategy"], ok=report.get("ok", 0),
                        total=report.get("total", len(hosts)),
                        avg_ms=report.get("avg_ms", 0.0), hosts=hosts)
                    if saved is None:
                        saved = self.save_preset(name, hosts, report["strategy"],
                                                 report.get("ok", 0),
                                                 report.get("total", len(hosts)),
                                                 report.get("avg_ms", 0.0))
                    if saved:
                        self.presets_changed.emit(self.site_presets())
                        self.log.emit(f"Пресет «{name}» обновлён: "
                                      f"{report['strategy']}, "
                                      f"{report.get('ok')}/{report.get('total')} сайтов.",
                                      "ok")

                if report.get("applied"):
                    self.log.emit(f"Стратегия для «{what}»: {report['strategy']} — применена.",
                                  "ok")
                else:
                    self.log.emit(f"Лучшая стратегия для «{what}»: {report['strategy']} "
                                  f"(не применялась).", "accent")
                self.finished.emit("targets", True, "")
            except Exception as exc:  # noqa: BLE001
                self.error_message = str(exc)
                self.hint = self._hint_for(exc)
                self.log.emit(f"Подбор под сайт не удался: {exc}", "error")
                self.log.emit(self.hint, "warn")
                self.finished.emit("targets", False, str(exc))
            finally:
                self.progress = None
                self.busy_detail = ""
                self._set_busy("")
                self.refresh_status()

        self.busy_detail = "Подготовка"
        threading.Thread(target=runner, daemon=True, name="zapret-targets").start()

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
        """Обновить код и зависимости, но интерфейс не трогать (старое поведение)."""
        self.update_and_restart(restart=False)

    def update_and_restart(self, restart: bool = True):
        """«Обновить и перезапустить»: код + зависимости + ярлык + права, затем рестарт.

        Обновлённый код подхватывается только новым процессом, поэтому после
        успешного обновления приложение само отделяет от себя «ждущий» запуск и
        закрывается — обход при этом не прерывается (nfqws живёт отдельно).
        """
        from .. import update as update_mod

        self._restart_after_update = bool(restart)

        def step(message: str) -> None:
            self.log.emit(message, "info")
            self.busy_detail = message.strip().splitlines()[-1][:90] if message else ""
            self.changed.emit()

        def work():
            result = update_mod.update_all(step, with_deps=True, with_shortcut=True)
            self.update_result = result
            code = result.get("steps", {}).get("code", {})
            if code.get("changed"):
                self.log.emit(f"Код обновлён: {code.get('version_before', '')} → "
                              f"{code.get('version_after', '')}", "ok")
            else:
                self.log.emit("Код уже актуален.", "info")
            for failed in result.get("failed_steps", []):
                self.log.emit(f"Шаг «{failed}» не завершился — продолжил с остальным.", "warn")
            if not result.get("ok"):
                raise RuntimeError("Не удалось обновить код приложения: "
                                   + str(code.get("error") or "репозиторий недоступен"))
            if self._restart_after_update:
                # «ждущий» процесс готовим здесь: relaunch смотрит на запущенные
                # экземпляры через ps, а вешать на это GUI-поток нельзя — окно
                # станет неживым на секунды.
                self._relaunch_plan = update_mod.relaunch()
            self.progress = 1.0

        self._submit("update", work, "Обновление завершено.", "Обновление не удалось")

    def check_update(self, notify: bool = True) -> None:
        """Проверяет доступность новой версии — в фоне, чтобы окно не подвисало."""
        from .. import update as update_mod

        if self._update_check_running:
            return
        self._update_check_running = True

        def worker():
            try:
                info = update_mod.check_update()
            except Exception as exc:  # noqa: BLE001 — проверка обновлений не должна пугать
                info = {"available": None, "message": str(exc)}
            self._update_check_running = False
            self.update_info = info
            self.update_info_ready.emit(info)
            if info.get("available") and notify:
                self.log.emit("Доступна новая версия Zapret Control — нажмите "
                              "«Обновить и перезапустить».", "accent")
                self.notify.emit("Zapret Control", "Доступно обновление приложения.")
            elif notify and info.get("message"):
                self.log.emit(str(info["message"]), "info")

        threading.Thread(target=worker, daemon=True, name="zapret-update-check").start()

    def repair_install(self):
        """Чинит установку: права на каталог, ярлык на столе, NOPASSWD, перенос из /root."""
        from .. import repair as repair_mod

        def work():
            result = repair_mod.repair(lambda message: self.log.emit(message, "info"),
                                       launch=False, with_data=True)
            self.repair_result = result
            if not result.get("ok"):
                raise RuntimeError("Починка помогла не полностью — подробности в журнале")

        self._submit("repair", work, "Установка починена.", "Починка не удалась")

    def _on_operation_finished(self, key: str, success: bool, _message: str) -> None:
        """Перезапуск после обновления — уже в GUI-потоке, с таймером на закрытие."""
        if key != "update":
            return
        from .. import update as update_mod

        pending, self._restart_after_update = self._restart_after_update, False
        relaunch, self._relaunch_plan = self._relaunch_plan, None
        if not (pending and success):
            return
        if relaunch is None:
            relaunch = update_mod.relaunch()
        if relaunch.get("ok"):
            self.log.emit("Перезапускаю интерфейс с новой версией — обход не отключается.",
                          "accent")
            self.restart_requested.emit("Обновление применено — перезапускаю Zapret Control.")
        else:
            self.log.emit("Самоперезапуск не удался (" + str(relaunch.get("error"))
                          + "). Закройте окно и откройте заново: "
                          + update_mod.restart_command(), "warn")

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
        """Создать (или пересоздать) ярлык: в меню приложений + значок на рабочем столе.

        Раньше кнопка умела только «есть/нет», и если установщик положил ярлык
        в чужой домашний каталог (sudo -i), на столе по-прежнему ничего не было.
        Теперь ярлыки всегда пересоздаются для текущего пользователя.
        """
        def work():
            integration.remove_shortcut()
            placed = integration.install_shortcut()
            if placed.get("menu"):
                self.log.emit("Ярлык в меню приложений: " + str(placed["menu"]), "ok")
            if placed.get("desktop"):
                self.log.emit("Значок на рабочем столе: " + str(placed["desktop"]), "ok")
                if not placed.get("trusted"):
                    self.log.emit("GNOME может спросить разрешение: правый клик по "
                                  "значку → «Разрешить запуск».", "warn")
            else:
                self.log.emit("Каталога «Рабочий стол» нет — ярлык только в меню "
                              "приложений (наберите «Zapret» в поиске).", "warn")

        self._submit("shortcut", work, "Ярлык создан.", "Не удалось создать ярлык")

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
                    on_step=self._on_autopilot_step,
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
