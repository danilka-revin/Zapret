"""Кнопка «Остановить всё»: API пульта (8080) и бортового экрана (8090) + помощник.

Кнопка гасит пульт, бортовой экран, супервизор лидара ROS 2 и окно киоска —
то же, что делает человек командой ``./scripts/stop.sh``. Здесь проверяется
именно то, что ломается у такой кнопки:

* остановку нельзя запустить без входа оператора (8080) или без заголовка «от
  кнопки» (бортовой экран, где входа нет) — это единственное действие, после
  которого интерфейс перестаёт отвечать;
* помощник отсоединён от процесса сервера (иначе он умер бы вместе с ним) и
  получает pid сервера явно: приложение, запущенное вручную, ``stop.sh`` по
  pid-файлу не найдёт, а под gunicorn гасить надо мастера, иначе воркер
  поднимется заново;
* команда помощника действительно зовёт ``scripts/stop.sh`` и не бьёт сигналом
  наугад по чужому процессу, если pid успел переиспользоваться.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

from robot_control import shutdown
from robot_control.config import PROJECT_ROOT

from conftest import login


# ---------------------------------------------------------------------------
# Помощник остановки
# ---------------------------------------------------------------------------


def test_stop_script_exists_and_is_executable():
    """Кнопка вызывает тот же скрипт, что и человек руками — и он запускаем."""
    script = PROJECT_ROOT / shutdown.STOP_SCRIPT
    assert script.is_file(), script
    assert script.stat().st_mode & 0o111, "scripts/stop.sh без бита исполнения"
    assert "--servers" in script.read_text(encoding="utf-8")


def test_server_pid_uses_gunicorn_master():
    """Под gunicorn гасим мастера: убитый воркер мастер поднимет заново."""
    assert shutdown.server_pid(1000, parent=900,
                               parent_cmdline="gunicorn robot_control.app:create_app()") == 900
    assert shutdown.server_pid(1000, parent=900,
                               parent_cmdline="/usr/bin/python3 run.py --dev") == 1000
    assert shutdown.server_pid(1000, parent=1, parent_cmdline="gunicorn") == 1000


def test_stop_command_calls_script_and_guards_the_kill(tmp_path):
    command = shutdown.stop_command(tmp_path, delay=1.0, target=4321)
    assert str(tmp_path / "scripts" / "stop.sh") in command
    assert "sleep 1.0" in command
    # Сигнал только своему процессу: pid мог достаться чужому.
    assert "grep -qaE" in command and "/proc/$pid/cmdline" in command
    assert "kill -TERM" in command and "kill -KILL" in command
    # Погасить сервер надо после скрипта, а не до него.
    assert command.index("stop.sh") < command.index("kill -TERM")
    # И это одна строка: её видно в var/stop.log как есть.
    assert "\n" not in command


def test_stop_command_of_a_manual_run_targets_own_pid(tmp_path):
    """Приложение, поднятое вручную (run.py --dev), гасит само себя."""
    command = shutdown.stop_command(tmp_path, delay=0.0)
    assert "pid=%d" % os.getpid() in command


def test_stop_all_starts_a_detached_helper(tmp_path, monkeypatch):
    root = tmp_path
    (root / "scripts").mkdir()
    (root / "scripts" / "stop.sh").write_text("#!/usr/bin/env bash\necho стоп\n",
                                              encoding="utf-8")
    calls: dict = {}

    class FakeProcess:
        pid = 777001

    def fake_popen(argv, **kwargs):
        calls["argv"] = argv
        calls.update(kwargs)
        return FakeProcess()

    monkeypatch.setattr(shutdown.subprocess, "Popen", fake_popen)
    result = shutdown.stop_all(root, delay=0.5, log_path=root / "var" / "stop.log")

    assert result["ok"] is True and result["stopping"] is True
    assert result["helperPid"] == 777001
    assert result["serverPid"] > 0
    assert "пульт оператора (8080)" in result["targets"]
    assert result["restart"] and result["autostart"]
    # Отсоединённый помощник: остановка приложений не должна убивать его самого.
    assert calls["start_new_session"] is True
    assert calls["stdin"] is subprocess.DEVNULL
    assert calls["argv"][:2] == ["bash", "-c"]
    assert "stop.sh" in calls["argv"][2]
    # Журнал кнопки: по нему потом видно, что именно остановилось.
    log = (root / "var" / "stop.log").read_text(encoding="utf-8")
    assert "кнопка «Остановить всё»" in log


def test_stop_all_without_script_reports_the_error(tmp_path, monkeypatch):
    """Скрипта нет — честная ошибка, а не «остановлено»."""
    monkeypatch.setattr(shutdown.subprocess, "Popen",
                        lambda *_a, **_k: pytest.fail("помощник запущен без scripts/stop.sh"))
    result = shutdown.stop_all(tmp_path)
    assert result["ok"] is False and result["stopping"] is False
    assert "stop.sh" in result["error"]


def test_helper_runs_the_real_script_and_kills_only_our_process(tmp_path):
    """Настоящий bash: помощник зовёт stop.sh, гасит «сервер» и щадит чужих.

    Роль сервера играет отдельный процесс с ``run.py`` в командной строке (по
    ней помощник и узнаёт «своё» приложение), роль постороннего — обычный
    ``sleep``: его сигнал задеть не должен.
    """
    root = tmp_path
    (root / "scripts").mkdir()
    (root / "scripts" / "stop.sh").write_text(
        "#!/usr/bin/env bash\necho скрипт остановки сработал\n", encoding="utf-8")
    fake_run = root / "run.py"
    fake_run.write_text("import time\ntime.sleep(30)\n", encoding="utf-8")
    server = subprocess.Popen([sys.executable, str(fake_run)])
    stranger = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    log_path = root / "var" / "stop.log"
    try:
        command = shutdown.stop_command(root, delay=0.1, target=server.pid)
        subprocess.run(["bash", "-c", command], cwd=str(root), check=True, timeout=30)
        server.wait(timeout=10)
        assert server.returncode in (-15, -9), server.returncode
        assert stranger.poll() is None, "помощник погасил посторонний процесс"
        text = log_path.read_text(encoding="utf-8") if log_path.is_file() else ""
        assert "stop.sh" in command and text == ""  # журнал пишет stop_all, не stop_command
    finally:
        stranger.terminate()
        if server.poll() is None:
            server.kill()


# ---------------------------------------------------------------------------
# Пульт оператора (8080)
# ---------------------------------------------------------------------------


def test_stop_api_needs_login(client):
    """Без входа кнопка не работает: остановка — не анонимное действие."""
    status = client.get("/api/system/stop")
    assert status.status_code == 401
    assert status.get_json()["canStop"] is False
    assert client.post("/api/system/stop", json={}).status_code == 401


def test_stop_api_needs_csrf(client, operator):
    assert login(client).status_code in (302, 303)
    assert client.post("/api/system/stop", json={}).status_code == 400


def test_stop_api_reports_what_it_will_stop(client, operator):
    assert login(client).status_code in (302, 303)
    body = client.get("/api/system/stop").get_json()
    assert body["ok"] is True and body["canStop"] is True
    assert body["csrfToken"]
    assert any("8090" in item for item in body["targets"])
    assert body["restart"] and body["autostart"]


def test_stop_api_runs_the_helper(client, operator, monkeypatch):
    assert login(client).status_code in (302, 303)
    token = client.get("/api/system/stop").get_json()["csrfToken"]
    ran: list = []
    monkeypatch.setattr("robot_control.app.shutdown.stop_all",
                        lambda root, **_kw: (ran.append(root), {
                            "ok": True, "stopping": True, "message": "останавливаю",
                            "targets": list(shutdown.TARGETS),
                            "log": "var/stop.log"})[1])
    response = client.post("/api/system/stop", json={}, headers={"X-CSRF-Token": token})
    assert response.status_code == 202
    assert response.get_json()["stopping"] is True
    assert ran and Path(ran[0]) == PROJECT_ROOT


def test_stop_api_reports_failure(client, operator, monkeypatch):
    assert login(client).status_code in (302, 303)
    token = client.get("/api/system/stop").get_json()["csrfToken"]
    monkeypatch.setattr("robot_control.app.shutdown.stop_all",
                        lambda *_a, **_k: {"ok": False, "stopping": False,
                                          "error": "нет scripts/stop.sh"})
    response = client.post("/api/system/stop", json={}, headers={"X-CSRF-Token": token})
    assert response.status_code == 500
    assert response.get_json()["stopping"] is False


# ---------------------------------------------------------------------------
# Бортовой экран (8090): входа нет — запрос помечается заголовком
# ---------------------------------------------------------------------------


def test_kiosk_stop_api_needs_the_button_header(kiosk_client, monkeypatch):
    monkeypatch.setattr("robot_control.kiosk_app.shutdown.stop_all",
                        lambda *_a, **_k: pytest.fail("остановка без заголовка от кнопки"))
    assert kiosk_client.post("/api/system/stop", json={}).status_code == 400
    body = kiosk_client.get("/api/system/stop").get_json()
    assert body["ok"] is True and body["canStop"] is True
    assert body["requiresCsrf"] is False


def test_kiosk_stop_api_stops_everything(kiosk_client, monkeypatch):
    ran: list = []
    monkeypatch.setattr("robot_control.kiosk_app.shutdown.stop_all",
                        lambda root, **_k: (ran.append(root), {
                            "ok": True, "stopping": True, "log": "var/stop.log",
                            "targets": list(shutdown.TARGETS)})[1])
    response = kiosk_client.post("/api/system/stop", json={},
                                 headers={"X-Robot-Control": "stop"})
    assert response.status_code == 202
    assert response.get_json()["stopping"] is True
    assert ran and Path(ran[0]) == PROJECT_ROOT


# ---------------------------------------------------------------------------
# Экранные файлы кнопки
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("path", ["/power.js", "/power.css"])
def test_stop_button_assets_are_served_by_both_apps(client, operator, kiosk_client, path):
    assert client.get(path).status_code == 200
    assert kiosk_client.get(path).status_code == 200


@pytest.mark.parametrize("page", ["/", "/console", "/kiosk"])
def test_pages_include_the_stop_button(client, operator, page):
    """Кнопка подключена к страницам и пульта, и бортового экрана."""
    assert login(client).status_code in (302, 303)
    html = client.get(page).get_data(as_text=True)
    assert "power.js" in html and "power.css" in html


def test_stop_button_talks_to_the_stop_api(client):
    """Скрипт кнопки: относительный адрес, CSRF-заголовок и заголовок киоска."""
    js = client.get("/power.js").get_data(as_text=True)
    assert "api/system/stop" in js
    assert "X-CSRF-Token" in js
    assert "X-Robot-Control" in js
    # Второй клик обязателен: остановка не должна срабатывать от одного касания.
    assert "Подтвердить остановку" in js


def test_stop_api_is_documented_in_readme():
    """Про кнопку и её API написано в README — это не «скрытая» кнопка.

    Остановка робота с экрана — действие, которое оператор должен найти и
    понимать сам; инструкция живёт рядом с описанием ``./scripts/stop.sh``.
    """
    text = (PROJECT_ROOT / "README.md").read_text(encoding="utf-8")
    assert "api/system/stop" in text
    assert "Остановить всё" in text
