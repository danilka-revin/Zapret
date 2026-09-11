"""
Тесты самообновления: проверка версии, обновление кода (git и архив),
зависимости, ярлык, перезапуск.

Сеть и git не вызываются: всё, что стучится наружу, подменяется.
"""

import io
import json
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import core  # noqa: E402
from zapret import update as update_mod  # noqa: E402


@pytest.fixture()
def app(tmp_path, monkeypatch):
    root = tmp_path / "app"
    (root / "zapret").mkdir(parents=True)
    (root / "zapret" / "__init__.py").write_text('APP_VERSION = "2.1.0"\n', encoding="utf-8")
    (root / "run.py").write_text("print('old')\n", encoding="utf-8")
    (root / "deps" / "flowseal").mkdir(parents=True)
    (root / "config.json").write_text('{"strategy": "general.bat"}\n', encoding="utf-8")
    monkeypatch.setenv("ZAPRET_APP_DIR", str(root))
    return root


def _archive(root: Path, version: str = "2.5.0") -> bytes:
    """Собирает tar.gz вида «Zapret-main/...» — как codeload."""
    buffer = io.BytesIO()
    stage = root.parent / "stage"
    top = stage / "Zapret-main"
    (top / "zapret").mkdir(parents=True)
    (top / "run.py").write_text("print('new')\n", encoding="utf-8")
    (top / "zapret" / "__init__.py").write_text('APP_VERSION = "%s"\n' % version, encoding="utf-8")
    (top / "README.md").write_text("новый readme\n", encoding="utf-8")
    with tarfile.open(fileobj=buffer, mode="w:gz") as tf:
        tf.add(top, arcname="Zapret-main")
    return buffer.getvalue()


# ---------------------------------------------------------------------------
# Проверка наличия обновления
# ---------------------------------------------------------------------------

def test_check_update_detects_new_commit(app, monkeypatch):
    monkeypatch.setattr(update_mod, "remote_commit", lambda: "abcdef1234567890")
    monkeypatch.setattr(update_mod, "local_commit", lambda path=None: "12345678")
    result = update_mod.check_update()
    assert result["available"] is True
    assert "abcdef12" in result["message"]


def test_check_update_says_no_when_versions_match(app, monkeypatch):
    monkeypatch.setattr(update_mod, "remote_commit", lambda: "abcdef1234567890")
    monkeypatch.setattr(update_mod, "local_commit", lambda path=None: "abcdef12")
    result = update_mod.check_update()
    assert result["available"] is False
    assert "не требуется" in result["message"]


def test_check_update_without_network_is_not_an_error(app, monkeypatch):
    monkeypatch.setattr(update_mod, "remote_commit", lambda: "")
    result = update_mod.check_update()
    assert result["available"] is None
    assert "репозитори" in result["message"]


def test_check_update_uses_recorded_commit_when_no_git(app, monkeypatch):
    monkeypatch.setattr(update_mod, "remote_commit", lambda: "99887766aa")
    monkeypatch.setattr(update_mod, "local_commit", lambda path=None: "")
    (app / update_mod.STATE_FILE).write_text(json.dumps({"commit": "11111111"}), encoding="utf-8")
    assert update_mod.check_update()["available"] is True
    (app / update_mod.STATE_FILE).write_text(json.dumps({"commit": "99887766aa"}), encoding="utf-8")
    assert update_mod.check_update()["available"] is False


# ---------------------------------------------------------------------------
# Обновление кода
# ---------------------------------------------------------------------------

class FakeGit:
    """Записывает вызовы git и отвечает так, будто есть новая версия."""

    def __init__(self, head="aaaa1111", new="bbbb2222", returncode=0):
        self.calls = []
        self.head = head
        self.new = new
        self.returncode = returncode

    def __call__(self, args, **kwargs):
        self.calls.append(list(args))
        if "fetch" in args:
            out = "" if self.returncode == 0 else "fatal: unable to access origin"
        elif "rev-parse" in args and "FETCH_HEAD" in args:
            out = self.new + "\n"
        elif "rev-parse" in args:
            out = self.head + "\n"
        elif "reset" in args and self.returncode == 0:
            self.head = self.new          # сброс HEAD — как после настоящего reset
            out = "HEAD is now at " + self.new
        else:
            out = "" if self.returncode == 0 else "error: cannot pull with rebase"
        return type("R", (), {"returncode": self.returncode, "stdout": out})()


def test_git_update_fetches_and_resets(app, monkeypatch):
    (app / ".git").mkdir()
    fake = FakeGit()
    monkeypatch.setattr(update_mod, "_git", fake)
    result = update_mod.update_code()
    assert result["source"] == "git"
    assert result["changed"] is True
    assert any("reset" in call for call in fake.calls)
    assert not any("clean" in " ".join(call) for call in fake.calls), \
        "git clean удал бы зависимости пользователя"


def test_git_update_keeps_user_data(app, monkeypatch):
    (app / ".git").mkdir()
    monkeypatch.setattr(update_mod, "_git", FakeGit())
    update_mod.update_code()
    assert (app / "config.json").exists()
    assert (app / "deps" / "flowseal").exists()


def test_git_failure_is_reported_not_raised(app, monkeypatch):
    (app / ".git").mkdir()
    fake = FakeGit(returncode=128)
    monkeypatch.setattr(update_mod, "_git", fake)
    result = update_mod.update_code()
    assert result["error"]
    assert result["changed"] is False


def test_archive_update_copies_code_and_keeps_state(app, monkeypatch):
    payload = _archive(app, version="2.5.0")
    monkeypatch.setattr(update_mod, "_http_get", lambda url, timeout=30: payload)
    monkeypatch.setattr(update_mod, "remote_commit", lambda: "feedface12ab")
    result = update_mod.update_code()
    assert result["source"] == "archive"
    assert result["changed"] is True
    assert result["version_after"] == "2.5.0"
    assert "print('new')" in (app / "run.py").read_text(encoding="utf-8")
    assert (app / "config.json").exists()               # настройки на месте
    assert (app / "deps" / "flowseal").exists()          # зависимости не тронуты
    assert json.loads((app / update_mod.STATE_FILE).read_text())["commit"]


def test_archive_update_survives_broken_download(app, monkeypatch):
    monkeypatch.setattr(update_mod, "remote_commit", lambda: "")

    def boom(url, timeout=30):
        raise OSError("сеть недоступна")

    monkeypatch.setattr(update_mod, "_http_get", boom)
    result = update_mod.update_code()
    assert "сеть недоступна" in result["error"]
    assert "print('old')" in (app / "run.py").read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# Полное обновление «одной кнопкой»
# ---------------------------------------------------------------------------

def test_update_all_runs_every_step(app, monkeypatch):
    calls = []
    monkeypatch.setattr(update_mod, "update_code",
                        lambda log=None: {"changed": True, "source": "git", "error": "",
                                          "commit_before": "a", "commit_after": "b",
                                          "version_before": "2.1.0", "version_after": "2.2.0"})
    from zapret import integration

    monkeypatch.setattr(integration, "install_shortcut",
                        lambda add_to_desktop=True: (calls.append("shortcut")
                                                      or {"menu": "/x", "desktop": "/y"}))
    monkeypatch.setattr(integration, "permissions_ready", lambda: (calls.append("perm-check")
                                                                    or False))
    monkeypatch.setattr(integration, "setup_permissions",
                        lambda user=None: calls.append("permissions:" + str(user)))
    monkeypatch.setattr(integration, "permissions_user", lambda: "tester")
    monkeypatch.setattr(core, "ensure_deps",
                        lambda *a, **k: calls.append("deps"))
    result = update_mod.update_all()
    assert result["ok"] is True
    assert result["restart_required"] is True
    assert "shortcut" in calls and "deps" in calls and "permissions:tester" in calls


def test_update_all_skips_permissions_when_already_ready(app, monkeypatch):
    written = []
    monkeypatch.setattr(update_mod, "update_code",
                        lambda log=None: {"changed": False, "source": "git", "error": "",
                                          "version_before": "2.1.0", "version_after": "2.1.0"})
    from zapret import integration

    monkeypatch.setattr(integration, "install_shortcut", lambda add_to_desktop=True: {})
    monkeypatch.setattr(integration, "permissions_ready", lambda: True)
    monkeypatch.setattr(integration, "setup_permissions", lambda user=None: written.append(user))
    monkeypatch.setattr(core, "ensure_deps", lambda *a, **k: None)
    result = update_mod.update_all()
    assert written == []
    assert result["steps"]["permissions"].get("already") is True


def test_deps_failure_does_not_break_update(app, monkeypatch):
    monkeypatch.setattr(update_mod, "update_code",
                        lambda log=None: {"changed": True, "source": "git", "error": "",
                                          "version_before": "2.1.0", "version_after": "2.2.0"})
    from zapret import integration

    monkeypatch.setattr(integration, "install_shortcut", lambda add_to_desktop=True: {})
    monkeypatch.setattr(integration, "permissions_ready", lambda: True)

    def broken(*a, **k):
        raise RuntimeError("GitHub недоступен")

    monkeypatch.setattr(core, "ensure_deps", broken)
    result = update_mod.update_all()
    assert result["ok"] is True
    assert "deps" in result["failed_steps"]
    assert any("GitHub" in message for message in result["messages"])


# ---------------------------------------------------------------------------
# Перезапуск
# ---------------------------------------------------------------------------

def test_relaunch_spawns_detached_waiter(app, monkeypatch):
    spawned = {}
    monkeypatch.setattr(update_mod.session, "session_env", lambda user=None: {})
    monkeypatch.setattr(update_mod.session, "running_gui_pids", lambda: [])

    def fake_popen(command, **kwargs):
        spawned["command"] = command
        spawned["kwargs"] = kwargs
        return type("P", (), {"pid": 777})()

    monkeypatch.setattr(subprocess, "Popen", fake_popen)
    result = update_mod.relaunch()
    assert result["ok"] is True
    assert "relaunch" in spawned["command"]
    assert "--pid" in spawned["command"]
    assert spawned["kwargs"].get("start_new_session") is True


def test_relaunch_does_not_open_second_window(app, monkeypatch):
    """Если интерфейс уже запущен, второго окна быть не должно — об этом сказано прямо."""
    monkeypatch.setattr(update_mod.session, "running_gui_pids", lambda: [4242])
    called = []
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **k: called.append(a))
    result = update_mod.relaunch()
    assert result["ok"] is False and not called
    assert "4242" in result["error"]


def test_relaunch_error_is_visible(app, monkeypatch):
    monkeypatch.setattr(update_mod.session, "running_gui_pids", lambda: [])

    def boom(*a, **k):
        raise OSError("нет прав")

    monkeypatch.setattr(subprocess, "Popen", boom)
    result = update_mod.relaunch()
    assert result["ok"] is False
    assert "нет прав" in result["error"]


def test_code_version_reads_installed_file(app):
    assert update_mod.code_version(app) == "2.1.0"
    (app / "zapret" / "__init__.py").write_text('APP_VERSION = "3.0.0"\n', encoding="utf-8")
    assert update_mod.code_version(app) == "3.0.0"
