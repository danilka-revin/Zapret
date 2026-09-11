"""
Тесты определения пользователя рабочего стола и его окружения.

Это тот слой, который ломался при «установке через sudo -i»: приложение,
ярлык и права уходили в /root, а окно не могло подключиться к сессии.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import session  # noqa: E402


@pytest.fixture(autouse=True)
def _clear_cache():
    session.desktop_user.cache_clear()
    yield
    session.desktop_user.cache_clear()


def test_current_user_falls_back_to_env(monkeypatch):
    monkeypatch.setattr(session.pwd, "getpwuid",
                        lambda uid: (_ for _ in ()).throw(KeyError("no passwd")))
    monkeypatch.setenv("USER", "tester")
    assert session.current_user() == "tester"


def test_desktop_user_prefers_sudo_user(monkeypatch, tmp_path):
    monkeypatch.setattr(session, "is_root", lambda: True)
    monkeypatch.setattr(session, "current_user", lambda: "root")
    monkeypatch.setenv("SUDO_USER", "tester")
    monkeypatch.setattr(session, "_looks_like_human", lambda name: name == "tester")
    assert session.desktop_user() == "tester"


def test_desktop_user_follows_sudo_uid(monkeypatch):
    monkeypatch.setattr(session, "is_root", lambda: True)
    monkeypatch.setattr(session, "current_user", lambda: "root")
    monkeypatch.delenv("SUDO_USER", raising=False)
    monkeypatch.setenv("SUDO_UID", "4242")
    monkeypatch.setattr(session, "passwd_user", lambda uid: "mate" if uid == 4242 else "")
    monkeypatch.setattr(session, "_looks_like_human", lambda name: name == "mate")
    assert session.desktop_user() == "mate"


def test_desktop_user_is_self_for_regular_user(monkeypatch):
    monkeypatch.setattr(session, "is_root", lambda: False)
    monkeypatch.setattr(session, "current_user", lambda: "tester")
    monkeypatch.setenv("SUDO_USER", "someone-else")
    assert session.desktop_user() == "tester"


def test_human_user_validation_rejects_service_accounts(monkeypatch):
    class Entry:
        def __init__(self, name, uid, home, shell):
            self.pw_name, self.pw_uid, self.pw_dir, self.pw_shell = name, uid, home, shell

    entries = {
        "tester": Entry("tester", 1000, "/home/tester", "/bin/bash"),
        "nologin": Entry("nologin", 1001, "/var/lib/x", "/usr/sbin/nologin"),
        "system": Entry("system", 500, "/srv", "/bin/sh"),
    }
    monkeypatch.setattr(session.pwd, "getpwnam", lambda name: entries[name])
    assert session._looks_like_human("tester") is True
    assert session._looks_like_human("nologin") is False
    assert session._looks_like_human("system") is False
    assert session._looks_like_human("root") is False
    assert session._looks_like_human("") is False


def test_session_env_for_same_user_keeps_environment(monkeypatch):
    monkeypatch.setattr(session, "is_root", lambda: False)
    monkeypatch.setattr(session, "current_user", lambda: "tester")
    monkeypatch.setattr(session, "desktop_user", lambda: "tester")
    monkeypatch.setenv("DISPLAY", ":1")
    assert session.session_env()["DISPLAY"] == ":1"


def test_session_env_for_root_uses_target_user_paths(monkeypatch):
    monkeypatch.setattr(session, "is_root", lambda: True)
    monkeypatch.setattr(session, "current_user", lambda: "root")
    monkeypatch.setattr(session, "desktop_user", lambda: "tester")
    monkeypatch.setattr(session, "uid_gid", lambda user: (1500, 1500))
    monkeypatch.setattr(session, "user_home", lambda user: Path("/home/tester"))
    monkeypatch.setattr(session, "user_info", lambda user: None)
    monkeypatch.setattr(session, "_pids_of", lambda user: [])
    monkeypatch.setattr(session, "has_session_process", lambda user=None: True)
    monkeypatch.setattr(Path, "is_dir", lambda self: False)

    env = session.session_env("tester")
    assert env["HOME"] == "/home/tester"
    assert env["USER"] == "tester" and env["LOGNAME"] == "tester"
    assert env["XDG_RUNTIME_DIR"] == "/run/user/1500"
    assert env["DISPLAY"] == ":0"          # сессия есть, а переменные потеряны
    assert "/home/tester/.local/bin" in env["PATH"]


def test_session_env_does_not_invent_display_without_session(monkeypatch):
    monkeypatch.setattr(session, "is_root", lambda: True)
    monkeypatch.setattr(session, "current_user", lambda: "root")
    monkeypatch.setattr(session, "uid_gid", lambda user: (1500, 1500))
    monkeypatch.setattr(session, "user_home", lambda user: Path("/home/tester"))
    monkeypatch.setattr(session, "user_info", lambda user: None)
    monkeypatch.setattr(session, "_pids_of", lambda user: [])
    monkeypatch.setattr(session, "has_session_process", lambda user=None: False)
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)

    env = session.session_env("tester")
    assert "DISPLAY" not in env
    assert session.has_display(env) is False


def test_xdg_user_dir_reads_config_and_falls_back(monkeypatch, tmp_path):
    home = tmp_path / "tester"
    (home / ".config").mkdir(parents=True)
    (home / ".config" / "user-dirs.dirs").write_text(
        'XDG_DESKTOP_DIR="$HOME/Рабочий стол"\nXDG_DOWNLOAD_DIR="$HOME/Загрузки"\n',
        encoding="utf-8")
    monkeypatch.setattr(session, "user_home", lambda user: home)

    assert session.xdg_user_dir("tester", "DESKTOP") == home / "Рабочий стол"
    assert session.xdg_user_dir("tester", "DOWNLOAD") == home / "Загрузки"
    assert session.xdg_user_dir("tester", "MUSIC") is None


def test_desktop_dir_prefers_existing_folder(monkeypatch, tmp_path):
    home = tmp_path / "tester"
    (home / "Рабочий стол").mkdir(parents=True)
    monkeypatch.setattr(session, "user_home", lambda user: home)
    monkeypatch.setattr(session, "desktop_user", lambda: "tester")
    assert session.desktop_dir() == home / "Рабочий стол"

    rmdir = home / "Рабочий стол"
    rmdir.rmdir()
    assert session.desktop_dir() == home / "Desktop"      # предлагается, а не выдумывается


def test_relevant_filter_keeps_only_session_variables():
    source = {"DISPLAY": ":0", "XAUTHORITY": "/run/user/1/x", "SECRET_TOKEN": "nope",
              "XDG_RUNTIME_DIR": "/run/user/1", "PATH": "/bin", "LANG": "ru_RU.UTF-8"}
    kept = session._relevant(source)
    assert kept["DISPLAY"] == ":0" and kept["XDG_RUNTIME_DIR"] == "/run/user/1"
    assert kept["LANG"] == "ru_RU.UTF-8"
    assert "SECRET_TOKEN" not in kept and "PATH" not in kept


def test_launch_gui_without_display_reports_honestly(monkeypatch, tmp_path):
    (tmp_path / "run.py").write_text("print('gui')", encoding="utf-8")
    monkeypatch.setattr(session, "desktop_user", lambda: "tester")
    monkeypatch.setattr(session, "session_env", lambda user=None: {})
    result = session.launch_gui(tmp_path)
    assert result["ok"] is False
    assert "Графическая сессия" in result["message"]


def test_launch_gui_checks_that_process_survives(monkeypatch, tmp_path):
    (tmp_path / "run.py").write_text("print('gui')", encoding="utf-8")
    monkeypatch.setattr(session, "desktop_user", lambda: "tester")
    monkeypatch.setattr(session, "has_display", lambda env=None: True)
    monkeypatch.setattr(session, "run_as_user",
                        lambda *a, **k: type("R", (), {"returncode": 0, "stdout": "QT_OK"})())

    launched = {"count": 0}

    class DeadProcess:
        pid = 4321

        def poll(self):
            launched["count"] += 1
            return 1 if launched["count"] > 1 else None

    monkeypatch.setattr(session, "popen_as", lambda *a, **k: DeadProcess())
    (tmp_path / "zapret-control.log").write_text("Traceback: boom\n", encoding="utf-8")

    result = session.launch_gui(tmp_path, settle=0.05)
    assert result["ok"] is False
    assert result["pid"] == 4321
    assert "boom" in result["log"]           # пользователь увидит причину в логе


def test_launch_gui_success_when_process_alive(monkeypatch, tmp_path):
    (tmp_path / "run.py").write_text("print('gui')", encoding="utf-8")
    monkeypatch.setattr(session, "desktop_user", lambda: "tester")
    monkeypatch.setattr(session, "has_display", lambda env=None: True)
    monkeypatch.setattr(session, "run_as_user",
                        lambda *a, **k: type("R", (), {"returncode": 0, "stdout": "QT_OK"})())

    class AliveProcess:
        pid = 99

        def poll(self):
            return None

    monkeypatch.setattr(session, "popen_as", lambda *a, **k: AliveProcess())
    result = session.launch_gui(tmp_path, settle=0.05)
    assert result["ok"] is True and result["pid"] == 99


def test_migrate_root_install_moves_only_heavy_stuff(tmp_path):
    source = tmp_path / "root"
    destination = tmp_path / "user"
    (source / "deps" / "flowseal").mkdir(parents=True)
    (source / "deps" / "flowseal" / "general.bat").write_text("nfqws", encoding="utf-8")
    (source / "nfqws").write_text("binary", encoding="utf-8")
    (source / "config.json").write_text("{}", encoding="utf-8")
    (source / "run.py").write_text("code", encoding="utf-8")

    moved = session.migrate_root_install(source, destination)
    assert set(moved) == {"nfqws", "deps", "config.json"}
    assert (destination / "deps" / "flowseal" / "general.bat").exists()
    assert (source / "run.py").exists()          # код переносит установщик, не мы


def test_root_install_leftovers_only_for_root(monkeypatch, tmp_path):
    base = tmp_path / "root-share"
    (base / "zapret-control").mkdir(parents=True)

    monkeypatch.setattr(session, "is_root", lambda: False)
    assert session.root_install_leftovers(base) is None

    monkeypatch.setattr(session, "is_root", lambda: True)
    assert session.root_install_leftovers(base) is None          # пустого каталога мало
    (base / "zapret-control" / "run.py").write_text("", encoding="utf-8")
    assert session.root_install_leftovers(base) == base / "zapret-control"


def test_chown_tree_skips_when_not_root(monkeypatch, tmp_path):
    target = tmp_path / "app"
    target.mkdir()
    monkeypatch.setattr(session, "is_root", lambda: False)
    monkeypatch.setattr(session, "uid_gid", lambda user: (4321, 4321))
    assert session.chown_tree(target, "tester") is False
    assert session.chown_tree(tmp_path / "missing", "tester") is False


def test_qt_platform_fallbacks_respect_explicit_choice():
    """Явно заданная платформа идёт первой, остальные — запасные."""
    assert session.qt_platform_fallbacks({"QT_QPA_PLATFORM": "xcb"})[0] == "xcb"
    assert "wayland" in session.qt_platform_fallbacks(
        {"QT_QPA_PLATFORM": "xcb", "WAYLAND_DISPLAY": "wayland-0"})


def test_qt_platform_fallbacks_default_order_wayland_session():
    """Без явного выбора: авто → xcb (XWayland) → wayland."""
    assert session.qt_platform_fallbacks(
        {"WAYLAND_DISPLAY": "wayland-0"}) == ["", "xcb", "wayland"]


def test_qt_platform_fallbacks_default_order_x11_session():
    """В чисто X11-сессии wayland-кандидата нет (не к чему подключаться)."""
    assert session.qt_platform_fallbacks({"DISPLAY": ":0"}) == ["", "xcb"]


def test_qt_platform_fallbacks_test_platforms_not_rotated():
    """Тестовые платформы не перебираем — иначе CI запускал бы не то."""
    assert session.qt_platform_fallbacks({"QT_QPA_PLATFORM": "offscreen"}) == ["offscreen"]
