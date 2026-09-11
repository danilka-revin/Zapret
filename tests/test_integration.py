"""
Тесты системной интеграции: ярлык в меню + на рабочем столе, права NOPASSWD.

Ярлык обязан появляться в домашнем каталоге пользователя рабочего стола —
даже когда команду запускали из-под root (частая причина «иконки нет на столе»).
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import integration, session  # noqa: E402


@pytest.fixture()
def sandbox(tmp_path, monkeypatch):
    """Полностью изолированный «дом» пользователя."""
    home = tmp_path / "tester-home"
    (home / "Desktop").mkdir(parents=True)
    (home / ".config").mkdir(parents=True)
    app = tmp_path / "app"
    (app / "assets").mkdir(parents=True)
    (app / "run.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (app / "assets" / "zapret-control.png").write_bytes(b"\x89PNG fake")

    monkeypatch.setenv("ZAPRET_APP_DIR", str(app))
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("XDG_DATA_HOME", raising=False)
    monkeypatch.setattr(session, "desktop_user", lambda: "tester")
    monkeypatch.setattr(session, "current_user", lambda: "tester")
    monkeypatch.setattr(session, "is_root", lambda: False)
    monkeypatch.setattr(session, "user_home", lambda user: home)
    monkeypatch.setattr(session, "desktop_dir", lambda user=None: home / "Desktop")
    monkeypatch.setattr(integration, "_own_to_owner", lambda path: None)
    return {"home": home, "app": app}


def test_shortcut_lands_in_menu_and_desktop(sandbox):
    placed = integration.install_shortcut()
    menu = Path(placed["menu"])
    desktop = Path(placed["desktop"])
    assert menu.parent == sandbox["home"] / ".local/share/applications"
    assert desktop.parent == sandbox["home"] / "Desktop"
    assert menu.exists() and desktop.exists()


def test_desktop_entry_launches_the_gui(sandbox):
    integration.install_shortcut()
    text = Path(integration.desktop_file_path()).read_text(encoding="utf-8")
    assert "Name=Zapret Control" in text
    assert "run.py gui" in text                       # без аргумента окно не откроется
    assert "Terminal=false" in text
    assert "[Desktop Entry]" in text
    assert "Icon=" in text


def test_desktop_copy_is_executable(sandbox):
    """Без +x и «доверия» GNOME/KDE не дают запустить значок с рабочего стола."""
    placed = integration.install_shortcut()
    mode = Path(placed["desktop"]).stat().st_mode
    assert mode & 0o111, "ярлык на столе должен быть исполняемым"


def test_icon_installed_into_hicolor_theme(sandbox):
    placed = integration.install_shortcut()
    assert placed  # ярлык создан
    icons_root = sandbox["home"] / ".local/share/icons/hicolor"
    found = list(icons_root.glob("*/apps/zapret-control.png"))
    assert found, "иконка должна попасть в пользовательскую тему"
    text = Path(integration.desktop_file_path()).read_text(encoding="utf-8")
    assert "Icon=zapret-control" in text       # имя темы, а не абсолютный путь


def test_remove_shortcut_cleans_both_places(sandbox):
    integration.install_shortcut()
    removed = integration.remove_shortcut()
    assert len(removed) == 2
    assert not Path(integration.desktop_file_path()).exists()
    assert not Path(integration.desktop_icon_path()).exists()


def test_shortcut_status_reports_both_locations(sandbox):
    assert integration.shortcut_status()["menu"] is False
    integration.install_shortcut()
    status = integration.shortcut_status()
    assert status["menu"] is True and status["desktop"] is True
    assert status["desktop_shown"] is True


def test_shortcut_recreated_is_idempotent(sandbox):
    """«Пересоздать ярлык» можно нажимать много раз — содержимое не «плывёт»."""
    integration.install_shortcut()
    first = Path(integration.desktop_file_path()).read_text(encoding="utf-8")
    integration.install_shortcut()
    integration.install_shortcut()
    second = Path(integration.desktop_file_path()).read_text(encoding="utf-8")
    assert first == second == Path(integration.desktop_icon_path()).read_text(encoding="utf-8")


def test_stale_entry_without_exec_is_treated_as_missing(sandbox, tmp_path):
    path = integration.desktop_file_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("[Desktop Entry]\nName=Что-то другое\n", encoding="utf-8")
    assert integration.desktop_entry_ready() is False
    assert integration.shortcut_installed() is False


def test_stale_entry_after_move_from_root_is_recreated(sandbox, tmp_path, monkeypatch):
    """Ярлык из «/root-установки» выглядит целым, но ведёт в несуществующий путь."""
    path = integration.desktop_file_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "[Desktop Entry]\nName=Zapret Control\n"
        "Exec=/usr/bin/python3 /root/.local/share/zapret-control/run.py gui\n",
        encoding="utf-8")
    assert integration.desktop_entry_ready() is False

    monkeypatch.setenv("ZAPRET_APP_DIR", str(sandbox["app"]))
    integration.install_shortcut()
    assert integration.desktop_entry_ready() is True
    assert "/root/" not in path.read_text(encoding="utf-8")


def test_sudoers_targets_the_desktop_user_not_root(sandbox, monkeypatch):
    """Раньше под sudo -i NOPASSWD доставался root — и кнопка продолжала просить пароль."""
    monkeypatch.setattr(integration, "permissions_user", lambda: "tester")
    text = integration._sudoers_text("tester")
    rules = [line for line in text.splitlines() if "NOPASSWD:" in line]
    assert len(rules) >= 6
    assert all(line.startswith("tester ") for line in rules)
    assert not any(line.startswith("root ") for line in rules)
    assert any("/nfqws *" in line for line in rules)


def test_setup_permissions_writes_without_sudo_when_root(sandbox, monkeypatch):
    calls = []

    def fake_run(cmd, **kwargs):
        calls.append((cmd, kwargs))
        return type("R", (), {"returncode": 0})()

    monkeypatch.setattr(session, "is_root", lambda: True)
    monkeypatch.setattr(integration, "which", lambda name: "/usr/bin/" + name)
    import subprocess

    monkeypatch.setattr(subprocess, "run", fake_run)
    integration.setup_permissions("tester")
    assert any("/etc/sudoers.d/zapret-control" in " ".join(cmd) for cmd, _k in calls)
    payload = next((k.get("input") for cmd, k in calls if k.get("input")), "")
    assert "tester ALL=(root) NOPASSWD" in payload
