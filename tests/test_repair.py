"""
Тесты починки установки (`run.py repair`).

Сценарий, из-за которого всё это появилось: установили через `sudo -i`,
приложение и ярлык остались в /root, на столе ничего нет, окно не показывается.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import core, repair, session  # noqa: E402


@pytest.fixture(autouse=True)
def _restore_app_dir(monkeypatch):
    """repair меняет ZAPRET_APP_DIR — не даём этому утечь в другие тесты."""
    yield
    monkeypatch.delenv("ZAPRET_APP_DIR", raising=False)


@pytest.fixture()
def world(tmp_path, monkeypatch):
    """Корневая копия + домашний каталог пользователя, всё во временных папках."""
    root_copy = tmp_path / "root-share" / "zapret-control"
    (root_copy / "deps" / "flowseal").mkdir(parents=True)
    (root_copy / "nfqws").write_text("binary", encoding="utf-8")
    (root_copy / "config.json").write_text('{"strategy": "general.bat"}', encoding="utf-8")
    for name in ("run.py", "install.sh", "README.md"):
        (root_copy / name).write_text(name + "\n", encoding="utf-8")
    (root_copy / "zapret").mkdir()
    (root_copy / "zapret" / "__init__.py").write_text('APP_VERSION = "2.1.0"\n', encoding="utf-8")

    home = tmp_path / "home" / "tester"
    home.mkdir(parents=True)
    monkeypatch.setattr(session, "is_root", lambda: True)
    monkeypatch.setattr(session, "desktop_user", lambda: "tester")
    monkeypatch.setattr(session, "user_home", lambda user: home)
    monkeypatch.setattr(session, "uid_gid", lambda user: (1500, 1500))
    monkeypatch.setattr(session, "chown_tree", lambda path, user: True)
    monkeypatch.setattr(session, "root_install_leftovers",
                        lambda base=None: root_copy if root_copy.exists() else None)
    monkeypatch.setattr(repair, "session", session)
    return {"root_copy": root_copy, "home": home, "app": home / ".local/share/zapret-control"}


def test_relocate_moves_data_and_code(world, monkeypatch):
    result = repair.relocate_root_install()
    assert result["moved"] is True
    app = world["app"]
    assert (app / "nfqws").exists()
    assert (app / "deps" / "flowseal").is_dir()
    assert (app / "config.json").exists()
    assert (app / "zapret" / "__init__.py").exists()      # код тоже переехал
    assert not world["root_copy"].exists()                  # в /root ничего не осталось
    import os

    assert os.environ["ZAPRET_APP_DIR"] == str(app)   # дальнейшие шаги — про новый каталог


def test_relocate_noop_for_regular_user(world, monkeypatch):
    monkeypatch.setattr(session, "is_root", lambda: False)
    assert repair.relocate_root_install()["moved"] is False


def test_ownership_problem_is_explained_and_actionable(world, monkeypatch):
    calls = []
    monkeypatch.setattr(session, "chown_tree",
                        lambda path, user: calls.append((str(path), user)) or False)
    result = repair.check_ownership()
    assert calls, "нужно пытаться починить права"
    assert "sudo chown -R tester" in result["problem"]


def test_ownership_ok_when_owner_matches(world, monkeypatch):
    import os

    app = world["app"]
    app.mkdir(parents=True)
    monkeypatch.setattr(session, "uid_gid", lambda user: (os.getuid(), os.getuid()))
    assert repair.check_ownership().get("problem", "") == ""


def test_ensure_data_skips_download_when_ready(world, monkeypatch):
    monkeypatch.setattr(core, "deps_ready", lambda: True)
    monkeypatch.setattr(core, "ensure_deps",
                        lambda *a, **k: pytest.fail("скачивать не нужно"))
    assert repair.ensure_data()["ready"] is True


def test_ensure_data_downloads_when_missing(world, monkeypatch):
    calls = []
    monkeypatch.setattr(core, "deps_ready", lambda: False)
    monkeypatch.setattr(core, "ensure_deps", lambda *a, **k: calls.append(a))
    assert repair.ensure_data()["ready"] is True
    assert calls


def test_ensure_data_reports_network_failure(world, monkeypatch):
    def broken(*a, **k):
        raise RuntimeError("GitHub недоступен")

    monkeypatch.setattr(core, "deps_ready", lambda: False)
    monkeypatch.setattr(core, "ensure_deps", broken)
    result = repair.ensure_data()
    assert result["ok"] is False
    assert "GitHub" in result["error"]


def test_repair_runs_steps_in_order(world, monkeypatch):
    order = []
    from zapret import integration

    monkeypatch.setattr(repair, "relocate_root_install",
                        lambda log=None: order.append("relocate") or {"moved": True, "notes": []})
    monkeypatch.setattr(repair, "check_ownership",
                        lambda log=None: order.append("ownership") or {"problem": ""})
    monkeypatch.setattr(repair, "ensure_data",
                        lambda log=None: order.append("data") or {"ok": True, "ready": True})
    monkeypatch.setattr(integration, "install_shortcut",
                        lambda add_to_desktop=True: (order.append("shortcut")
                                                      or {"menu": "/x", "desktop": "/y"}))
    monkeypatch.setattr(integration, "shortcut_status",
                        lambda: {"menu": True, "menu_path": "/x", "desktop": True,
                                 "desktop_path": "/y", "desktop_shown": True})
    monkeypatch.setattr(integration, "permissions_ready", lambda: (order.append("perm") or True))
    result = repair.repair(launch=False, with_data=True)
    assert order == ["relocate", "ownership", "data", "shortcut", "perm"]
    assert result["ok"] is True
    text = repair.summary_text(result)
    assert "перенос из /root: готово" in text
    assert "ярлык: меню — есть, рабочий стол — есть" in text
