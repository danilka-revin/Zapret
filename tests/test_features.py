"""
Тесты функций v3.0.0: пинг, скорость, DNS, параллельные проверки, стратегии,
история версий, сессии — всё без Qt и без сети (моки).
"""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import checks, config as config_mod, core, integration, session, update  # noqa: E402


# ---------------------------------------------------------------------------
# Версии и чейнджлог
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(("left", "right", "expected"), [
    ("3.0.0", "2.9.0", 1),
    ("2.9.0", "3.0.0", -1),
    ("3.0.0", "3.0.0", 0),
    ("3.0", "3.0.0", 0),
    ("v3.1", "3.0.9", 1),
    ("мусор", "0.0.0", 0),
    ("", "", 0),
    ("10.0.0", "9.9.9", 1),
])
def test_compare_versions(left, right, expected):
    assert update.compare_versions(left, right) == expected


def test_fetch_changelog_parses_releases(monkeypatch):
    payload = [
        {"tag_name": "v3.0.0", "name": "Аврора", "body": "Новое",
         "html_url": "https://example.com/r1"},
        {"tag_name": "v2.9.0", "name": "", "body": "", "html_url": ""},
        "мусор",
    ]
    monkeypatch.setattr(update, "_http_get",
                        lambda _url, timeout=20: json.dumps(payload).encode())
    entries = update.fetch_changelog(limit=5)
    assert len(entries) == 2
    assert entries[0]["tag"] == "v3.0.0"
    assert entries[0]["name"] == "Аврора"
    assert entries[1]["name"] == "v2.9.0"  # пустое имя = тег


def test_fetch_changelog_network_error_is_empty(monkeypatch):
    def boom(_url, timeout=20):
        raise OSError("нет сети")
    monkeypatch.setattr(update, "_http_get", boom)
    assert update.fetch_changelog() == []


# ---------------------------------------------------------------------------
# Пинг, DNS, скорость
# ---------------------------------------------------------------------------

PING_OK = """PING 1.1.1.1 (1.1.1.1) 56(84) bytes of data.

--- 1.1.1.1 ping statistics ---
3 packets transmitted, 3 received, 0% packet loss, time 2003ms
rtt min/avg/max/mdev = 12.100/15.400/18.200/2.100 ms
"""


def test_ping_parses_linux_output(monkeypatch):
    monkeypatch.setattr(checks.shutil, "which", lambda _name: "/bin/ping")

    class Proc:
        stdout = PING_OK

    monkeypatch.setattr(checks.subprocess, "run", lambda *a, **k: Proc())
    result = checks.ping("1.1.1.1")
    assert result["ok"] is True
    assert result["avg_ms"] == pytest.approx(15.4)
    assert result["loss"] == 0.0
    assert "мс" in result["detail"]


def test_ping_total_loss(monkeypatch):
    monkeypatch.setattr(checks.shutil, "which", lambda _name: "/bin/ping")

    class Proc:
        stdout = "3 packets transmitted, 0 received, 100% packet loss"

    monkeypatch.setattr(checks.subprocess, "run", lambda *a, **k: Proc())
    result = checks.ping("10.255.255.1")
    assert result["ok"] is False
    assert result["loss"] == 100.0


def test_ping_without_binary(monkeypatch):
    monkeypatch.setattr(checks.shutil, "which", lambda _name: None)
    result = checks.ping("1.1.1.1")
    assert result["ok"] is False
    assert "не найдена" in result["detail"]


def test_dns_check_ok(monkeypatch):
    import socket as _socket

    monkeypatch.setattr(_socket, "setdefaulttimeout", lambda _t: None)
    monkeypatch.setattr(_socket, "getaddrinfo",
                        lambda *a, **k: [(None, None, None, None, ("1.2.3.4", 443))])
    result = checks.dns_check("example.com")
    assert result["ok"] is True
    assert "1.2.3.4" in result["detail"]


def test_dns_check_failure(monkeypatch):
    import socket as _socket

    monkeypatch.setattr(_socket, "setdefaulttimeout", lambda _t: None)

    def boom(*a, **k):
        raise _socket.gaierror("nope")
    monkeypatch.setattr(_socket, "getaddrinfo", boom)
    result = checks.dns_check("example.invalid")
    assert result["ok"] is False


def test_speedtest_measures_fake_download(monkeypatch):
    chunks = [b"x" * (256 * 1024)] * 4 + [b""]

    class Resp:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def read(self, _n):
            return chunks.pop(0) if chunks else b""

    monkeypatch.setattr(checks.urlrequest, "urlopen", lambda *a, **k: Resp())
    result = checks.speedtest(max_bytes=8_000_000)
    assert result["ok"] is True
    assert result["mbps"] > 0
    assert "Мбит/с" in result["detail"]


def test_speedtest_all_servers_down(monkeypatch):
    def boom(*a, **k):
        raise OSError("dns")
    monkeypatch.setattr(checks.urlrequest, "urlopen", boom)
    result = checks.speedtest()
    assert result["ok"] is False
    assert result["mbps"] == 0.0


# ---------------------------------------------------------------------------
# Параллельные проверки
# ---------------------------------------------------------------------------

def test_probe_all_parallel_keeps_order(monkeypatch):
    calls = []

    def fake(service, timeout=6.0):
        calls.append(service.key)
        return {"key": service.key, "ok": True}

    monkeypatch.setattr(checks, "probe_service", fake)
    results = checks.probe_all_parallel()
    assert [r["key"] for r in results] == [s.key for s in checks.SERVICES]
    assert sorted(calls) == sorted(s.key for s in checks.SERVICES)


def test_probe_hosts_parallel_matches_sequential(monkeypatch):
    monkeypatch.setattr(checks, "probe_host",
                        lambda host, timeout=6.0, attempts=2: {"host": host, "ok": True})
    hosts = ["a.example", "b.example", "c.example", "d.example"]
    results = checks.probe_hosts_parallel(hosts)
    assert [r["host"] for r in results] == hosts


def test_site_groups_cover_expected_services():
    assert len(checks.SITE_GROUPS) >= 20
    titles = " ".join(group[1] for group in checks.SITE_GROUPS)
    for word in ("YouTube", "Discord", "Telegram"):
        assert word in titles


# ---------------------------------------------------------------------------
# Стратегии: список, превью, сводка
# ---------------------------------------------------------------------------

SAMPLE_BAT = (
    "@echo off\r\n"
    "set ARGS=--wf-tcp=80,443 --wf-udp=443,50000-50099 ^\r\n"
    "--dpi-desync=fake --dpi-desync-ttl=4 --new ^\r\n"
    "--filter-tcp=443 --dpi-desync=fake,multisplit ^\r\n"
    "--filter-udp=443 --dpi-desync=fake\r\n"
)


@pytest.fixture()
def strategies(tmp_path, monkeypatch):
    (tmp_path / "general.bat").write_text(SAMPLE_BAT)
    (tmp_path / "general_alt.bat").write_text(SAMPLE_BAT)
    (tmp_path / "discord.bat").write_text(SAMPLE_BAT)
    (tmp_path / "telegram.bat").write_text(SAMPLE_BAT)
    (tmp_path / "notes.txt").write_text("не стратегия")
    monkeypatch.setattr(core, "strategies_dir", lambda: tmp_path)
    return tmp_path


def test_list_strategies_filters_and_orders(strategies):
    names = core.list_strategies()
    assert names[0] == "general.bat"
    assert "general_alt.bat" in names
    assert "notes.txt" not in names


def test_list_strategies_missing_dir(monkeypatch, tmp_path):
    monkeypatch.setattr(core, "strategies_dir", lambda: tmp_path / "нет-такой")
    assert core.list_strategies() == []


def test_strategy_preview_starts_with_bat(strategies):
    preview = core.strategy_preview("general.bat")
    assert preview.startswith("@echo off")
    assert "dpi-desync" in preview


def test_strategy_preview_missing(strategies):
    assert "не найдена" in core.strategy_preview("нет-такой.bat")


def test_strategy_info_extracts_ports(strategies):
    info = core.strategy_info("general_alt.bat")
    assert info["exists"] is True
    assert info["size"] > 0
    assert "443" in (info["tcp"] + info["udp"])
    assert info["filters"] >= 1


def test_strategy_info_missing(strategies):
    assert core.strategy_info("нет-такой.bat")["exists"] is False


# ---------------------------------------------------------------------------
# Интеграция: экранирование, автозапуск
# ---------------------------------------------------------------------------

def test_exec_command_quotes_paths(monkeypatch, tmp_path):
    spaced = tmp_path / "my apps"
    spaced.mkdir()
    monkeypatch.setattr(integration, "run_py_path", lambda: spaced / "run.py")
    monkeypatch.setattr(integration.shutil, "which", lambda _n: "/usr/bin/python3")
    command = integration._exec_command()
    assert command.endswith(" gui")
    assert "'/usr/bin/python3'" not in command  # без пробелов — без кавычек
    assert "'" in command  # путь с пробелом — в кавычках


def test_user_autostart_path_under_config_home(monkeypatch, tmp_path):
    monkeypatch.setattr(integration, "user_home", lambda: tmp_path)
    path = integration.user_autostart_path()
    assert str(path).startswith(str(tmp_path / ".config" / "autostart"))
    assert path.suffix == ".desktop"


def test_user_autostart_install_remove_roundtrip(monkeypatch, tmp_path):
    monkeypatch.setattr(integration, "user_home", lambda: tmp_path)
    monkeypatch.setattr(integration, "install_icon", lambda: "icon.png")
    where = integration.install_user_autostart()
    assert integration.user_autostart_installed() is True
    integration.remove_user_autostart()
    assert integration.user_autostart_installed() is False
    assert where.endswith(".desktop")


# ---------------------------------------------------------------------------
# Сессия: wayland / flatpak / snap
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(("env", "expected"), [
    ({"XDG_SESSION_TYPE": "wayland"}, True),
    ({"XDG_SESSION_TYPE": "x11"}, False),
    ({"WAYLAND_DISPLAY": "wayland-0"}, True),
    ({"DISPLAY": ":0"}, False),
    ({"WAYLAND_DISPLAY": "wayland-0", "DISPLAY": ":0"}, False),
    ({}, False),
])
def test_is_wayland(env, expected):
    assert session.is_wayland(env) is expected


def test_is_snap_via_env(monkeypatch):
    monkeypatch.setenv("SNAP", "/snap/zapret/x1")
    assert session.is_snap() is True
    monkeypatch.delenv("SNAP")
    monkeypatch.delenv("SNAP_NAME", raising=False)
    assert session.is_snap() is False


def test_is_flatpak_via_env(monkeypatch):
    monkeypatch.setenv("FLATPAK_ID", "org.example.App")
    assert session.is_flatpak() is True
    monkeypatch.delenv("FLATPAK_ID")
    # в песочнице файла /.flatpak-info нет
    assert session.is_flatpak() is False


# ---------------------------------------------------------------------------
# Конфиг: новые ключи по умолчанию
# ---------------------------------------------------------------------------

def test_config_defaults_have_new_keys():
    for key in ("profiles", "onboarded", "auto_update_check"):
        assert key in config_mod.DEFAULTS
    assert config_mod.DEFAULTS["profiles"] == {}
    assert config_mod.DEFAULTS["onboarded"] is False
    assert config_mod.DEFAULTS["auto_update_check"] is True


def test_config_load_merges_new_defaults(monkeypatch, tmp_path):
    monkeypatch.setattr(config_mod, "config_path", lambda: tmp_path / "config.json")
    data = config_mod.load()
    assert data["profiles"] == {}
    assert data["onboarded"] is False
