"""
Тесты пресетов сайтов: готовые группы, свои группы и сохранённые результаты
подбора («домены + стратегия»).
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import presets  # noqa: E402


@pytest.fixture()
def cfg(monkeypatch, tmp_path):
    """Чистый конфиг в отдельной песочнице — файл не трогаем."""
    saved: dict = {}
    monkeypatch.setattr(presets.config_mod, "save", lambda data: saved.update(data))
    data = {"site_groups": [], "site_presets": []}
    data["_saved"] = saved
    return data


# ---------------------------------------------------------------------------
# Группы сайтов
# ---------------------------------------------------------------------------

def test_ready_groups_cover_popular_services(cfg):
    groups = presets.all_groups(cfg)
    keys = {group["key"] for group in groups}
    assert {"youtube", "discord", "telegram"} <= keys
    assert len(groups) == len(presets.checks.SITE_GROUPS)
    youtube = presets.group_by_key(cfg, "youtube")
    assert "youtube.com" in youtube["hosts"]
    assert youtube["custom"] is False


def test_selection_of_several_groups_has_no_duplicates(cfg):
    hosts = presets.hosts_for_selection(cfg, ["youtube", "discord"])
    assert hosts.count("youtube.com") == 1
    assert "discord.com" in hosts
    assert len(hosts) == len(set(hosts))


def test_selection_ignores_unknown_group(cfg):
    assert presets.hosts_for_selection(cfg, ["нет-такой"]) == []


def test_custom_group_is_saved_and_resolved(cfg):
    group = presets.add_group(cfg, "Работа", ["https://wiki.example.com/page",
                                              "git.example.com", "мусор"])
    assert group["key"] == "my:rabota"
    assert group["hosts"] == ["wiki.example.com", "git.example.com"]
    assert group["custom"] is True

    # Группа доступна и по названию, и по ключу
    assert presets.resolve_selection(cfg, "Работа") == ["wiki.example.com",
                                                        "git.example.com"]
    assert presets.resolve_selection(cfg, "rabota") == ["wiki.example.com",
                                                        "git.example.com"]
    # …и вместе с обычными доменами
    assert presets.resolve_selection(cfg, "Работа rutracker.org")[-1] == "rutracker.org"
    assert len(presets.all_groups(cfg)) == len(presets.checks.SITE_GROUPS) + 1


def test_custom_group_updates_by_title(cfg):
    presets.add_group(cfg, "Работа", ["a.example.com"])
    presets.add_group(cfg, "работа", ["b.example.com"])
    groups = presets.custom_groups(cfg)
    assert len(groups) == 1
    assert groups[0]["hosts"] == ["b.example.com"]


def test_group_requires_name_and_hosts(cfg):
    with pytest.raises(ValueError):
        presets.add_group(cfg, "", ["a.example.com"])
    with pytest.raises(ValueError):
        presets.add_group(cfg, "Пустая", ["мусор"])


def test_builtin_group_cannot_be_removed(cfg):
    assert presets.remove_group(cfg, "youtube") is False
    group = presets.add_group(cfg, "Работа", ["a.example.com"])
    assert presets.remove_group(cfg, group["key"]) is True
    assert presets.custom_groups(cfg) == []


def test_group_name_is_suggested_from_domains(cfg):
    assert presets.suggested_preset_name(cfg, [], ["rutracker.org"]) == "rutracker.org"
    assert presets.suggested_preset_name(cfg, [], ["a.ru", "b.ru"]) == "a.ru и ещё 1"
    assert presets.suggested_preset_name(cfg, ["youtube", "discord"], []) == \
        "YouTube + Discord"


# ---------------------------------------------------------------------------
# Пресеты: домены + стратегия
# ---------------------------------------------------------------------------

def test_preset_saves_result(cfg):
    preset = presets.save_preset(cfg, "Видео и чат", ["youtube.com", "discord.com"],
                                 "general_alt2.bat", ok=2, total=2, avg_ms=210.0)
    assert preset["ok"] == 2 and preset["checked_at"] > 0
    assert presets.preset_by_name(cfg, "видео и чат")["strategy"] == "general_alt2.bat"


def test_preset_is_updated_not_duplicated(cfg):
    presets.save_preset(cfg, "Мой", ["a.ru"], "general.bat", ok=1, total=1)
    presets.update_preset(cfg, "Мой", strategy="general_alt.bat", ok=0)
    assert len(presets.presets(cfg)) == 1
    assert presets.presets(cfg)[0]["strategy"] == "general_alt.bat"


def test_broken_preset_goes_last(cfg):
    presets.save_preset(cfg, "Рабочий", ["a.ru"], "general.bat", ok=1, total=1)
    presets.save_preset(cfg, "Сломался", ["b.ru"], "general_alt.bat", ok=0, total=1)
    assert [p["name"] for p in presets.presets(cfg)][0] == "Рабочий"


def test_preset_needs_name_and_hosts(cfg):
    with pytest.raises(ValueError):
        presets.save_preset(cfg, "", ["a.ru"], "general.bat")
    with pytest.raises(ValueError):
        presets.save_preset(cfg, "Пустой", [], "general.bat")


def test_preset_removal_and_unknown_update(cfg):
    presets.save_preset(cfg, "Мой", ["a.ru"], "general.bat")
    assert presets.update_preset(cfg, "чужой", strategy="x") is None
    assert presets.remove_preset(cfg, "Мой") is True
    assert presets.remove_preset(cfg, "Мой") is False
    assert presets.presets(cfg) == []


def test_broken_presets_are_skipped_on_load(cfg):
    cfg["site_presets"] = [{"name": "", "hosts": ["a.ru"]}, "мусор",
                           {"name": "Нормальный", "hosts": ["b.ru"], "ok": "1",
                            "total": "1", "strategy": "general.bat"},
                           {"name": "Без доменов", "hosts": []}]
    items = presets.presets(cfg)
    assert [p["name"] for p in items] == ["Нормальный"]
    assert items[0]["ok"] == 1 and items[0]["total"] == 1
