"""
Пресеты сайтов: готовые группы сервисов и сохранённые подборки доменов вместе
с найденной для них стратегией.

Идея: один раз подобрав стратегию под «YouTube + Discord», пользователь больше
не должен об этом думать — пресет хранит список доменов, рабочую стратегию и
результат последней проверки, поэтому его можно применить или перепроверить
одним нажатием.

Всё лежит в config.json:

    "site_groups":  [{"key": "...", "title": "...", "icon": "...", "hosts": [...]}]
    "site_presets": [{"name": "...", "hosts": [...], "strategy": "general_alt2.bat",
                      "ok": 5, "total": 5, "avg_ms": 210.0, "checked_at": 1712345678}]
"""

from __future__ import annotations

import re
import time
from typing import Iterable

from . import checks, config as config_mod

MAX_PRESETS = 20
MAX_GROUPS = 20
MAX_HOSTS = 24
ICON_FOR_CUSTOM_GROUP = "star"


# ---------------------------------------------------------------------------
# Пользовательские группы сайтов
# ---------------------------------------------------------------------------

def _clean_hosts(hosts: Iterable[str]) -> list[str]:
    result: list[str] = []
    for raw in hosts:
        host = checks.clean_host(str(raw))
        if host and host not in result:
            result.append(host)
    return result[:MAX_HOSTS]


def slugify(title: str) -> str:
    """«Моя группа!» → «moya-gruppa» — ключ для пользовательской группы."""
    table = str.maketrans({"а": "a", "б": "b", "в": "v", "г": "g", "д": "d", "е": "e",
                           "ё": "e", "ж": "zh", "з": "z", "и": "i", "й": "y", "к": "k",
                           "л": "l", "м": "m", "н": "n", "о": "o", "п": "p", "р": "r",
                           "с": "s", "т": "t", "у": "u", "ф": "f", "х": "h", "ц": "c",
                           "ч": "ch", "ш": "sh", "щ": "sch", "ъ": "", "ы": "y", "ь": "",
                           "э": "e", "ю": "yu", "я": "ya"})
    slug = re.sub(r"[^a-z0-9]+", "-", title.lower().translate(table)).strip("-")
    return slug or "group"


def custom_groups(cfg: dict) -> list[dict]:
    """Пользовательские группы из конфига (с проверкой данных)."""
    result: list[dict] = []
    for item in cfg.get("site_groups") or []:
        if not isinstance(item, dict):
            continue
        title = str(item.get("title", "")).strip()
        hosts = _clean_hosts(item.get("hosts") or [])
        if not title or not hosts:
            continue
        key = str(item.get("key") or slugify(title))
        result.append({"key": f"my:{key}", "title": title,
                       "icon": str(item.get("icon") or ICON_FOR_CUSTOM_GROUP),
                       "hosts": hosts, "custom": True})
    return result


def all_groups(cfg: dict) -> list[dict]:
    """Встроенные группы сервисов + группы пользователя."""
    groups = [
        {"key": key, "title": title, "icon": icon, "hosts": list(hosts), "custom": False}
        for key, title, icon, hosts in checks.SITE_GROUPS
    ]
    groups.extend(custom_groups(cfg))
    return groups


def group_by_key(cfg: dict, key: str) -> dict | None:
    return next((g for g in all_groups(cfg) if g["key"] == key), None)


def hosts_for_selection(cfg: dict, keys: Iterable[str]) -> list[str]:
    """Домены выбранных групп (в том числе пользовательских)."""
    hosts: list[str] = []
    for key in keys:
        group = group_by_key(cfg, key)
        if not group:
            continue
        for host in group["hosts"]:
            if host not in hosts:
                hosts.append(host)
    return hosts


def resolve_selection(cfg: dict, query: str) -> list[str]:
    """Разбирает запрос пользователя: группы по ключам/названиям + домены."""
    tokens = [t for t in re.split(r"[\s,;]+", query or "") if t]
    keys: list[str] = []
    rest: list[str] = []
    by_title = {g["title"].lower(): g["key"] for g in all_groups(cfg)}
    known = {g["key"].lower(): g["key"] for g in all_groups(cfg)}
    known.update({g["key"].split("my:")[-1].lower(): g["key"] for g in all_groups(cfg)})
    for token in tokens:
        low = token.lower()
        if low in known:
            keys.append(known[low])
        elif low in by_title:
            keys.append(by_title[low])
        else:
            rest.append(token)
    hosts = hosts_for_selection(cfg, keys) if keys else []
    for host in checks.parse_targets(" ".join(rest)):
        if host not in hosts:
            hosts.append(host)
    return hosts


def add_group(cfg: dict, title: str, hosts: Iterable[str]) -> dict:
    """Сохраняет пользовательскую группу сайтов. Возвращает её описание."""
    title = (title or "").strip()
    clean = _clean_hosts(hosts)
    if not title:
        raise ValueError("Введите название группы, например «Работа».")
    if not clean:
        raise ValueError("В группе нет ни одного домена — проверьте ввод.")

    items = [dict(item) for item in (cfg.get("site_groups") or [])
             if isinstance(item, dict)]
    key = slugify(title)
    for item in items:
        if str(item.get("title", "")).strip().lower() == title.lower() or \
                str(item.get("key", "")) == key:
            item["hosts"] = clean
            item["title"] = title
            cfg["site_groups"] = items
            config_mod.save(cfg)
            return {"key": f"my:{key}", "title": title, "icon": ICON_FOR_CUSTOM_GROUP,
                    "hosts": clean, "custom": True}
    if len(items) >= MAX_GROUPS:
        raise ValueError(f"Уже сохранено {MAX_GROUPS} групп — удалите лишние.")
    items.append({"key": key, "title": title, "icon": ICON_FOR_CUSTOM_GROUP,
                  "hosts": clean})
    cfg["site_groups"] = items
    config_mod.save(cfg)
    return {"key": f"my:{key}", "title": title, "icon": ICON_FOR_CUSTOM_GROUP,
            "hosts": clean, "custom": True}


def remove_group(cfg: dict, key: str) -> bool:
    if not key.startswith("my:"):
        return False
    short = key.split("my:")[-1]
    items = [item for item in (cfg.get("site_groups") or []) if isinstance(item, dict)]
    kept = [item for item in items
            if str(item.get("key", "")) != short
            and slugify(str(item.get("title", ""))) != short]
    if len(kept) == len(items):
        return False
    cfg["site_groups"] = kept
    config_mod.save(cfg)
    return True


# ---------------------------------------------------------------------------
# Пресеты: домены + найденная стратегия
# ---------------------------------------------------------------------------

def _clean_preset(item: dict) -> dict | None:
    if not isinstance(item, dict):
        return None
    name = str(item.get("name", "")).strip()
    hosts = _clean_hosts(item.get("hosts") or [])
    if not name or not hosts:
        return None
    preset = {
        "name": name,
        "hosts": hosts,
        "strategy": str(item.get("strategy", "")),
        "ok": int(item.get("ok") or 0),
        "total": int(item.get("total") or len(hosts)),
        "avg_ms": float(item.get("avg_ms") or 0.0),
        "checked_at": float(item.get("checked_at") or 0.0),
    }
    return preset


def presets(cfg: dict) -> list[dict]:
    """Сохранённые пресеты: сначала проверенные, потом по свежести."""
    result = [p for p in (_clean_preset(item) for item in (cfg.get("site_presets") or []))
              if p]
    result.sort(key=lambda p: (p["ok"] < p["total"], -p["checked_at"]))
    return result


def preset_by_name(cfg: dict, name: str) -> dict | None:
    low = (name or "").strip().lower()
    return next((p for p in presets(cfg) if p["name"].lower() == low), None)


def save_preset(cfg: dict, name: str, hosts: Iterable[str], strategy: str,
                ok: int = 0, total: int = 0, avg_ms: float = 0.0) -> dict:
    """Создаёт или обновляет пресет с результатом подбора."""
    name = (name or "").strip()
    clean = _clean_hosts(hosts)
    if not name:
        raise ValueError("Введите название пресета, например «Видео и чат».")
    if not clean:
        raise ValueError("В пресете нет ни одного домена.")

    items = [dict(item) for item in (cfg.get("site_presets") or [])
             if isinstance(item, dict)]
    preset = {
        "name": name,
        "hosts": clean,
        "strategy": str(strategy or ""),
        "ok": int(ok or 0),
        "total": int(total or len(clean)),
        "avg_ms": float(avg_ms or 0.0),
        "checked_at": time.time(),
    }
    for index, item in enumerate(items):
        if str(item.get("name", "")).strip().lower() == name.lower():
            items[index] = preset
            break
    else:
        if len(items) >= MAX_PRESETS:
            raise ValueError(f"Уже сохранено {MAX_PRESETS} пресетов — удалите лишние.")
        items.append(preset)
    cfg["site_presets"] = items
    config_mod.save(cfg)
    return preset


def update_preset(cfg: dict, name: str, **fields) -> dict | None:
    """Обновляет результат проверки существующего пресета."""
    items = [dict(item) for item in (cfg.get("site_presets") or [])
             if isinstance(item, dict)]
    for index, item in enumerate(items):
        if str(item.get("name", "")).strip().lower() != (name or "").strip().lower():
            continue
        merged = {**item, **fields}
        if "hosts" in fields:
            merged["hosts"] = _clean_hosts(fields["hosts"])
        merged["checked_at"] = fields.get("checked_at", time.time())
        clean = _clean_preset(merged)
        if clean is None:
            return None
        items[index] = merged
        cfg["site_presets"] = items
        config_mod.save(cfg)
        return clean
    return None


def remove_preset(cfg: dict, name: str) -> bool:
    items = [item for item in (cfg.get("site_presets") or []) if isinstance(item, dict)]
    kept = [item for item in items
            if str(item.get("name", "")).strip().lower() != (name or "").strip().lower()]
    if len(kept) == len(items):
        return False
    cfg["site_presets"] = kept
    config_mod.save(cfg)
    return True


def suggested_preset_name(cfg: dict, keys: Iterable[str], hosts: Iterable[str]) -> str:
    """Понятное имя для нового пресета: по группам или по доменам."""
    titles = []
    for key in keys:
        group = group_by_key(cfg, key)
        if group and group["title"] not in titles:
            titles.append(group["title"])
    if titles:
        return " + ".join(titles[:3])
    hosts = list(hosts)
    if not hosts:
        return "Мои сайты"
    if len(hosts) == 1:
        return hosts[0]
    return f"{hosts[0]} и ещё {len(hosts) - 1}"
