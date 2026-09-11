"""
Тесты темы и кастомизации: цвета, контраст, сохранение настроек.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret.qt.theme import (ACCENTS, PRESETS, Palette, UISettings, contrast_text,  # noqa: E402
                             luminance, mix, normalize_hex)


def test_normalize_hex_accepts_short_and_long():
    assert normalize_hex("#ABC") == "#aabbcc"
    assert normalize_hex("#112233") == "#112233"
    assert normalize_hex("мусор", "#000000") == "#000000"
    assert normalize_hex(None, "#111111") == "#111111"


def test_mix_and_luminance():
    assert mix("#000000", "#ffffff", 0.5) == "#808080"
    assert luminance("#ffffff") > luminance("#000000")
    assert contrast_text("#ffffff") == "#101510"
    assert contrast_text("#000000") == "#ffffff"


def test_settings_sanitized_fixes_broken_values():
    settings = UISettings.from_dict({
        "mode": "странный", "accent": "нет", "glass": "???", "density": "x",
        "radius": "y", "motion": "z", "font_scale": "w", "orbs_intensity": 9999,
        "check_interval": 1, "watch_services": [],
    })
    assert settings.mode == "dark"
    assert settings.accent == "#d5ff45"
    assert settings.glass == "soft"
    assert settings.orbs_intensity == 160
    assert settings.check_interval == 20
    assert settings.watch_services == ["youtube", "discord", "telegram"]


def test_palette_light_and_dark_differ():
    dark = Palette(UISettings(mode="dark", accent="#d5ff45"))
    light = Palette(UISettings(mode="light", accent="#d5ff45"))
    assert dark.dark is True and light.dark is False
    assert dark.text != light.text
    # в светлой теме акцент для текста темнее исходного — иначе не читается
    assert light.accent_text != light.accent
    assert luminance(light.accent_text) < luminance(light.accent)


def test_palette_metrics_follow_density_and_font_scale():
    comfortable = Palette(UISettings(density="comfortable", font_scale="normal"))
    compact = Palette(UISettings(density="compact", font_scale="small"))
    assert compact.pad < comfortable.pad
    assert compact.font_md < comfortable.font_md
    square = Palette(UISettings(radius="square"))
    soft = Palette(UISettings(radius="soft"))
    assert square.r_card < soft.r_card


def test_animations_disabled_with_motion_off():
    assert Palette(UISettings(motion="off")).anim_ms == 0
    assert Palette(UISettings(motion="off")).animated is False
    assert Palette(UISettings(motion="full")).anim_ms > 0


def test_settings_roundtrip():
    settings = UISettings.from_dict({"accent": "#69a7ff", "glass": "vivid",
                                     "autopilot": False, "check_interval": 300})
    restored = UISettings.from_dict(settings.to_dict())
    assert restored == settings


def test_presets_and_accents_are_valid():
    for _key, color, _label in ACCENTS:
        assert normalize_hex(color) == color
    for _key, label, values in PRESETS:
        assert label
        UISettings.from_dict(values).sanitized()
