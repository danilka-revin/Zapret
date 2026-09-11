"""
Тесты скачивания зависимостей: кэш по версии и сохранность пользовательских
списков при обновлении стратегий.

Оба пункта важны для кнопки «Обновить и перезапустить»: без кэша она каждый раз
тянула бы ~10 МБ, а без сохранения списков — молча стирала домены пользователя.
"""

import io
import sys
import tarfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import core  # noqa: E402


@pytest.fixture()
def app(tmp_path, monkeypatch):
    root = tmp_path / "app"
    (root / "deps").mkdir(parents=True)
    monkeypatch.setenv("ZAPRET_APP_DIR", str(root))
    return root


def _flowseal_archive(files: dict[str, str], top: str = "zapret-discord-youtube-abc") -> bytes:
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as tf:
        for name, content in files.items():
            info = tarfile.TarInfo(f"{top}/{name}")
            payload = content.encode("utf-8")
            info.size = len(payload)
            tf.addfile(info, io.BytesIO(payload))
    return buffer.getvalue()


def test_nfqws_skipped_when_cached(app, monkeypatch):
    (app / "deps").mkdir(exist_ok=True)
    (app / "deps" / core.NFQWS_MARKER).write_text("v72.13", encoding="utf-8")
    (app / "nfqws").write_text("binary", encoding="utf-8")

    def no_download(*a, **k):
        raise AssertionError("скачивать не нужно")

    monkeypatch.setattr(core, "_download_to", no_download)
    monkeypatch.setattr(core, "latest_release_tag", lambda repo: "v72.13")
    assert core.ensure_nfqws("latest", force=False) == app / "nfqws"


def test_nfqws_downloaded_when_forced(app, monkeypatch):
    payload = b"#!/bin/sh\necho nfqws\n"

    def fake_download(url, dest, progress_cb=None, attempts=3):
        archive = io.BytesIO()
        with tarfile.open(fileobj=archive, mode="w:gz") as tf:
            name = f"zapret-v9/binaries/{core.platform_dir()}/nfqws"
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            tf.addfile(info, io.BytesIO(payload))
        dest.write_bytes(archive.getvalue())

    monkeypatch.setattr(core, "_download_to", fake_download)
    monkeypatch.setattr(core, "latest_release_tag", lambda repo: "v9")
    path = core.ensure_nfqws("latest", force=True)
    assert path.read_bytes() == payload
    assert (app / "deps" / core.NFQWS_MARKER).read_text(encoding="utf-8") == "v9"


def test_strategies_keep_user_lists(app, monkeypatch):
    lists = app / "deps" / "flowseal" / "lists"
    lists.mkdir(parents=True)
    (lists / "list-general-user.txt").write_text("my-site.ru\nanother.site\n", encoding="utf-8")
    (app / "deps" / "flowseal" / "general.bat").write_text("старая стратегия\n", encoding="utf-8")

    archive = _flowseal_archive({
        "general.bat": 'start "" "%BIN%nfqws" --dpi-desync=fake\n',
        "lists/list-general.txt": "youtube.com\n",
    })
    monkeypatch.setattr(core, "_download_to",
                        lambda url, dest, progress_cb=None, attempts=3: dest.write_bytes(archive))
    core.ensure_strategies("abc", force=True)

    assert "nfqws" in (app / "deps" / "flowseal" / "general.bat").read_text(encoding="utf-8")
    saved = (lists / "list-general-user.txt").read_text(encoding="utf-8")
    assert saved == "my-site.ru\nanother.site\n", "списки пользователя нельзя терять"
    assert (app / "deps" / core.STRATEGIES_MARKER).read_text(encoding="utf-8") == "abc"


def test_strategies_skipped_when_revision_matches(app, monkeypatch):
    (app / "deps" / "flowseal").mkdir()
    (app / "deps" / core.STRATEGIES_MARKER).write_text("abc", encoding="utf-8")

    def no_download(*a, **k):
        raise AssertionError("тот же коммит — качаем зря")

    monkeypatch.setattr(core, "_download_to", no_download)
    assert core.ensure_strategies("abc", force=False).is_dir()


def test_ensure_deps_passes_force_down(app, monkeypatch):
    seen = {}

    def fake_nfqws(version, progress_cb=None, force=True):
        seen["nfqws_force"] = force
        return app / "nfqws"

    def fake_strategies(rev, progress_cb=None, force=True):
        seen["strategies_force"] = force
        return app / "deps" / "flowseal"

    monkeypatch.setattr(core, "ensure_nfqws", fake_nfqws)
    monkeypatch.setattr(core, "ensure_strategies", fake_strategies)
    core.ensure_deps("latest", "", None, force=False)
    assert seen == {"nfqws_force": False, "strategies_force": False}
