"""A browser can stage a signed update over LAN without changing the installed code."""
from __future__ import annotations

import io
import os
import shutil
import subprocess
import threading
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import pytest
from werkzeug.serving import make_server

from conftest import login
from robot_control import app as web_app, estop, usb_update
from robot_control.updater import UpdateBusy, UpdateManager
from scripts import make_usb_update
from test_updater import git, wait_for_terminal
from test_usb_update import usb_repo  # noqa: F401 - shared fixture

UPLOAD = "/api/update/upload"
CHECK = "/api/update/check?source=upload"
RUN = "/api/update/run"
CONTENT_TYPE = "application/octet-stream"


@pytest.fixture
def lan_update(config, operator, usb_repo, monkeypatch, request):  # noqa: F811
    seed, robot, drive, key, base = usb_repo
    # Exercise a real upload larger than the ordinary 64 KiB request cap.
    (seed / "update-content.bin").write_bytes(os.urandom(96 << 10))
    git(seed, "add", "update-content.bin")
    git(seed, "commit", "-m", "update content for LAN")
    git(seed, "push", "origin", "main")
    path, sha = make_usb_update.make_package(seed, drive, key, branch="main", base=base,
                                            as_zip=getattr(request, "param", "zip") == "zip")
    monkeypatch.setattr(web_app, "PROJECT_ROOT", robot)
    application = web_app.create_app(config)
    application.config.update(TESTING=True)
    return application, application.extensions["rc_updater"], path.read_bytes(), sha, base


def upload(client, package, headers=None, query=""):
    return client.post(UPLOAD + query, data=package, headers=headers, content_type=CONTENT_TYPE)


def logged_in(client):
    assert login(client).status_code == 302
    response = client.get(CHECK)
    assert response.status_code == 200
    return {"X-CSRF-Token": response.get_json()["csrfToken"]}


@pytest.mark.parametrize("lan_update", ["zip", "bulka"], indirect=True)
def test_lan_upload_only_prepares_then_installs_offline(lan_update):
    app, manager, package, sha, base = lan_update
    client = app.test_client()
    headers = logged_in(client)

    assert client.get(CHECK).get_json()["pending"] is False
    received = upload(client, package, headers)
    assert received.status_code == 201, received.get_json()
    offer = received.get_json()
    assert offer["mode"] == "upload" and offer["latestSha"] == sha
    assert offer["needsEstop"] and not offer["canUpdate"]
    assert manager.local_revision() == base  # No automatic install on upload.
    assert app.config["MAX_CONTENT_LENGTH"] == 64 * 1024
    assert client.get(CHECK).get_json()["latestSha"] == sha

    command = {"source": "upload", "expectedSha": sha}
    assert client.post(RUN, json=command, headers=headers).status_code == 202
    assert "E-STOP" in wait_for_terminal(manager)["error"]
    assert manager.local_revision() == base
    assert manager.upload_path.is_file()  # E-STOP can be enabled without another upload.

    estop.latch()
    assert client.get(CHECK).get_json()["canUpdate"]
    assert client.post(RUN, json=command, headers=headers).status_code == 202
    state = wait_for_terminal(manager)
    assert state["source"] == "upload" and state["status"] == "done", state
    assert state["toSha"] == sha == manager.local_revision()
    assert (manager.root / ".env").read_text() == "RC_PORT=18080\n"
    assert not manager.upload_path.exists()
    assert not manager.upload_path.with_suffix(".part").exists()


def test_upload_requires_login_csrf_and_bounded_body(lan_update, monkeypatch):
    app, manager, package, _sha, _base = lan_update
    client = app.test_client()
    assert upload(client, package).status_code == 302
    headers = logged_in(client)
    assert upload(client, package).status_code == 400
    assert client.post(UPLOAD, json={}, headers=headers).status_code == 415
    assert client.post(UPLOAD, data=package, content_type=CONTENT_TYPE, headers=headers,
                       environ_overrides={"CONTENT_LENGTH": None}).status_code == 411
    monkeypatch.setattr(web_app, "UPLOAD_LIMIT", len(package) - 1)
    assert upload(client, package, headers).status_code == 413
    assert not manager.upload_path.exists()
    assert not manager.upload_path.with_suffix(".part").exists()


def test_upload_works_with_url_session_and_without_cookies(lan_update):
    app, _manager, package, _sha, _base = lan_update
    client = app.test_client(use_cookies=False)
    response = login(client)
    token = parse_qs(urlsplit(response.headers["Location"]).query)["st"][0]
    csrf = client.get(f"{CHECK}&st={token}").get_json()["csrfToken"]
    assert upload(client, package, {"X-CSRF-Token": csrf}, f"?st={token}").status_code == 201


@pytest.mark.parametrize("kind, reason", [
    ("source-code", "Code → Download ZIP"),
    ("unapproved-entry", "Code → Download ZIP"),
    ("deflate", "неподдерживаемый формат"),
    ("wrong-data", "Контрольная сумма"),
    ("wrong-signature", "Подпись файла обновления не подтверждена"),
    ("truncated", "архив повреждён"),
])
def test_invalid_zip_does_not_replace_valid_upload(lan_update, kind, reason):
    app, manager, package, sha, base = lan_update
    client = app.test_client()
    headers = logged_in(client)
    assert upload(client, package, headers).status_code == 201
    with zipfile.ZipFile(io.BytesIO(package)) as archive:
        member = archive.read(usb_update.PACKAGE_NAME)
    bad = io.BytesIO()
    if kind == "truncated":
        bad.write(package[:-16])
    else:
        with zipfile.ZipFile(bad, "w") as archive:
            if kind == "source-code":
                archive.writestr("Zapret-main/README.md", b"GitHub source archive")
            elif kind == "deflate":
                archive.writestr(usb_update.PACKAGE_NAME, member, compress_type=zipfile.ZIP_DEFLATED)
            else:
                if kind == "unapproved-entry":
                    archive.writestr("../not-an-update.txt", b"Not an update")
                elif kind == "wrong-data":
                    member = member[:-1] + bytes([member[-1] ^ 1])
                elif kind == "wrong-signature":
                    # A different signed-revision claim of the same length.
                    a, metadata, c, payload = member.split(b"\n", 3)
                    member = b"\n".join((a, metadata.replace(sha.encode(), b"1" * 40), c, payload))
                archive.writestr(usb_update.PACKAGE_NAME, member)

    result = upload(client, bad.getvalue(), headers)
    assert result.status_code == 422 and reason in result.get_json()["error"], result.get_json()
    assert client.get(CHECK).get_json()["latestSha"] == sha
    assert manager.upload_path.read_bytes() == package
    assert manager.local_revision() == base
    assert not manager.upload_path.with_suffix(".part").exists()


def test_disconnected_upload_preserves_pending_package_and_cleans_up(lan_update):
    app, manager, package, sha, _base = lan_update
    client = app.test_client()
    headers = logged_in(client)
    partial = manager.upload_path.with_suffix(".part")
    partial.parent.mkdir(exist_ok=True)
    partial.write_bytes(b"left behind by an earlier worker")
    assert upload(client, package, headers).status_code == 201
    assert not partial.exists()
    response = client.post(UPLOAD, data=package[:4096], content_type=CONTENT_TYPE, headers=headers,
                           environ_overrides={"CONTENT_LENGTH": str(len(package))})
    assert response.status_code == 400
    assert "Передача прервана" in response.get_json()["error"]
    assert not partial.exists()
    assert client.get(CHECK).get_json()["latestSha"] == sha
    assert manager.upload_path.read_bytes() == package


def test_mismatched_confirmation_cannot_install_a_different_upload(lan_update):
    app, manager, package, _sha, base = lan_update
    client = app.test_client()
    headers = logged_in(client)
    estop.latch()
    assert upload(client, package, headers).status_code == 201
    assert client.post(RUN, headers=headers, json={"source": "upload"}).status_code == 400
    wrong = client.post(RUN, headers=headers, json={"source": "upload", "expectedSha": "1" * 40})
    assert wrong.status_code == 202
    assert "Файл обновления изменился" in wait_for_terminal(manager)["error"]
    assert manager.local_revision() == base
    assert manager.upload_path.is_file()


def test_upload_and_install_hold_the_same_lock_across_workers(lan_update):
    _app, manager, package, sha, base = lan_update
    receiving = threading.Event()
    proceed = threading.Event()

    class PausedBody(io.BytesIO):
        def read(self, size=-1):
            receiving.set()
            assert proceed.wait(10)
            return super().read(size)

    with ThreadPoolExecutor(max_workers=1) as executor:
        pending = executor.submit(manager.stage_upload, PausedBody(package), len(package))
        try:
            assert receiving.wait(10)
            peer = UpdateManager(manager.root)
            assert peer.start(usb_sha=sha) is False
            with pytest.raises(UpdateBusy):
                peer.stage_upload(io.BytesIO(package), len(package))
        finally:
            proceed.set()
        assert pending.result(timeout=10)["latestSha"] == sha
    assert manager.local_revision() == base


def test_release_zip_for_pinned_revision_even_if_main_advanced(usb_repo):  # noqa: F811
    seed, robot, drive, key, base = usb_repo
    revision = git(seed, "rev-parse", "HEAD")
    git(seed, "commit", "--allow-empty", "-m", "new tip after workflow was started")
    git(seed, "push", "origin", "main")
    path, sha = make_usb_update.make_package(seed, drive, key, branch="main", base=base, as_zip=True,
                                            target_revision=revision)
    assert path.name == usb_update.ZIP_NAME
    assert sha == revision != git(seed, "rev-parse", "HEAD")
    with usb_update.verified_package(path, robot / usb_update.TRUST_FILE) as (stream, manifest):
        assert manifest.revision == revision
        usb_update.verify_bundle_bytes(stream, manifest)


def test_update_panel_in_chromium(lan_update, tmp_path):
    node = shutil.which("node")
    if not node or not all(os.environ.get(key) for key in ("RC_TEST_CHROMIUM", "RC_TEST_PLAYWRIGHT_CORE")):
        pytest.skip("Set RC_TEST_CHROMIUM and RC_TEST_PLAYWRIGHT_CORE to run the browser test")
    app, manager, package, sha, base = lan_update
    manager.on_restart = None  # No service restarts in this test.
    path = tmp_path / usb_update.ZIP_NAME
    path.write_bytes(package)
    server = make_server("127.0.0.1", 0, app, threaded=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        script = Path(__file__).parent / "js" / "update_upload_browser.mjs"
        result = subprocess.run([node, str(script), f"http://127.0.0.1:{server.server_port}",
                                 str(path), str(estop.latch_path()), base, sha],
                                capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout + result.stderr
    finally:
        server.shutdown()
        thread.join(timeout=5)
        server.server_close()
