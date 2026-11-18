"""Signed USB updates, exercised with real Git bundles and temporary SSH keys."""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import threading
from pathlib import Path

import pytest

from conftest import login
from robot_control import estop, usb_update
from robot_control.updater import UpdateManager
from scripts import make_usb_update
from test_updater import git, make_remote, wait_for_terminal


def keygen(path: Path) -> None:
    if not shutil.which("ssh-keygen"):
        pytest.skip("openssh-client is required for USB updates")
    subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-f", str(path)], check=True)


@pytest.fixture
def usb_repo(tmp_path, monkeypatch):
    remote, seed, robot = make_remote(tmp_path)
    (robot / ".git/info/exclude").write_text(".env\ndata/\n", encoding="utf-8")
    (robot / ".env").write_text("RC_PORT=18080\n", encoding="utf-8")
    base = git(robot, "rev-parse", "HEAD")
    (seed / "README.md").write_text("version from GitHub\n", encoding="utf-8")
    (seed / "scripts").mkdir()
    (seed / "scripts/ensure_deps.sh").write_text('test "$RC_DEPS_OFFLINE" = 1\n', encoding="utf-8")
    git(seed, "add", ".")
    git(seed, "commit", "-m", "new version")
    git(seed, "push", "origin", "main")
    # The packer only signs the fetched branch, not this local edit.
    (seed / "README.md").write_text("uncommitted edit on PC\n", encoding="utf-8")
    key = tmp_path / "signing-key"
    keygen(key)
    trust = robot / usb_update.TRUST_FILE
    trust.parent.mkdir()
    trust.write_text(f"{usb_update.SIGNER} {key.with_suffix('.pub').read_text()}", encoding="utf-8")
    drive = tmp_path / "USB Drive"
    drive.mkdir()
    monkeypatch.setattr(make_usb_update, "GITHUB_URL", str(remote))
    monkeypatch.setenv("GIT_ALLOW_PROTOCOL", "file")  # No network even if something tried.
    git(robot, "remote", "remove", "origin")
    return seed, robot, drive, key, base


@pytest.fixture
def package(usb_repo, monkeypatch, request):
    seed, robot, drive, key, base = usb_repo
    variant = getattr(request, "param", "delta")
    path, sha = make_usb_update.make_package(seed, drive, key, branch="main",
                                            base=base if variant == "delta" else None, as_zip=variant == "zip")
    monkeypatch.setattr(usb_update, "find_package", lambda _mount: path if path.exists() else None)
    assert path == drive / (usb_update.ZIP_NAME if variant == "zip" else usb_update.PACKAGE_NAME)
    assert git(seed, "for-each-ref", "refs/bulka-usb/") == ""
    return path, sha, robot, base


@pytest.mark.parametrize("package", ["full", "delta", "zip"], indirect=True)
def test_signed_usb_update_applies_offline_and_preserves_data(package):
    path, sha, robot, _base = package
    settings = robot / "data" / "settings.json"
    settings.write_text('{"calibration": 123}', encoding="utf-8")
    assert set(path.parent.iterdir()) == {path}  # One file for the flash drive.
    restarted = []
    manager = UpdateManager(robot, on_restart=lambda: restarted.append(True),
                            ros_builder=lambda _root: pytest.fail("USB must not fetch ROS dependencies"))
    estop.latch()
    offer = manager.check_usb()
    assert offer["canUpdate"] and offer["latestSha"] == sha
    assert manager.start(usb_sha=sha)
    result = wait_for_terminal(manager)
    assert result["status"] == "done", result
    assert result["source"] == "usb" and result["toSha"] == sha
    assert git(robot, "rev-parse", "HEAD") == sha
    assert (robot / "README.md").read_text() == "version from GitHub\n"
    assert (robot / ".env").read_text() == "RC_PORT=18080\n"
    assert settings.read_text() == '{"calibration": 123}'
    assert restarted == [True]
    assert not manager.check_usb()["updateAvailable"]
    assert not list((robot / "var").glob("usb-update-*"))


def test_older_usb_package_is_not_reapplied(package):
    _path, sha, robot, _base = package
    estop.latch()
    manager = UpdateManager(robot)
    assert manager.start(usb_sha=sha)
    assert wait_for_terminal(manager)["status"] == "done"
    git(robot, "commit", "--allow-empty", "-m", "newer than the USB package")
    current = git(robot, "rev-parse", "HEAD")
    assert not manager.check_usb()["canUpdate"]
    retry = UpdateManager(robot, on_restart=lambda: pytest.fail("an older package cannot restart"))
    assert retry.start(usb_sha=sha)
    result = wait_for_terminal(retry)
    assert result["status"] == "done" and result["changes"] == 0
    assert git(robot, "rev-parse", "HEAD") == current


@pytest.mark.parametrize("failure, message", [
    ("payload", "Контрольная сумма"),
    ("unsigned", "не является файлом обновления"),
    ("symlink", "не является обычным файлом"),
    ("untrusted", "Подпись файла обновления не подтверждена"),
    ("wrong_branch", "робот использует «other-branch»"),
    ("unexpected_version", "Файл обновления изменился"),
    ("dirty", "в рабочей копии есть изменения"),
    ("diverged", "ветки разошлись"),
    ("build", "исходный код откатан"),
    ("moving", "E-STOP"),
])
def test_usb_refuses_an_unsafe_update(package, failure, message):
    path, sha, robot, _base = package
    if failure != "moving":
        estop.latch()
    if failure == "payload":
        with path.open("r+b") as handle:
            handle.seek(-1, os.SEEK_END)
            last = handle.read(1)[0]
            handle.seek(-1, os.SEEK_END)
            handle.write(bytes([last ^ 1]))
    elif failure == "unsigned":
        path.write_text("#!/bin/sh\nexit 0\n")
    elif failure == "symlink":
        actual = path.rename(path.with_suffix(".other"))
        path.symlink_to(actual)
    elif failure == "wrong_branch":
        git(robot, "branch", "-m", "other-branch")
    elif failure == "untrusted":
        keygen(path.parent.parent / "other-key")
        (robot / usb_update.TRUST_FILE).write_text(
            f"{usb_update.SIGNER} {(path.parent.parent / 'other-key.pub').read_text()}")
    elif failure == "unexpected_version":
        sha = "1" * 40
    elif failure in ("dirty", "diverged"):
        (robot / "README.md").write_text("local changes\n")
        if failure == "diverged":
            git(robot, "commit", "-am", "local commit")
    old_sha = git(robot, "rev-parse", "HEAD")
    manager = UpdateManager(robot, build_runner=lambda root: (1, "syntax error"),
                            on_restart=lambda: pytest.fail("unsafe update cannot restart"))
    assert manager.start(usb_sha=sha)
    result = wait_for_terminal(manager)
    assert result["status"] == "error" and message in result["error"], result
    assert git(robot, "rev-parse", "HEAD") == old_sha
    if failure in ("dirty", "diverged"):
        assert (robot / "README.md").read_text() == "local changes\n"
    assert not list((robot / "var").glob("usb-update-*"))


@pytest.mark.parametrize("package", ["delta", "zip"], indirect=True)
def test_auto_usb_waits_for_stop_and_attempts_the_file_once(package, monkeypatch):
    path, sha, robot, _base = package
    manager = UpdateManager(robot)
    starts = []
    monkeypatch.setattr(manager, "start", lambda *, usb_sha: starts.append(usb_sha) or True)
    assert manager._poll_usb(None) is None
    assert starts == []
    estop.latch()
    token = manager._poll_usb(None)
    assert token is not None
    assert manager._poll_usb(token) == token
    assert starts == [sha]
    path.unlink()
    assert manager._poll_usb(token) is None


def test_usb_and_online_updates_share_the_lock(package):
    _path, sha, robot, _base = package
    estop.latch()
    building = threading.Event()
    proceed = threading.Event()

    def build(_root):
        building.set()
        return (0, "ok") if proceed.wait(10) else (1, "build timed out")

    manager = UpdateManager(robot, build_runner=build)
    assert manager.start(usb_sha=sha)
    try:
        assert building.wait(10)
        assert UpdateManager(robot).start() is False
        assert manager.check_usb()["busy"] is True
    finally:
        proceed.set()
    assert wait_for_terminal(manager)["status"] == "done"


def test_usb_package_is_discovered_only_at_a_matching_mount(tmp_path, monkeypatch):
    flash = tmp_path / "USB with spaces"
    flash.mkdir()
    (flash / "Zapret-main.zip").touch()  # The ordinary GitHub ZIP is not a release asset.
    package = flash / usb_update.ZIP_NAME
    volumes = []

    def findmnt(args, **_kwargs):
        assert args[0] == "findmnt"
        return subprocess.CompletedProcess(args, 0, json.dumps({"filesystems": volumes}))

    monkeypatch.setattr(usb_update.subprocess, "run", findmnt)
    volumes.append({"source": "/dev/sdz1", "target": str(flash)})
    assert usb_update.find_package(flash) is None  # Ignore the ordinary source archive.
    package.touch()
    assert usb_update.find_package(flash) == package
    assert usb_update.find_package() is None  # Nonstandard mount must be configured.
    (flash / usb_update.PACKAGE_NAME).touch()
    with pytest.raises(usb_update.UsbUpdateError, match="несколько пакетов"):
        usb_update.find_package(flash)
    volumes.clear()
    assert usb_update.find_package(flash) is None  # A leftover directory is not a mount.


def test_usb_update_api_requires_login_csrf_and_checked_revision(client, app, operator, monkeypatch):
    manager = app.extensions["rc_updater"]
    sha = "2" * 40
    monkeypatch.setattr(manager, "check_usb", lambda: {"ok": True, "mode": "usb", "latestSha": sha})
    starts = []
    monkeypatch.setattr(manager, "start", lambda *, usb_sha: starts.append(usb_sha) or True)
    check_url = "/api/update/check?source=usb"
    run_url = "/api/update/run"
    payload = {"source": "usb", "expectedSha": sha}
    assert client.get(check_url).status_code == 302
    assert client.post(run_url, json=payload).status_code == 302
    assert login(client).status_code == 302
    offer = client.get(check_url).get_json()
    assert offer["mode"] == "usb" and offer["latestSha"] == sha
    headers = {"X-CSRF-Token": offer["csrfToken"]}
    assert client.post(run_url, json=payload).status_code == 400
    assert client.post(run_url, json={"source": "usb"}, headers=headers).status_code == 400
    assert client.post(run_url, json={"source": "other"}, headers=headers).status_code == 400
    assert client.post(run_url, json=payload, headers=headers).status_code == 202
    assert starts == [sha]
