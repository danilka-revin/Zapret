"""Один владелец UART мастера: бортовой экран (8090) открывает порт, пульт (8080) читает с него.

Раньше оба процесса открывали /dev/ttyACM0 и читали его вперемешку: строки
телеметрии рвались, команды поворота отклонялись (serial_unavailable,
module_offline, not_homed). Тесты фиксируют, что порт открывается монопольно
и что пульт порт не трогает.
"""

from __future__ import annotations

import os
import sys
import time

import pytest

from robot_control import slam
from robot_control.slam import make_source, steering_control_info


def _backend():
    return slam.load_backend()


def _snapshot(angle_fr=12.5, linked=True, estop=None):
    motors = [
        {"id": mid, "title": mid, "angle": angle_fr if mid == "FR" else 0.0,
         "targetAngle": angle_fr if mid == "FR" else 0.0, "online": True,
         "homed": True, "moving": False, "calibrating": False}
        for mid in ("FL", "FR", "RL", "RR")
    ]
    return {"ok": True, "data": {
        "motors": motors,
        "drive": {"available": True, "canDrive": False, "reason": "not_homed"},
        "linkOk": linked,
        "mode": "МАСТЕР НА СВЯЗИ" if linked else None,
        "steeringControl": {"available": True, "source": "master"},
        "estop": estop,
    }}


@pytest.mark.skipif(sys.platform.startswith("win"), reason="нужен POSIX pty")
def test_master_port_is_opened_exclusively_by_one_process():
    """Второй процесс не может открыть порт колеса, пока его держит владелец.

    Порт считается своим после @TLM: плата называет колесо, и сервер сам
    привязывает его к порту (I²C больше нет, все четыре колеса на своих USB).
    """
    serial = pytest.importorskip("serial")
    backend = _backend()
    master_fd, slave_fd = os.openpty()
    try:
        name = os.ttyname(slave_fd)
        owner = backend.MasterSource(name)
        deadline = time.time() + 3.0
        while not owner.serial_ready.is_set() and time.time() < deadline:
            # плата «перезагрузилась» при открытии и вскоре печатает @TLM
            try:
                os.write(master_fd, b"@TLM mod=FL deg=0.00 tgt=0.00 moving=0 cal=1 "
                                    b"cycle=0 calib=0 opto=1 rpm=0.0 drive=0 fw=4 t=1\r\n")
            except OSError:
                pass
            time.sleep(0.05)
        assert owner.serial_ready.is_set(), "владелец должен опознать колесо"
        assert owner.modules.get("FL") == name, "FL привязан к порту автоматически"

        with pytest.raises(serial.SerialException):
            serial.Serial(name, 115200, timeout=0.1, exclusive=True)
        owner._stop = True
    finally:
        os.close(master_fd)
        os.close(slave_fd)


def test_peer_source_reads_owner_snapshot_and_never_opens_port(monkeypatch, tmp_path):
    backend = _backend()
    monkeypatch.setenv("RC_SLAM_PORT", str(tmp_path / "no-such-tty"))
    monkeypatch.setenv("RC_SLAM_PEER_URL", "http://127.0.0.1:9/api/kiosk_state")
    peer = make_source("master", master_role="peer")
    assert isinstance(peer, backend.MasterPeerSource)
    onboard_estop = {"reason": "test", "since": 1.0, "corrupt": False}
    peer._fetch = lambda: _snapshot(estop=onboard_estop)
    time.sleep(0.4)

    telemetry = peer.read()
    motors = {m["id"]: m for m in telemetry["motors"]}
    assert telemetry["estop"] == onboard_estop
    assert motors["FR"]["angle"] == 12.5
    assert motors["FR"]["online"] is True
    assert steering_control_info(peer)["available"] is False
    assert steering_control_info(peer)["source"] == "master-peer"


def test_peer_refuses_commands_and_reports_offline_when_owner_is_down():
    backend = _backend()
    peer = backend.MasterPeerSource("http://127.0.0.1:9/api/kiosk_state",
                                    fetch=lambda: (_ for _ in ()).throw(OSError("refused")))
    time.sleep(0.4)

    assert peer.steer_module("FR", 5)["reason"] == "control_not_supported"
    assert peer.zero_module("FR")["reason"] == "control_not_supported"
    assert peer.drive(5)["reason"] == "control_not_supported"
    out = peer.read()
    assert out["linkOk"] is False
    assert all(m["online"] is False for m in out["motors"])
    assert out["estop"] == {"unknown": True}
    assert "refused" in out["peer"]["error"]


def test_peer_snapshot_goes_stale_when_owner_stops_answering():
    backend = _backend()
    state = {"fresh": True}

    def fetch():
        if state["fresh"]:
            return _snapshot()
        raise OSError("owner gone")

    peer = backend.MasterPeerSource("http://127.0.0.1:9/api/kiosk_state", fetch=fetch)
    time.sleep(0.3)
    assert peer.read()["linkOk"] is True

    state["fresh"] = False
    time.sleep(backend.MasterPeerSource.STALE_S + 0.5)
    assert peer.read()["linkOk"] is False


def test_owner_role_is_default_for_the_onboard_screen(monkeypatch, tmp_path):
    backend = _backend()
    monkeypatch.setenv("RC_SLAM_PORT", str(tmp_path / "missing-tty"))
    monkeypatch.delenv("RC_SLAM_PEER_URL", raising=False)
    owner = make_source("master")
    try:
        assert isinstance(owner, backend.MasterSource)
    finally:
        owner._stop = True


def test_peer_url_defaults_to_onboard_screen_port(monkeypatch):
    monkeypatch.delenv("RC_SLAM_PEER_URL", raising=False)
    monkeypatch.setenv("RC_KIOSK_PORT", "8090")
    assert slam.peer_kiosk_url() == "http://127.0.0.1:8090/api/kiosk_state"
    monkeypatch.setenv("RC_SLAM_PEER_URL", "http://10.0.0.5:8090/api/kiosk_state")
    assert slam.peer_kiosk_url() == "http://10.0.0.5:8090/api/kiosk_state"
