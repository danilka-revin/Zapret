"""Бортовой ползунок: точное значение Servo.write и независимость от руля/SC."""

from __future__ import annotations

import queue
import threading
import time

import pytest

from robot_control import estop, slam

MODULES = ("FL", "FR", "RL", "RR")
HEADER = {"X-Robot-Control": "drive"}


class RecordingServo:
    supports_drive = True
    name = "master"

    def __init__(self):
        self.calls = []

    def drive_servo(self, value, module=None):
        self.calls.append((module, value))
        return {"ok": True}

    def drive(self, percent):
        self.calls.append(("legacy", percent))
        return {"ok": True, "percent": percent}


@pytest.mark.parametrize("value", [0, 1, 40, 75, 149, 150])
def test_kiosk_transmits_exact_integers(kiosk_client, value):
    source = RecordingServo()
    kiosk_client.application.extensions["rc_slam"].source = source
    response = kiosk_client.post("/api/kiosk_drive", json={"module": "fr", "value": value}, headers=HEADER)
    assert response.status_code == 200, response.json
    assert response.json["value"] == value and response.json["module"] == "FR"
    assert source.calls == [("FR", value)]

    all_wheels = kiosk_client.post("/api/kiosk_drive", json={"value": value}, headers=HEADER)
    assert all_wheels.status_code == 200 and all_wheels.json["value"] == value
    assert source.calls[-1] == (None, value)


@pytest.mark.parametrize("value", [-1, 151, True, None, 40.5, 40.0, "40", float("nan")])
def test_kiosk_rejects_non_integer_and_out_of_range_values(kiosk_client, value):
    source = RecordingServo()
    kiosk_client.application.extensions["rc_slam"].source = source
    response = kiosk_client.post("/api/kiosk_drive", json={"value": value}, headers=HEADER)
    assert response.status_code == 400
    assert source.calls == []


def test_kiosk_checks_payload_header_and_estop(kiosk_client):
    source = RecordingServo()
    kiosk_client.application.extensions["rc_slam"].source = source
    assert kiosk_client.post("/api/kiosk_drive", json={"value": 40}).status_code == 400
    for body in ({"value": 40, "percent": 0}, {"module": "XX", "value": 40},
                 {"module": None, "value": 40}):
        assert kiosk_client.post("/api/kiosk_drive", json=body, headers=HEADER).status_code == 400
    estop.latch("test")
    assert kiosk_client.post("/api/kiosk_drive", json={"value": 40}, headers=HEADER).json["reason"] == "estop"
    assert kiosk_client.post("/api/kiosk_drive", json={"value": 0}, headers=HEADER).status_code == 200
    assert source.calls == [(None, 0)]


def test_expired_remote_direct_command_is_refused_but_zero_is_allowed(kiosk_client):
    source = RecordingServo()
    kiosk_client.application.extensions["rc_slam"].source = source
    headers = {**HEADER, "X-Robot-Command-Deadline": str(time.time() - 1)}
    result = kiosk_client.post("/api/kiosk_drive", json={"value": 40}, headers=headers)
    assert result.status_code == 408 and result.json["reason"] == "command_expired"
    assert kiosk_client.post("/api/kiosk_drive", json={"value": 0}, headers=headers).status_code == 200
    assert source.calls == [(None, 0)]


def bare_source():
    backend = slam.load_backend()
    source = object.__new__(backend.MasterSource)
    source.lock = threading.Lock()
    source._commands_lock = threading.RLock()
    source.commands = queue.Queue(maxsize=16)
    source.serial_ready = threading.Event()
    source.serial_ready.set()
    source.frames = {}
    source.drive_percent = 0
    source.drive_modules = {}
    source.servo_commands = {}
    source.drive_last_command = 0.0
    source.module_ports = {m: f"/dev/test-{m}" for m in MODULES}
    source.links = {}
    source.messages = []
    source.on_message = None
    source.maneuvers = None
    for module, path in source.module_ports.items():
        link = backend._WheelLink(path)
        link.module = module
        source.links[path] = link
    return backend, source


def frame(module, *, fw=14, cal=0, moving=1, calib=1, tach=0, sv=0):
    return (f"@TLM mod={module} deg=0 tgt=0 moving={moving} cal={cal} cycle=0 calib={calib} "
            f"rpm=0 drive=0 fw={fw} tach={tach} dmax=100 sv={sv} duty=0 ppr=90 hz=0 t=1")


def test_source_routes_direct_values_without_calibration_or_tachometer():
    backend, source = bare_source()
    for module in MODULES:
        assert source.feed(frame(module), now=100.0)
    status = source.servo_drive_status(now=100.2)
    assert status["canDrive"] and all(x["canDrive"] for x in status["perModule"].values())
    # Прежняя процентная команда заблокирована движением/нулём руля,
    # а новый прямой путь не проверяет эти функции.
    assert not source.drive_status(now=100.2)["canDrive"]
    for value in (1, 40, 75, 150):
        assert source.drive_servo(value, module="FR", now=100.2)["ok"]
        assert source.commands.get_nowait() == f"v FR {value}\n".encode()
    assert source.drive_servo(150, now=100.2)["ok"]
    assert source.commands.get_nowait() == b"v 150\n"
    assert source._module_drive_active("FR")
    assert backend.DRIVE_DIRECT_MAX_VALUE == 150


def test_source_requires_new_firmware_and_fresh_port_but_always_allows_stop():
    _, source = bare_source()
    source.feed(frame("FL", fw=13), now=100.0)
    source.feed(frame("FR"), now=100.0)
    assert source.drive_servo(40, module="FL", now=100.1)["reason"] == "servo_firmware_required"
    assert source.drive_servo(40, module="RL", now=100.1)["reason"] == "module_offline"
    assert source.drive_servo(40, module="FR", now=101.0)["reason"] == "module_offline"
    assert source.drive_servo(40, now=100.1)["reason"] == "servo_firmware_required"
    assert source.drive_servo(0, module="FL", now=101.0)["ok"]
    assert source.commands.get_nowait() == b"b FL\n"
    assert source.drive_servo(0, now=101.0)["ok"]
    assert source.commands.get_nowait() == b"b\n"


def test_direct_fanout_and_single_stop_do_not_restart_or_stop_other_wheels():
    _, source = bare_source()
    for module in MODULES:
        source.feed(frame(module), now=100.0)
    source.drive_servo(40, module="FL", now=100.1)
    source._fanout_commands()
    assert source.links["/dev/test-FL"].commands.get_nowait() == b"v 40\n"
    assert all(source.links[f"/dev/test-{m}"].commands.empty() for m in ("FR", "RL", "RR"))

    # Pending older FL/broadcast commands cannot follow an individual stop;
    # queued FR movement and steering for other wheels are preserved.
    source.commands.put(b"v 100\n")
    source.commands.put(b"v FL 75\n")
    source.commands.put(b"v FR 50\n")
    source.commands.put(b"w RR 10\n")
    source.links["/dev/test-FL"].commands.put(b"v 99\n")
    source.links["/dev/test-FR"].commands.put(b"v 50\n")
    assert source.drive_servo(0, module="FL", now=100.2)["ok"]
    assert list(source.commands.queue) == [b"v FR 50\n", b"w RR 10\n", b"b FL\n"]
    assert source.links["/dev/test-FL"].commands.empty()
    assert source.links["/dev/test-FR"].commands.get_nowait() == b"v 50\n"
    source._fanout_commands()
    assert source.links["/dev/test-FL"].commands.get_nowait() == b"b\n"
    assert source.links["/dev/test-FR"].commands.get_nowait() == b"v 50\n"
    assert source.links["/dev/test-RR"].commands.get_nowait() == b"w 10\n"


def test_telemetry_parses_exact_servo_value_without_tying_it_to_tach():
    backend, source = bare_source()
    assert source.feed(frame("FL", sv=150, tach=0), now=100.0)
    motor = next(m for m in source.read(now=100.1)["motors"] if m["id"] == "FL")
    assert motor["driveServoValue"] == 150 and motor["tach"] is False
    assert backend.parse_master_line(frame("FL", sv=181)) is None
    assert backend.parse_master_line(frame("FL", sv=-1)) is None


def test_direct_command_waits_for_maneuver_tick_then_cancels_queued_thrust():
    """Если тик манёвра уже выполняется в другом потоке, ручной ввод главнее."""
    backend, source = bare_source()
    source._manual_motion_lock = threading.RLock()
    source._maneuver_context = threading.local()
    source.maneuver_core = backend._load_maneuvers()
    runner = source.maneuvers = source.maneuver_core.ManeuverRunner(brake=source._brake_all)
    for module in MODULES:
        source.feed(frame(module, cal=1, moving=0, calib=0), now=time.time())
    assert runner.start("forward", time.monotonic(), limit_percent=20)["ok"]
    entered, release, manual_done = threading.Event(), threading.Event(), threading.Event()
    manual_result = []

    def slow_tick(now, frames, steer, drive):
        with runner.lock:
            entered.set()
            assert release.wait(2)
            drive(None, 15)  # ещё один heartbeat манёвра из его собственного потока

    runner.tick = slow_tick
    worker = threading.Thread(target=source._maneuver_tick)
    worker.start()
    assert entered.wait(2)

    def manual():
        try:
            manual_result.append(source.drive_servo(40, "FL"))
        finally:
            manual_done.set()

    operator = threading.Thread(target=manual)
    operator.start()
    try:
        assert not manual_done.wait(0.1)  # нельзя проскочить мимо занятого runner.lock
    finally:
        release.set()
        operator.join(timeout=2)
        worker.join(timeout=2)
    assert not operator.is_alive() and not worker.is_alive()
    assert manual_result == [{"ok": True, "queued": True, "value": 40,
                              "maxValue": 150, "module": "FL"}]
    assert runner.status()["state"] == "cancelled"
    assert list(source.commands.queue) == [b"v FL 40\n"]  # t 15 удалён
    source._fanout_commands()
    assert list(source.links["/dev/test-FL"].commands.queue) == [b"b\n", b"v 40\n"]
    assert all(list(source.links[f"/dev/test-{m}"].commands.queue) == [b"b\n"]
               for m in ("FR", "RL", "RR"))


def test_new_maneuver_is_rejected_while_direct_drive_is_active():
    backend, source = bare_source()
    source._manual_motion_lock = threading.RLock()
    source.maneuver_core = backend._load_maneuvers()
    source.maneuvers = source.maneuver_core.ManeuverRunner(brake=source._brake_all)
    now = time.time()
    for module in MODULES:
        source.feed(frame(module, cal=1, moving=0, calib=0), now=now)
    assert source.drive_servo(40, module="FL", now=now)["ok"]
    result = source.start_maneuver("forward", now=now)
    assert result == {"ok": False, "reason": "module_busy"}


def test_estop_cannot_be_overtaken_by_an_inflight_direct_command():
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
    results = []

    class SlowSource(RecordingServo):
        def drive_servo(self, value, module=None):
            entered.set()
            assert release.wait(2)
            return super().drive_servo(value, module)

        def drive(self, percent):
            result = super().drive(percent)
            stopped.set()
            return result

    source = SlowSource()
    manual = threading.Thread(target=lambda: results.append(slam.dispatch_drive_command(source, {"value": 40})))
    manual.start()
    assert entered.wait(2)
    safety = threading.Thread(target=lambda: slam.engage_estop(source))
    safety.start()
    try:
        assert not stopped.wait(0.1)  # сначала завершить уже принятую команду, потом тормоз
    finally:
        release.set()
        manual.join(timeout=2)
        safety.join(timeout=2)
    assert not manual.is_alive() and not safety.is_alive()
    assert results[0][0]["ok"] and source.calls == [(None, 40), ("legacy", 0)]
    assert slam.dispatch_drive_command(source, {"value": 40})[0]["reason"] == "estop"
    assert source.calls == [(None, 40), ("legacy", 0)]


def _navigation_worker_that_brakes_after_cancel(kiosk_client, source, delay=0.08):
    navigation = kiosk_client.application.extensions["rc_navigation"]
    navigation.source = source
    navigation._state["phase"] = "executing"

    def finish():
        navigation._cancel.wait(2)
        time.sleep(delay)
        navigation._brake(None, "navigation_cancel")

    worker = threading.Thread(target=finish)
    navigation._worker = worker
    worker.start()
    return navigation, worker


def test_manual_slider_waits_until_navigation_finishes_braking(kiosk_client):
    source = RecordingServo()
    kiosk_client.application.extensions["rc_slam"].source = source
    _, worker = _navigation_worker_that_brakes_after_cancel(kiosk_client, source)
    reply = kiosk_client.post("/api/kiosk_drive", json={"value": 40}, headers=HEADER)
    worker.join(timeout=2)
    assert reply.status_code == 200, reply.json
    assert source.calls == [("legacy", 0), ("legacy", 0), (None, 40)]


def test_navigation_cannot_send_a_late_brake_after_a_direct_command(kiosk_client):
    source = RecordingServo()
    kiosk_client.application.extensions["rc_slam"].source = source
    _, worker = _navigation_worker_that_brakes_after_cancel(kiosk_client, source, delay=0.45)
    try:
        reply = kiosk_client.post("/api/kiosk_drive", json={"value": 40}, headers=HEADER)
        assert reply.status_code == 409 and reply.json["reason"] == "navigation_stopping"
        assert not any(call == (None, 40) for call in source.calls)
    finally:
        worker.join(timeout=2)


def test_network_deadline_is_rechecked_after_navigation_cancels(kiosk_client):
    source = RecordingServo()
    kiosk_client.application.extensions["rc_slam"].source = source
    _, worker = _navigation_worker_that_brakes_after_cancel(kiosk_client, source)
    headers = {**HEADER, "X-Robot-Command-Deadline": str(time.time() + 0.04)}
    reply = kiosk_client.post("/api/kiosk_drive", json={"value": 40}, headers=headers)
    worker.join(timeout=2)
    assert reply.status_code == 408 and reply.json["reason"] == "command_expired"
    assert not any(call == (None, 40) for call in source.calls)


def test_losing_unrelated_port_does_not_interrupt_a_single_direct_wheel():
    _, source = bare_source()
    now = time.time()
    for module in MODULES:
        source.feed(frame(module), now=now)
    assert source.drive_servo(40, "FL", now=now)["ok"]
    assert not source._brake_for_lost_module("RR")
    assert source._brake_for_lost_module("FL")
    # Старый процентный режим по-прежнему гасит все колёса при потере порта.
    source.drive_modules = {"FR": 8}
    assert source._brake_for_lost_module("RR")
