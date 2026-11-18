"""Логика вкладки «Автономность»: движение вперёд, знаки, кочка, безопасность.

Проверяется сам модуль robot_control/track_autonomy.py без USB, Flask и модели:
восприятие подменяется словарями и маленькими фейками, как в других тестах
проекта (tests/test_maneuvers.py, tests/test_exploration.py).
"""
from __future__ import annotations

import math
import queue
import threading
from dataclasses import replace
from types import SimpleNamespace

import pytest

from robot_control.kinect import KinectDepthService
from robot_control.track_autonomy import (
    BumpDetector, Odometer, ScriptedWorld, TrackAutonomyError,
    TrackAutonomyService, TrackConfig, TrackMotion, _downsample_depth_frame,
    camera_specs, chassis_feedback, cone_points, estimate_corridor, kinect_navigation_ready,
    lidar_clearance, sign_marks, simulate_track, traffic_decision,
)


# -------------------------------------------------------------------- helpers
def cfg(**kwargs) -> TrackConfig:
    return replace(TrackConfig(), **kwargs)


def cone(x: float, y: float, cam: int = 0) -> dict:
    return {"x": x, "y": y, "distance": x, "cameraId": cam, "confidence": 0.9}


def straight_cones(center: float = 0.0, half: float = 0.8) -> list[dict]:
    return [cone(x, center - half, 2) for x in (0.8, 1.6, 2.4, 3.2)] + \
           [cone(x, center + half, 0) for x in (0.8, 1.6, 2.4, 3.2)]


def layout() -> dict:
    return {"calibrated": True, "cameras": [
        {"id": 0, "yaw_deg": -25.0, "hfov_deg": 82.0, "x_m": 0.30, "y_m": 0.25},
        {"id": 1, "yaw_deg": -135.0, "hfov_deg": 82.0, "x_m": -0.30, "y_m": 0.25},
        {"id": 2, "yaw_deg": 25.0, "hfov_deg": 82.0, "x_m": 0.30, "y_m": -0.25},
        {"id": 3, "yaw_deg": 135.0, "hfov_deg": 82.0, "x_m": -0.30, "y_m": -0.25},
    ]}


def detection(cls: str, cx: float, h: float, confidence: float = 0.9,
              signal: str | None = None) -> dict:
    return {"class": cls, "confidence": confidence,
            "box": {"x": cx - 0.05, "y": 0.4, "w": 0.1, "h": h},
            "signal": signal, "signalCounts": {}}


def vision(*, detections: dict[int, list] | None = None,
           field: list | None = None,
           traffic: dict | None = None,
           running: bool = True) -> dict:
    cameras = {}
    for cid in range(4):
        cameras[str(cid)] = {"id": cid, "active": True, "ageS": 0.1,
                             "width": 640, "height": 360, "frameStamp": 99.9,
                             "detections": (detections or {}).get(cid, []), "error": None}
    return {"ok": True, "enabled": True, "running": running,
            "cameras": cameras, "counts": {},
            "trafficLight": traffic or {"state": "unknown", "fresh": False,
                                        "counts": {"red": 0, "yellow": 0, "green": 0}},
            "field": field or []}


class FakeKinect:
    """Навигационный кадр Kinect: preview() вызывать категорически нельзя."""

    def __init__(self, *, nav_fresh: bool = True, frame_id: int = 7,
                 values=(1.0, 1.0, 1.0, 1.0)):
        self.is_nav_fresh = nav_fresh
        self.frame_id = frame_id
        self.frame = SimpleNamespace(width=2, height=2, values=list(values),
                                     fx=100.0, fy=100.0, stamp=10.0)
        self.nav_frames = 0

    @property
    def spec(self):
        return {"x_m": 0.35, "y_m": 0.0, "z_m": 0.45, "yaw_deg": 0.0,
                "pitch_deg": 0.0, "hfov_deg": 57.0, "vfov_deg": 43.0,
                "min_range_m": 0.45, "max_range_m": 4.0}

    def fresh(self, _age=None):
        return self.is_nav_fresh  # старый API: «points» тоже может быть свежим

    def navigation_fresh(self, _age=None, *, allow_demo=True):
        return self.is_nav_fresh

    def navigation_frame(self):
        self.nav_frames += 1
        return {"ok": True, "frameId": self.frame_id, "frame": self.frame,
                "spec": self.spec, "notForNavigation": False}

    def preview(self):
        raise AssertionError("preview() помечен notForNavigation: True")


def gate_vision():
    return vision(detections={0: [detection("cone", 0.08, 0.22)],
                              2: [detection("cone", 0.92, 0.22)]})


def fake_motors(**fields):
    motor = dict(online=True, scCount=0, scPpr=90, scRecent=False, scRpm=0.0,
                 drivePercent=0, angle=0.0, moving=False, telemetryAgeS=0.01, firmware=24,
                 homed=True, brakeId=3)
    return [dict(motor, id=role) | fields for role in ("FL", "FR", "RL", "RR")]


class FakeSource:
    supports_drive = True
    supports_individual_steer = True

    def __init__(self):
        self.brake_id = 3
        self.brakes = 0

    def read(self):
        return {"motors": fake_motors(brakeId=self.brake_id), "wheelSpeedMps": 0.0, "linkOk": True}

    def brake_for_keyboard(self):
        self.brake_id += 1
        self.brakes += 1
        return {"ok": True, "brakeId": self.brake_id}


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def advance(self, dt: float):
        self.t += dt


def perception(cones=None, signs=None, traffic=None, ground=None, imu=None, fresh=True):
    return {
        "ok": True, "fresh": fresh, "freshCameras": 4,
        "cones": cones if cones is not None else [],
        "signs": signs or [],
        "traffic": traffic or {"visible": False, "state": "none", "hold": False, "fresh": True},
        "ground": ground or {"ok": False, "nearestBumpM": None},
        "imu": imu or {"fresh": True, "pitch": 0.0, "roll": 0.0, "gyroZ": 0.0},
        "telemetry": {"linkOk": True, "onlineWheels": 4, "wheelSpeedMps": 0.0},
    }


def service_with_perception(script, *, clock: FakeClock | None = None, **kwargs):
    clock = clock or FakeClock()
    service = TrackAutonomyService(
        config=replace(cfg(), simulation=True, require_depth=False,
                       require_imu=False, require_blind_check=False),
        perception_provider=lambda _now: script(),
        telemetry_provider=lambda: {"motors": [], "wheelSpeedMps": 0.0},
        source_provider=lambda: FakeSource(), estop=lambda: None,
        clock=clock, log_events=False, **kwargs)
    service._session = service._new_session(clock())
    return service, clock


# -------------------------------------------------------------- геометрия ---
def test_camera_specs_and_cone_projection():
    specs = camera_specs(layout())
    assert specs[0]["yaw_deg"] == -25.0 and specs[1]["yaw_deg"] == -135.0
    # Один и тот же конус левее центра кадра: перед-Л и краевая камера дают
    # разные пеленги, но обе точки конечны и впереди/сбоку.
    markers = cone_points(vision(detections={0: [detection("cone", 0.25, 0.2)],
                                             1: [detection("cone", 0.75, 0.2)]}),
                          layout(), cfg())
    assert len(markers) == 2
    assert all(0.3 <= m["distance"] <= 6.0 for m in markers)
    assert all(math.isfinite(m["x"]) and math.isfinite(m["y"]) for m in markers)


def test_corridor_center_and_left_right_turns():
    straight = estimate_corridor(straight_cones(), cfg())
    assert straight["ok"] and abs(straight["centerM"]) < 0.05
    assert abs(straight["steerLeftDeg"]) < 1.0
    # Доступный крабовый проезд, у которого оба столбика с одной стороны оси.
    for sign in (-1, 1):
        gate = estimate_corridor([cone(3.4, sign * 0.3), cone(3.4, sign * 1.9)], cfg())
        assert gate["ok"] and sign * gate["steerLeftDeg"] > 5
    unreachable = estimate_corridor([cone(0.8, 0.6), cone(0.8, 2.2)], cfg())
    assert not unreachable["ok"] and unreachable["reason"] == "unreachable_gap"


def test_corridor_rejects_narrow_missing_and_far_cones():
    assert estimate_corridor([], cfg())["reason"] == "cones_missing"
    assert estimate_corridor([cone(1.0, -0.2), cone(1.0, 0.2)], cfg())["reason"] == "corridor_narrow"
    assert estimate_corridor([cone(5.8, -0.8), cone(5.8, 0.8)], cfg())["reason"] == "cones_far"
    one_side = estimate_corridor([cone(1.0, 1.0), cone(2.0, 1.0)], cfg())
    assert not one_side["ok"]
    assert not estimate_corridor([cone(1.0, -0.8), cone(2.0, 0.8)], cfg())["ok"]
    assert not estimate_corridor([cone(1.0, -2), cone(1.0, 2)], cfg())["ok"]


def test_gap_fits_full_chassis_and_does_not_ignore_solo_post():
    gate = [cone(1.1, -0.65), cone(1.1, 0.65)]
    assert estimate_corridor(gate, cfg())["ok"]
    assert estimate_corridor(gate, cfg(robot_width_m=1.0))["reason"] == "corridor_narrow"
    assert not estimate_corridor(straight_cones() + [cone(1.25, 0.0)], cfg())["ok"]
    assert not estimate_corridor(gate + [cone(0.09, 0.0)], cfg())["ok"]
    assert not estimate_corridor([cone(1.0, -0.46), cone(1.0, 0.46)] +
                                 [cone(2.5, -0.9), cone(2.5, 0.9)], cfg())["ok"]


def test_scan_clearance_rejects_api_or_incomplete_scans_and_checks_crab_path(monkeypatch):
    monkeypatch.setenv("RC_LIDAR_OFFSET_M", "0.28")
    info = ScriptedWorld.scan(0.5)
    clear = lidar_clearance(info, 0, cfg())
    assert clear["ready"] and not clear["blocked"] and clear["nearestM"] is None
    assert clear["points"] == 45  # only forward angles, not the entire revolution
    assert clear["nearestAheadM"] == pytest.approx(6.91)
    assert clear["nearestPoint"] == {"x": 7.28, "y": 0.0}
    assert not lidar_clearance(dict(info, trusted=False), 0, cfg())["ready"]
    assert not lidar_clearance(dict(info, points=info["points"][:30]), 0, cfg())["ready"]
    obstacle = {"a": math.atan2(0.58, 0.72), "r": math.hypot(0.58, 0.72)}  # chassis (1.0, 0.58)
    info["points"].append(obstacle)
    assert not lidar_clearance(info, 0, cfg())["blocked"]
    assert lidar_clearance(info, 22, cfg())["blocked"]


def test_forward_lidar_sector_does_not_require_a_full_revolution():
    from robot_control.navigation import _scan_coverage

    info = ScriptedWorld("straight").scan_ahead(0.5)
    assert lidar_clearance(info, 0, cfg())["ready"]
    assert not _scan_coverage(info["points"])[0]  # map navigation stays stricter
    assert _scan_coverage(ScriptedWorld.scan(0.5)["points"])[0]


@pytest.mark.parametrize("kind", ["rear_only", "front_hole", "missing_edges", "sparse", "duplicates", "invalid"])
def test_forward_lidar_sector_rejects_blind_or_invalid_coverage(kind):
    info = ScriptedWorld("straight").scan_ahead(0.5)
    if kind == "rear_only":
        info["points"] = [{"a": math.radians(a), "r": 7.0} for a in range(91, 270, 3)]
    elif kind == "front_hole":
        info["points"] = [p for p in info["points"] if abs(p["a"]) > math.radians(21)]
    elif kind == "missing_edges":
        info["points"] = [p for p in info["points"] if abs(p["a"]) < math.radians(55)]
    elif kind == "sparse":
        info["points"] = info["points"][::3]
    elif kind == "duplicates":
        info["points"] = info["points"][::3] * 5
    else:
        info["points"] = [None, {}, {"a": 0, "r": float("nan")},
                          {"a": float("inf"), "r": 2.0}, {"a": 0, "r": -1.0}]
    result = lidar_clearance(info, 0, cfg())
    assert not result["ready"]
    assert result["nearestAheadM"] is None and result["nearestPoint"] is None


def test_sign_marks_front_side_and_fallback():
    field = [
        {"class": "stop_sign", "cameraId": 0, "x_m": 2.0, "y_m": 1.0,
         "distanceM": 2.2, "ageS": 0.1, "confidence": 0.9, "signal": None},
        {"class": "stop_sign", "cameraId": 1, "x_m": -0.2, "y_m": 1.0,
         "distanceM": 1.0, "ageS": 0.2, "confidence": 0.9, "signal": None},
    ]
    marks = sign_marks(vision(field=field), layout(), cfg())
    assert [m["side"] for m in marks] == ["front", "side"]
    # Без field работают рамки: краевая камера 1 даёт «side».
    fallback = sign_marks(vision(detections={1: [detection("stop_sign", 0.5, 0.3)]}),
                          layout(), cfg())
    assert len(fallback) == 1 and fallback[0]["side"] == "side"


def test_traffic_light_only_current_green_allows_motion():
    base = {"state": "green", "fresh": True, "ageS": 0.1}
    light = {"class": "traffic_light", "side": "side", "signal": "green"}
    green = dict(detection("traffic_light", 0.5, 0.4, signal="green"), signalCounts={"green": 42})
    picture = vision(traffic=base, detections={2: [green]})
    decision = traffic_decision(picture, [light], cfg())
    assert decision["hold"] is False and decision["frameStamp"] == 99.9
    assert traffic_decision(vision(traffic=base), [light], cfg())["hold"]
    # Tracker may still show green after the physical light changed.
    for state in ("red", "yellow", "unknown"):
        picture["cameras"]["2"]["detections"] = [dict(green, signal=state)]
        assert traffic_decision(picture, [light], cfg())["hold"]
    picture["cameras"]["2"]["detections"] = [dict(green, signalCounts={"red": 12, "green": 42})]
    assert traffic_decision(picture, [light], cfg())["state"] == "red"
    picture["cameras"]["2"]["detections"] = [green]
    picture["cameras"]["2"]["captureAgeS"] = 2.0
    assert traffic_decision(picture, [light], cfg())["hold"]


# ------------------------------------------------------------------ привод ---
def test_motion_waits_for_current_brake_and_steering_ack():
    clock, src = FakeClock(), FakeSource()
    sent = []
    motion = TrackMotion(lambda: src,
                         lambda _s, body: (sent.append(("drive", body)) or {"ok": True}, 200),
                         lambda _s, body: (sent.append(("steer", body)) or {"ok": True}, 200),
                         cfg(steer_settle_s=0.4), simulation=False, clock=clock)
    old = chassis_feedback(src.read())
    assert motion.command(8.0, 10.0, old) is False
    assert src.brakes == 1 and sent == []
    still_moving = src.read()
    still_moving["motors"][0]["scRecent"] = True
    assert not motion.command(8.0, 10.0, chassis_feedback(still_moving))
    assert not motion.command(8.0, 10.0, old) and sent == []
    assert not motion.command(8.0, 10.0, chassis_feedback(src.read()))
    assert [item[1]["angle"] for item in sent] == [-10.0] * 4
    clock.advance(0.5)
    assert not motion.command(8.0, 10.0, chassis_feedback(src.read()))  # time isn't angle confirmation
    aligned = dict(src.read(), motors=fake_motors(brakeId=src.brake_id, angle=-10.0))
    assert motion.command(8.0, 10.0, chassis_feedback(aligned))
    assert sent[-1] == ("drive", {"percent": 8.0})
    assert motion.command(8.0, 10.0, chassis_feedback(aligned))
    assert sent[-1] == sent[-2]  # watchdog heartbeat


def test_motion_hold_clears_pending_steering_on_all_four_port_queues():
    from robot_control.slam import load_backend

    # Use the real backend command queue but construct it without workers or ports.
    backend = load_backend()
    source = backend.MasterSource.__new__(backend.MasterSource)
    source._commands_lock, source.lock = threading.RLock(), threading.RLock()
    source.serial_ready = threading.Event()
    source.serial_ready.set()
    source.commands = queue.Queue()
    source.links = {m: SimpleNamespace(module=m, commands=queue.Queue()) for m in ("FL", "FR", "RL", "RR")}
    for link in source.links.values():
        link.commands.put_nowait(b"w 10\n")
    source.commands.put_nowait(b"t 8\n")

    def unexpected(_source, _body):
        pytest.fail("a hold must not enqueue drive or steer commands")

    motion = TrackMotion(lambda: source, unexpected, unexpected, cfg())
    motion.steering_at = motion.clock()
    motion.hold()
    assert motion.steering_at is None and motion.percent == 0.0 and motion.brake_id is not None
    assert source.commands.empty()
    for link in source.links.values():
        assert link.commands.get_nowait() == f"s q\nb {motion.brake_id}\n".encode()
        assert link.commands.empty()


def test_motion_drive_refusal_is_not_swallowed():
    src = FakeSource()
    motion = TrackMotion(lambda: src,
                         lambda _s, _body: ({"ok": False, "reason": "estop"}, 409),
                         lambda _s, _body: ({"ok": True}, 200), cfg())
    motion.stop()
    with pytest.raises(TrackAutonomyError, match="estop") as caught:
        motion.command(8.0, 0.0, chassis_feedback(src.read()))
    assert caught.value.code == "drive_refused"


def test_odometer_does_not_invent_distance():
    telemetry = {"motors": [{"id": name, "online": True, "scCount": 0, "scPpr": 90}
                            for name in ("FL", "FR", "RL", "RR")],
                 "wheelSpeedMps": 0.0}
    odo = Odometer(lambda: telemetry)
    assert odo.update(0.2) == 0.0
    for motor in telemetry["motors"]:
        motor["scCount"] = 90  # один оборот колеса каждого модуля
    delta = odo.update(0.2)
    assert delta == pytest.approx(math.pi * 0.2159, rel=1e-6)
    assert odo.source == "sc"
    empty = Odometer(lambda: {})
    empty.update(0.2)
    assert empty.distance == 0.0 and empty.source == "none"


def test_downsample_depth_frame_keeps_optics_in_metres():
    full = SimpleNamespace(width=240, height=180,
                           values=[1.0] * (240 * 180),
                           fx=240.0, fy=180.0, stamp=12.0)
    spec = {"hfov_deg": 57.0, "vfov_deg": 43.0}
    small = _downsample_depth_frame(full, spec)
    assert small.width == 120 and small.height == 90
    assert len(small.values) == 120 * 90
    assert small.fx == pytest.approx(120.0) and small.fy == pytest.approx(90.0)
    # Без fx/fy в кадре оптика берётся из калибровки и тоже делится на stride.
    inferred = _downsample_depth_frame(SimpleNamespace(
        width=120, height=90, values=[1.0] * (120 * 90), fx=0.0, fy=0.0), spec)
    assert inferred.fx > 0 and inferred.fy > 0
    assert _downsample_depth_frame(SimpleNamespace(
        width=0, height=0, values=[], fx=0.0, fy=0.0), spec) is None


def test_bump_detector_uses_navigation_frame_not_preview(monkeypatch):
    import robot_control.track_autonomy as ta
    calls = []

    def counted(frame, spec, **_kwargs):
        calls.append(frame)
        return []  # плоскость не набрала клеток → unknown, а не «ровно»

    monkeypatch.setattr(ta, "ground_points_from_depth", counted)
    kinect = FakeKinect()
    detector = BumpDetector()
    first = detector.update(kinect, {"pitch": 0.0}, 1.0, cfg())
    assert first["ok"] is False and not first["depthFresh"] and first["status"] == "unknown"
    assert len(calls) == 1 and kinect.nav_frames == 1
    # Повтор того же frameId не должен набирать confirm_frames обманом.
    detector.update(kinect, {"pitch": 0.0}, 1.2, cfg())
    assert len(calls) == 1
    kinect.frame_id = 8
    detector.update(kinect, {"pitch": 0.0}, 1.4, cfg())
    assert len(calls) == 2


def test_kinect_navigation_frame_requires_raw_depth():
    clock = FakeClock()
    service = KinectDepthService(clock=clock)
    assert not service.navigation_fresh(1.5)
    service.feed({"width": 2, "height": 2, "depth": [1.0, 1.0, 1.0, 1.0],
                  "units": "m", "stamp": clock()})
    assert service.navigation_fresh(1.5)
    snap = service.navigation_frame()
    assert snap["frameId"] == 1 and snap["notForNavigation"] is False
    assert list(snap["frame"].values) == [1.0] * 4
    assert kinect_navigation_ready(service, cfg())
    # points/ranges дают свежий псевдоскан, но не тот кадр, по которому
    # считается рельеф: fresh() True, navigation_fresh() False.
    service.feed({"points": [{"a": 0.0, "r": 1.0}], "stamp": clock() + 0.1})
    clock.advance(0.1)
    assert service.fresh(1.5) is True
    assert service.navigation_fresh(1.5) is False
    assert kinect_navigation_ready(service, cfg()) is False

    service.feed({"width": 2, "height": 2, "depth": [1.0] * 4, "units": "m",
                  "stamp": clock(), "capture_source": "demo"})
    assert service.navigation_frame()["captureSource"] == "demo"
    assert not kinect_navigation_ready(service, cfg())
    assert kinect_navigation_ready(service, cfg(simulation=True))


def test_bump_detector_empty_depth_is_unknown_not_flat():
    detector = BumpDetector()
    result = detector.update(None, {"pitch": 0.0}, 1.0, cfg())
    assert result["ok"] is False and result["nearestBumpM"] is None
    kinect = FakeKinect(nav_fresh=False)
    assert detector.update(kinect, {}, 2.0, cfg())["status"] == "unknown"


# -------------------------------------------------------------- предполёт ---
def test_preflight_lists_all_blockers_and_allows_clean_start():
    fake_vision = gate_vision()
    base = dict(config=replace(cfg(enabled=True, require_depth=True), simulation=True),
                vision_provider=lambda: fake_vision,
                layout_provider=layout,
                kinect_provider=lambda: FakeKinect(),
                imu_provider=lambda: {"ok": True, "stale": False, "pitch": 0.0},
                source_provider=lambda: FakeSource(),
                telemetry_provider=lambda: FakeSource().read(),
                estop=lambda: None, log_events=False)
    service = TrackAutonomyService(**base)
    # status() опрашивает готовность без галочки: она не должна навечно
    # отключать кнопку, но сам POST /start её всё равно требует.
    status_checks = service.preflight()
    assert status_checks["canStart"] is True
    assert status_checks["requireBlindCheck"] is True
    blocked = service.preflight(for_start=True)
    assert blocked["canStart"] is False
    assert any("галочку" in item for item in blocked["blockers"])
    clean = service.preflight(blind_zone_checked=True, for_start=True)
    assert clean["canStart"] is True, clean["blockers"]
    assert clean["checks"]["conesVisible"]
    assert not clean["checks"]["corridorOk"] and clean["preview"]["corridor"]["required"] is False
    # E-STOP и чужой режим тяги блокируют пуск, даже если всё остальное готово.
    service._estop = lambda: {"reason": "оператор"}
    assert service.preflight(blind_zone_checked=True)["checks"]["estopClear"] is False
    service._estop = lambda: None
    service._rivals = lambda: ["навигация по карте"]
    assert any("владеет приводом" in item
               for item in service.preflight(blind_zone_checked=True)["blockers"])


def test_default_enabled_means_manual_start_only(monkeypatch):
    monkeypatch.delenv("RC_TRACK_ENABLE", raising=False)
    assert TrackConfig().enabled and TrackConfig.from_env().enabled
    source = FakeSource()
    service = TrackAutonomyService(
        vision_provider=vision, layout_provider=lambda: dict(layout(), calibrated=False),
        kinect_provider=FakeKinect, lidar_provider=ScriptedWorld("straight").scan_ahead,
        source_provider=lambda: source, telemetry_provider=source.read,
        estop=lambda: None, log_events=False)
    for _ in range(2):
        status = service.status()
        assert status["enabled"] and status["canStart"], status["blockers"]
        assert status["navigationMode"] == "forward"
        assert not status["active"] and status["phase"] == "idle"
        assert not status["checks"]["conesVisible"] and not status["checks"]["geometryCalibrated"]
        assert status["warnings"] and status["corridor"]["required"] is False
    assert service._session is None and service.motion.command_count == 0
    assert service.motion.percent == 0 and source.brakes == 0
    monkeypatch.setenv("RC_TRACK_ENABLE", "0")
    assert TrackConfig.from_env().enabled is False


@pytest.mark.parametrize("failure", ["depth", "lidar", "ports"])
def test_approximate_geometry_does_not_waive_required_sensors_or_camera_ports(failure):
    source = FakeSource()
    service = TrackAutonomyService(
        config=cfg(require_depth=False, require_blind_check=False),
        vision_provider=vision, layout_provider=lambda: dict(layout(), calibrated=False),
        kinect_provider=FakeKinect, lidar_provider=ScriptedWorld("straight").scan_ahead,
        source_provider=lambda: source, telemetry_provider=source.read,
        estop=lambda: None, log_events=False)
    assert service.preflight(blind_zone_checked=True, for_start=True)["canStart"]
    if failure == "depth":
        service._kinect = lambda: FakeKinect(nav_fresh=False)
        key = "depthFresh"
    elif failure == "lidar":
        service._lidar = lambda _age: {"trusted": False, "points": ScriptedWorld.scan(0.5)["points"]}
        key = "lidarReady"
    else:
        service._layout = lambda: dict(layout(), calibrated=False, duplicatePorts=["/dev/video0"])
        key = "cameraPortsDistinct"
    status = service.preflight(blind_zone_checked=True, for_start=True)
    assert not status["canStart"] and not status["checks"][key] and status["blockers"]
    assert not any("Откалибруйте" in b for b in status["blockers"])
    assert service.motion.command_count == 0


def test_preflight_requires_four_wheels_and_live_link_for_real_drive():
    fake_vision = gate_vision()

    def service_with_motors(motors, link_ok=True):
        return TrackAutonomyService(
            config=replace(cfg(enabled=True), simulation=False, require_depth=False,
                           require_imu=False, require_blind_check=False),
            vision_provider=lambda: fake_vision, layout_provider=layout,
            kinect_provider=lambda: FakeKinect(), lidar_provider=ScriptedWorld.scan,
            imu_provider=lambda: {"ok": True, "stale": False},
            source_provider=lambda: FakeSource(),
            telemetry_provider=lambda: {"motors": motors, "wheelSpeedMps": 0.0,
                                        "linkOk": link_ok},
            estop=lambda: None, log_events=False)

    two = [{"id": name, "online": True, "scCount": 0, "scPpr": 90}
           for name in ("FL", "FR")]
    blocked = service_with_motors(two).preflight()
    assert blocked["checks"]["chassisLink"] is True
    assert blocked["checks"]["wheelsOnline"] is False
    assert any("Не все четыре колеса онлайн: 2 из 4" in item
               for item in blocked["blockers"])
    no_link = service_with_motors(two * 2, link_ok=False).preflight()
    assert any("связи с платами колёс" in item for item in no_link["blockers"])

    four = fake_motors()
    clean = service_with_motors(four).preflight(blind_zone_checked=True, for_start=True)
    assert clean["canStart"], clean["blockers"]
    assert clean["checks"]["chassisLink"] is True
    assert clean["checks"]["wheelsOnline"] is True
    assert not any("колеса онлайн" in item or "связи с платами" in item
                   for item in clean["blockers"])


def test_start_requires_blind_check_and_enabled(monkeypatch):
    fake_vision = vision(detections={0: [detection("cone", 0.15, 0.2)],
                                     2: [detection("cone", 0.85, 0.2)]})
    service = TrackAutonomyService(
        config=replace(cfg(enabled=False), simulation=True),
        vision_provider=lambda: fake_vision, layout_provider=layout,
        kinect_provider=lambda: FakeKinect(), source_provider=lambda: FakeSource(),
        estop=lambda: None, log_events=False)
    with pytest.raises(TrackAutonomyError) as caught:
        service.start({"blindZoneChecked": True})
    assert caught.value.code == "not_ready"
    assert "RC_TRACK_ENABLE" in caught.value.message


# ------------------------------------------------------- состояния автомата ---
def test_stop_sign_waits_three_seconds_and_resumes():
    clock = FakeClock()

    def front():
        return perception(signs=[{"class": "stop_sign", "x": 2.0, "y": 1.0,
                                  "distance": 2.2, "side": "front", "cameraId": 0,
                                  "confidence": 0.9}])
    service, clock = service_with_perception(front, clock=clock)
    service._step(clock())
    assert service._state["phase"] == "following"
    # Знак перешёл в краевую камеру — стоп возле знака.
    service._perception = lambda _now: perception(signs=[
        {"class": "stop_sign", "x": 0.1, "y": 1.0, "distance": 1.0,
         "side": "side", "cameraId": 1, "confidence": 0.9}])
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "stop_braking"
    assert service.motion.percent == 0.0
    clock.advance(0.5)
    service._step(clock())
    assert service._state["phase"] == "stop_wait"
    clock.advance(2.9)
    service._step(clock())
    assert service.motion.percent == 0.0
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "following"
    assert service._state["counters"]["stop"] == 1


def test_traffic_red_yellow_green_sequence_replay_and_loss():
    states = {"current": "red", "frame": 90.0, "visible": True}

    def script():
        green = states["current"] == "green"
        return perception(traffic={"visible": states["visible"], "state": states["current"],
                                    "hold": not green, "fresh": True, "frames": {1: states["frame"]},
                                    "cameraId": 1 if green else None,
                                    "frameStamp": states["frame"] if green else None}, signs=[
            {"class": "traffic_light", "x": 1.5, "y": 1.2, "distance": 1.6,
             "side": "side", "signal": states["current"], "cameraId": 1}])

    service, clock = service_with_perception(script)

    def click(color, stamp, expected):
        states.update(current=color, frame=stamp)
        clock.advance(0.2)
        service._step(clock())
        assert service._state["phase"] == expected
        assert (service.motion.percent == 0.0) == (expected == "traffic_wait")

    click("red", 90.0, "traffic_wait")
    click("yellow", 90.1, "traffic_wait")
    click("green", 90.2, "traffic_wait")
    click("green", 90.2, "traffic_wait")  # same frame is not another vote
    click("green", 90.3, "following")
    click("red", 90.4, "traffic_wait")  # no cooldown can suppress a new red
    click("green", 90.2, "traffic_wait")
    click("green", 90.3, "traffic_wait")  # old frames arriving in order are still replays
    click("green", 90.5, "traffic_wait")
    click("green", 90.6, "following")
    states["visible"] = False
    click("green", 90.7, "traffic_wait")
    states["visible"] = True
    click("green", 90.6, "traffic_wait")
    click("green", 90.8, "traffic_wait")
    states["visible"] = False
    clock.advance(4.0)
    service._step(clock())
    assert service._state["phase"] == "error" and service.motion.percent == 0


def test_bump_slows_adds_power_by_gyro_then_slows_again():
    clock = FakeClock()
    world = {"bump": 1.4, "pitch": 0.0, "stall": False}
    script = lambda: perception(
        signs=[{"class": "speed_bump", "x": world["bump"], "y": 1.0,
                "distance": world["bump"], "side": "front", "confidence": 0.9}],
        ground={"ok": True, "nearestBumpM": world["bump"], "bumpFarM": world["bump"] + 0.75},
        imu={"fresh": True, "pitch": world["pitch"], "roll": 0.0, "gyroZ": 0.0})
    service, clock = service_with_perception(script, clock=clock)
    service._step(clock())
    assert service._state["phase"] == "bump_slow"
    assert service._state["percent"] == pytest.approx(4.0)
    world["bump"] = 0.2
    world["pitch"] = 9.0
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "bump_climb"
    clock.advance(0.9)
    service._step(clock())
    assert service._state["counters"]["powerBoosts"] >= 1
    assert service._state["percent"] > 4.0
    # «Проехали»: имитируем прирост одометрии (счётчики SC).
    service._odometer.distance = 1.2
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "bump_climb"  # rear of the body has not cleared the far edge
    service._odometer.distance = 1.6
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "bump_exit"
    assert service._state["boostPercent"] == 0.0
    assert service._state["percent"] == pytest.approx(4.0)


def test_missing_or_unpaired_posts_do_not_gate_motion_or_steer():
    service, clock = service_with_perception(lambda: perception(cones=straight_cones()))
    for posts in (straight_cones(), [], [cone(3.4, 1.5)], [], []):
        service._perception = lambda _now: perception(cones=posts, imu={"fresh": True, "gyroZ": 30.0})
        clock.advance(2.0)  # longer than the former lost-corridor timeout
        service._step(clock())
        assert service._state["phase"] == "following" and not service._session["finished"]
        assert service.motion.percent == service.cfg.cruise_percent and service.motion.steer_deg == 0
        assert service._state["corridor"]["required"] is False


def test_camera_loss_and_estop_still_stop_the_loop():
    service, clock = service_with_perception(lambda: perception())
    service._step(clock())
    assert service.motion.percent > 0
    service._perception = lambda _now: perception(fresh=False)
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "vision_lost" and service.motion.percent == 0
    clock.advance(2.0)
    service._step(clock())
    assert service._state["phase"] == "blocked" and service.motion.percent == 0
    assert "кадров" in service._state["message"].lower()

    service2, clock2 = service_with_perception(lambda: perception(), clock=FakeClock())
    service2._estop = lambda: {"reason": "кнопка"}
    service2._step(clock2())
    assert service2._state["phase"] == "error" and service2.motion.percent == 0
    assert "E-STOP" in service2._state["message"]


def test_bump_sign_without_kinect_confirmation_remains_stopped():
    service, clock = service_with_perception(lambda: perception(
        signs=[{"class": "speed_bump", "x": 0.4, "y": 0.9, "side": "front"}],
        ground={"ok": True, "nearestBumpM": None},
        imu={"fresh": True, "pitch": 9.0}))
    service._step(clock())
    assert service.motion.percent == 0 and service._state["phase"] == "bump_wait"
    clock.advance(service.cfg.bump_cross_timeout_s + 0.2)
    service._step(clock())
    assert service.motion.percent == 0 and service._state["phase"] == "error"
    assert service._state["counters"]["bump"] == service._state["counters"]["powerBoosts"] == 0


def test_bump_timeout_is_not_success_even_with_confirmed_depth():
    service, clock = service_with_perception(lambda: perception(
        signs=[{"class": "speed_bump", "x": 0.4, "y": 0.9, "side": "front"}],
        ground={"ok": True, "nearestBumpM": 0.4},
        imu={"fresh": True, "pitch": 0.0}))
    service._step(clock())
    assert service.motion.percent == 4 and service._state["phase"] == "bump_climb"
    clock.advance(service.cfg.bump_cross_timeout_s + 0.2)
    service._step(clock())
    assert service._state["phase"] == "error" and service.motion.percent == 0
    assert service._state["counters"]["bump"] == 0


def real_service_on_fakes():
    """Run physical-mode guards with explicit fake actuator and sensor providers."""
    clock, source = FakeClock(), FakeSource()
    feedback = source.read()
    sent = []
    service = TrackAutonomyService(
        config=cfg(enabled=True), lidar_provider=ScriptedWorld.scan,
        telemetry_provider=lambda: feedback, estop=lambda: None, source_provider=lambda: source,
        drive=lambda _src, body: (sent.append(body["percent"]) or {"ok": True}, 200),
        clock=clock, log_events=False)
    service._perceive = lambda _now: dict(perception(ground={"ok": True, "depthFresh": True}),
                                         geometryCalibrated=False)
    service._session = service._new_session(clock())
    service.motion.stop()
    feedback.update(source.read())  # initial tagged brake was applied
    return service, clock, source, feedback, sent


@pytest.mark.parametrize("failure", ["untrusted", "missing_scan", "front_gap", "obstacle",
                                     "wheel", "link", "overspeed", "depth", "ports"])
def test_real_run_brakes_on_live_failure(failure):
    service, clock, src, feedback, sent = real_service_on_fakes()
    service._step(clock())
    assert sent == [8.0]
    if failure == "untrusted":
        service._lidar = lambda age: dict(ScriptedWorld.scan(age), trusted=False)
    elif failure == "missing_scan":
        service._lidar = lambda _age: {}  # real provider returns no points on stale scans
    elif failure == "front_gap":
        service._lidar = lambda age: dict(ScriptedWorld.scan(age), points=ScriptedWorld.scan(age)["points"][:30])
    elif failure == "ports":
        service._perceive = lambda _now: dict(perception(ground={"depthFresh": True}), cameraPortsDistinct=False)
    elif failure == "link":
        feedback["linkOk"] = False
    elif failure == "obstacle":
        def blocked(age):
            scan = ScriptedWorld.scan(age)
            scan["points"].append({"a": 0.0, "r": 0.5})
            return scan
        service._lidar = blocked
    elif failure == "wheel":
        feedback["motors"][3]["telemetryAgeS"] = 1.0
    elif failure == "overspeed":
        # One wheel can exceed the limit even when the aggregate is small.
        feedback["motors"][3]["scRpm"] = 60.0
        feedback["wheelSpeedMps"] = 0.17
    else:
        service._perceive = lambda _now: dict(perception(), geometryCalibrated=True)
    clock.advance(0.2)
    service._step(clock())
    assert sent == [8.0] and src.brakes == 2
    # Препятствие впереди — пауза, потеря данных и отказы — конец поездки.
    expected = {"obstacle_hold"} if failure == "obstacle" else {"blocked", "error"}
    assert service.motion.percent == 0 and service._state["phase"] in expected


def test_real_forward_run_slows_then_pauses_before_obstacle_and_resumes(monkeypatch):
    monkeypatch.setenv("RC_LIDAR_OFFSET_M", "0.28")
    service, clock, src, _feedback, sent = real_service_on_fakes()
    world = ScriptedWorld("straight")
    service._lidar = world.scan_ahead  # no full revolution and no posts
    service._step(clock())
    assert sent == [8.0]

    def approaching(_age):
        scan = world.scan_ahead(0.5)
        # Not just the centre ray: a point near the chassis' left edge counts.
        x, y = obstacle["x"] - 0.28, 0.40
        scan["points"].append({"a": math.atan2(y, x), "r": math.hypot(x, y)})
        return scan

    obstacle = {"x": 1.9}
    service._lidar = approaching
    clock.advance(0.2)
    service._step(clock())
    assert sent == [8.0, 4.0] and service._state["phase"] == "obstacle_approach"
    assert service._state["lidar"]["nearestAheadM"] == pytest.approx(1.52)
    assert not service._state["lidar"]["blocked"]
    obstacle["x"] = 1.1
    clock.advance(0.2)
    service._step(clock())
    # Препятствие ближе полуметра: тяга снята, но поездка не завершена.
    assert service.motion.percent == 0 and service._state["phase"] == "obstacle_hold"
    assert service._state["lidar"]["blocked"] and src.brakes == 2
    assert service._session["obstacle_since"] is not None
    assert not service._session["finished"]
    assert "Препятствие впереди" in service._state["message"]
    # Путь освободили, пауза вышла — поездка продолжается сама.
    service._lidar = world.scan_ahead
    clock.advance(6.0)
    _feedback.update(src.read())   # телеметрия опрашивается, а не замирает
    service._step(clock())
    assert service._state["phase"] != "obstacle_hold"
    assert service.motion.command_count > 2 and service.motion.percent > 0
    assert any(event["phase"] == "obstacle_clear" for event in service._session["events"])


@pytest.mark.parametrize("limit", ["distance", "time"])
def test_forward_run_keeps_distance_and_time_failsafes(limit):
    service, clock, src, _feedback, sent = real_service_on_fakes()
    service._step(clock())
    if limit == "distance":
        service._odometer.distance = service.cfg.max_distance_m
    else:
        clock.advance(service.cfg.max_duration_s + 1)
    service._step(clock())
    assert sent == [8.0] and src.brakes == 2 and service.motion.percent == 0
    assert service._state["phase"] == "error"


def test_stop_dwell_requires_new_brake_id_and_all_stopped():
    service, clock, src, feedback, sent = real_service_on_fakes()
    service._step(clock())
    sign = {"class": "stop_sign", "x": 0.4, "y": 1.0, "distance": 1.1, "side": "side"}
    service._perceive = lambda _now: dict(perception(signs=[sign], ground={"depthFresh": True}),
                                         geometryCalibrated=True)
    clock.advance(0.2)
    service._step(clock())  # a second, uniquely identified STOP
    clock.advance(3.1)
    service._step(clock())  # old brake ID is not acknowledgement
    assert service._state["phase"] == "stop_braking" and sent == [8.0]
    feedback.update(src.read())
    feedback["motors"][0]["scRecent"] = True
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "stop_braking"
    feedback["motors"][0]["scRecent"] = False
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "stop_wait"
    clock.advance(2.9)
    service._step(clock())
    assert sent == [8.0]
    clock.advance(0.2)
    service._step(clock())
    assert sent == [8.0, 8.0] and service._state["counters"]["stop"] == 1


def test_real_run_stops_when_a_wheel_does_not_move():
    service, clock, src, feedback, sent = real_service_on_fakes()
    service._step(clock())
    for motor in feedback["motors"]:
        motor["drivePercent"] = 8
    for _ in range(20):
        for motor in feedback["motors"][:3]:
            motor["scCount"] += 2
        clock.advance(0.2)
        service._step(clock())
    assert sent and src.brakes == 2 and service.motion.percent == 0
    assert "RR не подтвердило движение" in service._state["message"]


def test_interrupt_cannot_be_overtaken_by_a_tick_already_reading_sensors():
    service, clock, src, _feedback, sent = real_service_on_fakes()
    entered, release = threading.Event(), threading.Event()
    perceive = service._perceive

    def delayed(now):
        entered.set()
        assert release.wait(3.0)
        return perceive(now)

    service._perceive = delayed
    worker = threading.Thread(target=service._step, args=(clock(),))
    worker.start()
    try:
        assert entered.wait(1.0)
        service.interrupt("Стоп")
    finally:
        release.set()
        worker.join(3.0)
    assert not worker.is_alive() and sent == []
    assert src.brakes == 2 and service._state["phase"] == "cancelled"


# ------------------------------------------------------------- сухой прогон ---
@pytest.mark.parametrize("scenario", ["full", "straight", "stop", "bump", "light", "gate"])
def test_simulated_track_finishes(scenario):
    report = simulate_track(scenario)
    assert report["ok"] is True and report["passed"] is True
    assert report["phase"] == "done" and report["distanceM"] >= 40.0
    assert report["steps"] > 50


def test_full_simulation_covers_signs_bump_and_light():
    report = simulate_track("full")
    counters = report["counters"]
    assert counters["stop"] == 1 and counters["traffic"] == 1
    assert counters["bump"] == 1 and counters["powerBoosts"] >= 2
    assert report["maxBoostPercent"] >= 6.0
    phases = [event["phase"] for event in report["events"]]
    for expected in ("stop_wait", "bump_climb", "bump_exit", "traffic_wait", "done"):
        assert expected in phases
    # СТОП выдержан не меньше трёх секунд, светофор отпустил только на зелёном.
    stopped = [e for e in report["events"] if e["phase"] == "stop_wait"][0]
    resumed = [e for e in report["events"] if e["phase"] == "following"][0]
    assert resumed["atS"] - stopped["atS"] >= 3.0


def test_scripted_world_straight_has_no_posts_and_a_front_sector():
    world = ScriptedWorld("straight")
    snap = world.snapshot(0.0)
    assert snap["cones"] == [] and snap["fresh"] is True
    assert lidar_clearance(world.scan_ahead(0.5), 0, cfg())["ready"]
    world.advance(1.0, 8.0)
    assert world.s > 0 and world.y == 0


def test_simulated_dead_end_moves_then_holds_before_wall():
    """Стена впереди: поездка не «падает», а стоит на паузе у препятствия."""
    report = simulate_track("dead_end")
    assert report["passed"] and report["expectedStop"]
    assert report["phase"] == "obstacle_hold" and 0 < report["distanceM"] < 8.0
    assert report["finalPercent"] == 0 and "Препятствие впереди" in report["message"]
    phases = [event["phase"] for event in report["events"]]
    assert "obstacle_hold" in phases


def test_simulated_blocked_gate_never_moves():
    report = simulate_track("blocked_gate")
    assert report["passed"] and report["expectedStop"]
    assert report["phase"] == "obstacle_hold" and report["distanceM"] == 0
    assert report["finalPercent"] == 0
