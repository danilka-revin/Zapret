"""Логика вкладки «Автономность»: коридор, знаки, светофор, кочка, безопасность.

Проверяется сам модуль robot_control/track_autonomy.py без USB, Flask и модели:
восприятие подменяется словарями и маленькими фейками, как в других тестах
проекта (tests/test_maneuvers.py, tests/test_exploration.py).
"""
from __future__ import annotations

import math
from dataclasses import replace
from types import SimpleNamespace

import pytest

from robot_control.kinect import KinectDepthService
from robot_control.track_autonomy import (
    BumpDetector, Odometer, ScriptedWorld, TrackAutonomyError,
    TrackAutonomyService, TrackConfig, TrackMotion, _downsample_depth_frame,
    camera_specs, cone_points, estimate_corridor, kinect_navigation_ready,
    sign_marks, simulate_track, traffic_decision,
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
                             "width": 640, "height": 360,
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

    def navigation_fresh(self, _age=None):
        return self.is_nav_fresh

    def navigation_frame(self):
        self.nav_frames += 1
        return {"ok": True, "frameId": self.frame_id, "frame": self.frame,
                "spec": self.spec, "notForNavigation": False}

    def preview(self):
        raise AssertionError("preview() помечен notForNavigation: True")


class FakeSource:
    supports_drive = True
    supports_individual_steer = True

    def read(self):
        return {"motors": [], "wheelSpeedMps": 0.0, "linkOk": True}


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
        "cones": cones if cones is not None else straight_cones(),
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
    # Обе границы ушли влево (поворот трассы), а не «одна слева, одна справа».
    left_curve = [cone(x, 0.6 + 0.25 * x, 2) for x in (0.8, 1.6, 2.4, 3.2)] + \
                 [cone(x, 2.2 + 0.25 * x, 0) for x in (0.8, 1.6, 2.4, 3.2)]
    turned = estimate_corridor(left_curve, cfg())
    assert turned["ok"] and turned["steerLeftDeg"] > 5
    right_curve = [cone(x, -0.6 - 0.25 * x, 2) for x in (0.8, 1.6, 2.4, 3.2)] + \
                  [cone(x, 1.0 - 0.25 * x, 0) for x in (0.8, 1.6, 2.4, 3.2)]
    assert estimate_corridor(right_curve, cfg())["steerLeftDeg"] < -5


def test_corridor_rejects_narrow_missing_and_far_cones():
    assert estimate_corridor([], cfg())["reason"] == "cones_missing"
    assert estimate_corridor([cone(1.0, -0.2), cone(1.0, 0.2)], cfg())["reason"] == "corridor_narrow"
    assert estimate_corridor([cone(5.8, -0.8), cone(5.8, 0.8)], cfg())["reason"] == "cones_far"
    one_side = estimate_corridor([cone(1.0, 1.0), cone(2.0, 1.0)], cfg())
    assert one_side["ok"] and one_side["degraded"] is True


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


def test_traffic_light_only_green_allows_motion():
    base = {"state": "green", "fresh": True,
            "counts": {"red": 0, "yellow": 0, "green": 42}}
    light = [{"class": "traffic_light", "side": "side", "signal": "green"}]
    assert traffic_decision(vision(traffic=base), light, cfg())["hold"] is False
    for state in ("red", "yellow", "unknown"):
        payload = dict(base, state=state)
        assert traffic_decision(vision(traffic=payload), light, cfg())["hold"] is True
    # Малейший красный на светодиодном табло перебивает «зелёный».
    mixed = dict(base, counts={"red": 12, "yellow": 0, "green": 42})
    assert traffic_decision(vision(traffic=mixed), light, cfg())["state"] == "red"
    assert traffic_decision(vision(traffic=base), [], cfg())["hold"] is False


# ------------------------------------------------------------------ привод ---
def test_motion_brakes_before_steer_and_inverts_sign():
    clock = FakeClock()
    sent = []
    motion = TrackMotion(lambda: object(),
                         lambda _s, body: (sent.append(("drive", body)) or {"ok": True}, 200),
                         lambda _s, body: (sent.append(("steer", body)) or {"ok": True}, 200),
                         cfg(steer_settle_s=0.4), simulation=False, clock=clock)
    assert motion.command(8.0, 10.0) is False           # влево → угол прошивки -10°
    assert sent[0] == ("drive", {"percent": 0})
    assert all(item[1]["angle"] == -10.0 for item in sent if item[0] == "steer")
    clock.advance(0.5)
    assert motion.command(8.0, 10.0) is True
    assert sent[-1] == ("drive", {"percent": 8.0})
    # Без изменения угла тяга повторяется каждый такт (watchdog 650 мс).
    assert motion.command(8.0, 10.0) is True
    assert sent[-1] == ("drive", {"percent": 8.0})


def test_motion_drive_refusal_is_not_swallowed():
    motion = TrackMotion(lambda: object(),
                         lambda _s, _body: ({"ok": False, "reason": "estop"}, 409),
                         lambda _s, _body: ({"ok": True}, 200),
                         cfg(), simulation=False)
    with pytest.raises(TrackAutonomyError) as caught:
        motion.command(8.0, 0.0)
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
    assert first["ok"] is False and first["status"] == "unknown"
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


def test_bump_detector_empty_depth_is_unknown_not_flat():
    detector = BumpDetector()
    result = detector.update(None, {"pitch": 0.0}, 1.0, cfg())
    assert result["ok"] is False and result["nearestBumpM"] is None
    kinect = FakeKinect(nav_fresh=False)
    assert detector.update(kinect, {}, 2.0, cfg())["status"] == "unknown"


# -------------------------------------------------------------- предполёт ---
def test_preflight_lists_all_blockers_and_allows_clean_start():
    cones = {0: [detection("cone", 0.15, 0.2), detection("cone", 0.15, 0.12)],
             2: [detection("cone", 0.85, 0.2), detection("cone", 0.85, 0.12)]}
    fake_vision = vision(detections=cones)
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
    assert clean["checks"]["conesVisible"] and clean["checks"]["corridorOk"]
    # E-STOP и чужой режим тяги блокируют пуск, даже если всё остальное готово.
    service._estop = lambda: {"reason": "оператор"}
    assert service.preflight(blind_zone_checked=True)["checks"]["estopClear"] is False
    service._estop = lambda: None
    service._rivals = lambda: ["навигация по карте"]
    assert any("владеет приводом" in item
               for item in service.preflight(blind_zone_checked=True)["blockers"])


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
    script = {"phase": "front"}
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
    assert service._state["phase"] == "stop_wait"
    assert service.motion.percent == 0.0
    clock.advance(3.1)
    service._step(clock())
    assert service._state["phase"] == "following"
    assert service._state["counters"]["stop"] == 1


def test_traffic_red_yellow_green_sequence():
    clock = FakeClock()
    states = {"current": "red"}
    script = lambda: perception(traffic={"visible": True, "state": states["current"],
                                         "hold": states["current"] != "green",
                                         "fresh": True}, signs=[
        {"class": "traffic_light", "x": 1.5, "y": 1.2, "distance": 1.6,
         "side": "side", "signal": states["current"]}])
    service, clock = service_with_perception(script, clock=clock)
    service._step(clock())
    assert service._state["phase"] == "traffic_wait"
    states["current"] = "yellow"
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "traffic_wait"
    states["current"] = "green"
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "following"


def test_bump_slows_adds_power_by_gyro_then_slows_again():
    clock = FakeClock()
    world = {"bump": 1.4, "pitch": 0.0, "stall": False}
    script = lambda: perception(
        signs=[{"class": "speed_bump", "x": world["bump"], "y": 1.0,
                "distance": world["bump"], "side": "front", "confidence": 0.9}],
        ground={"ok": True, "nearestBumpM": world["bump"]},
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
    assert service._state["counters"]["powerBoosts"] >= 1
    assert service._state["percent"] > 4.0
    # «Проехали»: имитируем прирост одометрии (счётчики SC).
    service._odometer.distance = 1.2
    clock.advance(0.2)
    service._step(clock())
    assert service._state["phase"] == "bump_exit"
    assert service._state["boostPercent"] == 0.0
    assert service._state["percent"] == pytest.approx(4.0)


def test_lane_lost_and_estop_stop_the_loop():
    clock = FakeClock()
    service, clock = service_with_perception(lambda: perception(cones=[], fresh=True),
                                             clock=clock)
    service._step(clock())
    assert service._state["phase"] == "lane_lost"
    clock.advance(2.0)
    service._step(clock())
    assert service._state["phase"] == "error"
    assert "потерян" in service._state["message"].lower()
    assert service.motion.percent == 0.0

    service2, clock2 = service_with_perception(lambda _now: perception(), clock=FakeClock())
    service2._estop = lambda: {"reason": "кнопка"}
    service2._step(clock2())
    assert service2._state["phase"] == "error"
    assert "E-STOP" in service2._state["message"]


# ------------------------------------------------------------- сухой прогон ---
@pytest.mark.parametrize("scenario", ["full", "straight", "stop", "bump", "light"])
def test_simulated_track_finishes(scenario):
    report = simulate_track(scenario)
    assert report["ok"] is True
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


def test_scripted_world_track_can_turn_both_ways():
    world = ScriptedWorld("straight")
    assert world._center(4.0) == 0.0
    assert world._center(12.0) > 0.0    # левая дуга
    assert world._center(24.0) < 0.0   # правая дуга
    snap = world.snapshot(0.0)
    assert len(snap["cones"]) >= 8 and snap["fresh"] is True
