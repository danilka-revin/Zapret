"""Исполнитель навигации целиком: движение, обход препятствия и продолжение.

Настоящий контур ``NavigationService._run`` прогоняется на фальшивом шасси:
карта — обычная occupancy grid в памяти, лидар отдаёт точки от мировых
препятствий относительно текущей позы, а ``MasterBridge`` подменён мостом,
который двигает позу и ведёт себя как заказано тестом (едет, стоит, уходит в
сторону). Реальный протокол моторов здесь не участвует — его проверяют
``tests/test_firmware.py`` и ``tests/test_usb_auto_discovery.py``.
"""
from __future__ import annotations

import math
import threading
import time

import pytest

from robot_control import navigation as nav
from robot_control.navigation import NavigationError, NavigationService, plan_grid_path, plan_route

LIDAR_OFFSET_M = 0.28  # лидар закреплён впереди центра корпуса, как на роботе
STEP_M = 0.1          # сколько метров проходит фальшивое шасси за один тик
CLOCK_STEP = 0.2      # сколько «секунд» длится один тик в подставных часах


class FakeClock:
    """Часы без сна: время двигает только шасси, тесты не ждут реальных секунд."""

    def __init__(self, step: float = CLOCK_STEP):
        self.now = 1000.0
        self.step = step

    def __call__(self) -> float:
        return self.now

    def tick(self) -> None:
        self.now += self.step


class FakeMap:
    """Occupancy grid в памяти и поза робота, которую двигает фальшивый мост."""

    def __init__(self, clock: FakeClock, width: int = 80, height: int = 80,
                 resolution: float = 0.1):
        self.clock = clock
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = {"x": -width * resolution / 2.0, "y": -height * resolution / 2.0}
        self._data = bytes(width * height)
        self._pose = [0.0, 0.0, 0.0]
        self._lock = threading.Lock()
        self.revision = 0

    def pose(self) -> tuple[float, float, float]:
        with self._lock:
            return tuple(self._pose)  # type: ignore[return-value]

    def set_pose(self, x: float, y: float, yaw: float = 0.0) -> None:
        with self._lock:
            self._pose = [float(x), float(y), float(yaw)]

    def move(self, dx: float, dy: float) -> None:
        with self._lock:
            self._pose[0] += float(dx)
            self._pose[1] += float(dy)

    def block(self, x0: float, x1: float, y0: float, y1: float) -> None:
        """Отметить прямоугольник мировых координат занятым (стена карты)."""
        data = bytearray(self._data)
        ix0, iy0 = self._index(x0, y0)
        ix1, iy1 = self._index(x1, y1)
        for iy in range(min(iy0, iy1), max(iy0, iy1) + 1):
            for ix in range(min(ix0, ix1), max(ix0, ix1) + 1):
                if 0 <= ix < self.width and 0 <= iy < self.height:
                    data[iy * self.width + ix] = 100
        with self._lock:
            self._data = bytes(data)

    def _index(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor((x - self.origin["x"]) / self.resolution)),
                int(math.floor((y - self.origin["y"]) / self.resolution)))

    def navigation_snapshot(self) -> dict:
        with self._lock:
            self.revision += 1
            return {
                "data": self._data,
                "width": self.width,
                "height": self.height,
                "resolution": self.resolution,
                "origin": dict(self.origin),
                "pose": {"x": self._pose[0], "y": self._pose[1], "yaw": self._pose[2]},
                "quality": 0.9,
                "scans": 5,
                "points": 400,
                "updatedAt": self.clock(),
                "revision": self.revision,
                # Карта настоящая: сопоставление включено, препятствия не забываются.
                "trusted": True,
                "navigationIntegrity": True,
                "lidarOffsetM": LIDAR_OFFSET_M,
                "settings": {"min_range": 0.3, "matching": True, "decay": 0},
            }


class FakeLidar:
    """Точки лидара от мировых препятствий, пересчитанные в систему корпуса."""

    def __init__(self, world: FakeMap):
        self.world = world
        self.obstacles: list[tuple[float, float]] = []
        self.stale = False

    def get_scan(self, max_age=None):
        return self.get_scan_info(max_age).get("points")

    def get_scan_info(self, max_age=None):
        """Как отдаёт настоящий менеджер: точки витка и признак доверия."""
        if self.stale:
            return {"points": [], "trusted": False, "reason": "stale"}
        x, y, yaw = self.world.pose()
        # Лидар стоит впереди центра корпуса: точки приходят относительно него.
        sensor_x = x + LIDAR_OFFSET_M * math.cos(yaw)
        sensor_y = y + LIDAR_OFFSET_M * math.sin(yaw)
        # Полный виток: сервер требует плотный скан без больших пропусков.
        points = [{"a": math.radians(deg), "r": 5.0} for deg in range(0, 360)]
        for ox, oy in self.obstacles:
            dx, dy = ox - sensor_x, oy - sensor_y
            distance = math.hypot(dx, dy)
            if distance > 6.0:
                continue
            angle = math.atan2(dy, dx) - yaw
            points.append({"a": math.atan2(math.sin(angle), math.cos(angle)),
                           "r": distance})
        return {"points": points, "trusted": True, "reason": ""}


class FakeSource:
    """Источник шасси: телеметрия четырёх колёс и журнал команд.

    Исполнитель требует не просто «привод готов», а свежие кадры всех четырёх
    плат: номер подтверждённого STOP, нулевую тягу, отсутствие движения и
    растущий счётчик SC. Поэтому фальшивый источник публикует такие кадры так
    же, как настоящий мастер.
    """

    name = "master"
    supports_drive = True
    supports_individual_steer = True
    STALE_S = 1.0
    WHEELS = ("FL", "FR", "RL", "RR")

    def __init__(self, clock=None):
        self.clock = clock or time.time
        self.percent = 0
        self.commands: list[tuple] = []
        self.lock = threading.RLock()
        self.brake_id = 3
        self.counters = {name: 100 for name in self.WHEELS}
        self.frames: dict[str, tuple[dict, float]] = {}
        self._publish(0)

    def _publish(self, percent: int, brake_id: int | None = None) -> None:
        now = self.clock()
        brake = self.brake_id if brake_id is None else brake_id
        for name in self.WHEELS:
            self.frames[name] = ({
                "brakeId": brake,
                "drivePercent": int(percent),
                "cal": True,
                "moving": False,
                "calib": False,
                "cycle": False,
                "scRecent": bool(percent),
                "firmware": 23,
                "scCount": self.counters[name],
                "scRpm": 0.4 if percent else 0.0,
            }, now)

    def drive(self, percent, now=None):
        with self.lock:
            self.percent = int(percent)
            self.commands.append(("drive", int(percent)))
            if percent:
                for name in self.WHEELS:
                    self.counters[name] = (self.counters[name] + 7) & 0xFFFFFFFF
            self._publish(self.percent)
        return {"ok": True}

    def steer_module(self, name, deg, now=None):
        with self.lock:
            self.commands.append(("steer", name, deg))
        return {"ok": True}

    def brake_for_keyboard(self, now=None):
        """Barrier STOP: новая метка торможения и кадры «стоим» со всех плат."""
        with self.lock:
            self.brake_id += 1
            self.percent = 0
            self.commands.append(("brake", self.brake_id))
            self._publish(0, brake_id=self.brake_id)
            return {"ok": True, "brakeId": self.brake_id}

    def drive_status(self):
        with self.lock:
            percent = self.percent
        return {"canDrive": True, "available": True, "active": percent != 0,
                "commandedPercent": percent, "reason": ""}


@pytest.fixture
def rig(monkeypatch):
    """Собрать сервис с фальшивым шасси; возвращает все части для проверок."""

    def build(*, detours=None, clock_step: float = CLOCK_STEP, robots=1,
              speed_mps: float = 0.05, mps_per_percent: float = 0.02):
        if detours is None:
            monkeypatch.delenv("RC_NAV_DETOUR_ATTEMPTS", raising=False)
        else:
            monkeypatch.setenv("RC_NAV_DETOUR_ATTEMPTS", str(detours))
        # Защитный радиус не меньше половины диагонали корпуса 75×60 см (0,48 м).
        monkeypatch.setenv("RC_NAV_FOOTPRINT_M", "0.55")
        clock = FakeClock(clock_step)
        world = FakeMap(clock)
        source = FakeSource(clock)
        lidar = FakeLidar(world)
        robot = {"mode": "advance", "steps": 0, "brakes": [], "bridges": 0, "created": []}

        class Bridge:
            def __init__(self, source_, kin=None, **kwargs):
                self.source = source_
                self.percent = 0
                robot["bridges"] += 1
                robot["created"].append(kwargs)

            def step(self, vx, vy, wz, now=None):
                if not vx and not vy and not wz:
                    self.percent = 0
                    return {"ok": True, "phase": "stopped", "reason": "", "percent": 0}
                robot["steps"] += 1
                clock.tick()
                mode = robot["mode"]
                if mode == "advance":
                    norm = math.hypot(vx, vy) or 1.0
                    world.move(vx / norm * STEP_M, vy / norm * STEP_M)
                elif mode == "drift":
                    world.move(0.0, STEP_M)
                # "stuck": команда принята, но поза не меняется
                self.percent = 2
                self.source.drive(2)
                return {"ok": True, "phase": "driving", "reason": "", "percent": 2}

            def brake(self, reason="manual", now=None):
                self.percent = 0
                self.source.drive(0)
                robot["brakes"].append(reason)
                return {"ok": True, "phase": "stopped", "reason": "", "percent": 0}

        monkeypatch.setattr(nav, "MasterBridge", Bridge)
        service = NavigationService(
            world, source, lidar, enabled=True,
            mps_per_percent=mps_per_percent, speed_mps=speed_mps, max_drive_percent=4,
            clock=clock,
        )
        return service, world, source, lidar, robot

    return build


def wait_idle(service: NavigationService, timeout: float = 20.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not service.active:
            return
        time.sleep(0.01)
    raise AssertionError("фоновый исполнитель навигации не завершился")


def start_to_goal(service, x: float, y: float):
    goal = {"x": x, "y": y}
    service.plan({"goal": goal})
    return service.start({"goal": goal})


# --------------------------------------------------------------------------- #
# Обычное движение по маршруту
# --------------------------------------------------------------------------- #
def test_executor_drives_to_goal_and_brakes(rig):
    service, world, source, _lidar, robot = rig()
    payload, code = start_to_goal(service, 1.0, 0.0)
    assert code == 202
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "succeeded", status
    assert status["progress"] == 1.0
    assert "navigation_goal" in robot["brakes"]
    assert ("drive", 0) in source.commands
    assert world.pose()[0] > 0.85
    assert status["canResume"] is False


def test_executor_never_ignores_a_stale_lidar(rig):
    service, _world, _source, lidar, _robot = rig()
    start_to_goal(service, 1.0, 0.0)
    lidar.stale = True
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "blocked"
    assert "скан" in status["message"].lower() or "лидар" in status["message"].lower()


def test_executor_stops_when_the_chassis_does_not_move(rig):
    service, world, _source, _lidar, robot = rig(clock_step=1.5)
    robot["mode"] = "stuck"
    start_to_goal(service, 1.0, 0.0)
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "blocked"
    assert "не меняется" in status["message"]
    assert world.pose()[0] == 0.0


def test_executor_stops_when_pose_drifts_off_the_route(rig):
    service, _world, _source, _lidar, robot = rig()
    robot["mode"] = "drift"
    start_to_goal(service, 1.0, 0.0)
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "blocked"
    assert "в сторону" in status["message"]


def test_executor_cancel_brakes_immediately(rig):
    service, _world, source, _lidar, robot = rig()
    start_to_goal(service, 1.0, 0.0)
    deadline = time.time() + 5.0
    while robot["steps"] < 2 and time.time() < deadline:
        time.sleep(0.01)
    result = service.cancel("Маршрут отменён оператором.")
    assert result["ok"] is True
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "cancelled"
    assert "navigation_cancel" in robot["brakes"]
    assert source.percent == 0


def test_executor_estop_latch_stops_the_mission(rig):
    from robot_control import estop

    service, _world, _source, _lidar, robot = rig()
    start_to_goal(service, 1.0, 0.0)
    deadline = time.time() + 5.0
    while robot["steps"] < 2 and time.time() < deadline:
        time.sleep(0.01)
    estop.latch("тест")
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "blocked"
    assert "E-STOP" in status["message"]


# --------------------------------------------------------------------------- #
# Препятствие: остановка, обход, предел попыток
# --------------------------------------------------------------------------- #
def test_executor_stops_in_front_of_an_unknown_obstacle_when_detours_are_off(rig):
    service, _world, _source, lidar, robot = rig(detours=0)
    # Препятствие прямо по курсу, но карта о нём ещё не знает.
    lidar.obstacles = [(0.9, 0.0)]
    start_to_goal(service, 1.6, 0.0)
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "blocked"
    assert "препятствие" in status["message"].lower()
    assert "navigation_blocked" in robot["brakes"]
    # Робот остановился, не доехав до препятствия.
    assert 0.0 <= _world.pose()[0] < 0.9


def test_executor_detours_around_an_obstacle_and_reaches_the_goal(rig):
    service, world, _source, lidar, robot = rig()
    # Объект сбоку от прямой линии: его можно объехать в пределах ±28°.
    # Запас 0,55 м (квадрат габарита) требует держаться от него дальше.
    lidar.obstacles = [(0.9, 0.45)]
    phases = []
    original = service._set_state

    def record(**fields):
        phases.append(dict(fields))
        original(**fields)

    service._set_state = record  # type: ignore[method-assign]
    start_to_goal(service, 2.0, 0.0)
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "succeeded", status
    assert any("обход" in str(item.get("message", "")).lower() for item in phases)
    detours = [item.get("detour") for item in phases if item.get("detour")]
    assert detours and detours[-1] >= 1
    assert "navigation_detour" in robot["brakes"]
    assert world.pose()[0] > 1.8


def test_executor_reports_that_a_detour_is_impossible(rig):
    service, world, _source, lidar, robot = rig(detours=2)
    # Узкий коридор: объезд закрыт стенами, поэтому после первой же попытки
    # исполнитель останавливается с понятной причиной, а не кружит по кругу.
    world.block(-4.1, 4.1, 0.9, 1.0)
    world.block(-4.1, 4.1, -1.0, -0.9)
    lidar.obstacles = [(0.9, 0.0)]
    start_to_goal(service, 1.6, 0.0)
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "blocked"
    assert "препятств" in status["message"].lower()
    assert "navigation_blocked" in robot["brakes"]
    assert robot["steps"] > 0


# --------------------------------------------------------------------------- #
# Продолжение остановленного маршрута
# --------------------------------------------------------------------------- #
def test_resume_restarts_the_saved_route_after_the_obstacle_is_gone(rig):
    service, world, _source, lidar, robot = rig(detours=0)
    lidar.obstacles = [(0.9, 0.0)]
    start_to_goal(service, 1.6, 0.0)
    wait_idle(service)
    assert service.status()["phase"] == "blocked"
    assert service.status()["canResume"] is True

    # Препятствие убрали: тот же маршрут снова свободен. Первое продолжение
    # обязано показать изменившийся от новой позы путь, второе — едет.
    lidar.obstacles = []
    with pytest.raises(NavigationError) as caught:
        service.resume()
    assert caught.value.code == "plan_changed"
    assert service.status()["canResume"] is True
    payload, code = service.resume()
    assert code == 202, payload
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "succeeded", status
    assert world.pose()[0] > 1.4
    assert robot["bridges"] >= 2


def test_resume_requires_a_new_preview_when_the_path_changed(rig):
    service, _world, _source, lidar, _robot = rig(detours=0)
    lidar.obstacles = [(0.9, 0.0)]
    start_to_goal(service, 1.6, 0.0)
    wait_idle(service)
    assert service.status()["phase"] == "blocked"

    with pytest.raises(NavigationError) as caught:
        service.resume()
    assert caught.value.code == "plan_changed"
    assert caught.value.details.get("plan")
    assert service.status()["phase"] == "planned"

    # Препятствие на месте: подтверждённый маршрут снова останавливается на нём,
    # а не проезжает вслепую.
    payload, code = service.resume()
    assert code == 202, payload
    wait_idle(service)
    assert service.status()["phase"] == "blocked"


def test_resume_rejects_an_idle_service(rig):
    service, _world, _source, _lidar, _robot = rig()
    with pytest.raises(NavigationError) as caught:
        service.resume()
    assert caught.value.code in {"plan_required", "not_resumable"}


def test_resume_endpoint_requires_the_control_header(kiosk_client):
    reply = kiosk_client.post("/api/navigation/resume", json={})
    assert reply.status_code == 400
    assert reply.json["reason"] == "request_header_required"
    reply = kiosk_client.post("/api/navigation/resume", json={},
                              headers={"X-Robot-Control": "navigation"})
    assert reply.status_code == 409
    assert reply.json["code"] in {"plan_required", "not_resumable"}


def test_web_resume_route_is_proxied_behind_login(client):
    assert client.post("/api/navigation/resume", json={}).status_code in (302, 401)


# --------------------------------------------------------------------------- #
# Планировщик: клетки препятствий, которых нет на карте
# --------------------------------------------------------------------------- #
def _blank_grid(cells=80, resolution=0.1):
    return {"width": cells, "height": cells, "resolution": resolution,
            "origin": {"x": -4.0, "y": -4.0}, "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "data": bytes(cells * cells), "revision": 3}


def _cell_of(snapshot, x, y):
    return (int(math.floor((x - snapshot["origin"]["x"]) / snapshot["resolution"])),
            int(math.floor((y - snapshot["origin"]["y"]) / snapshot["resolution"])))


def test_planner_goes_around_extra_blocked_cells():
    snapshot = _blank_grid()
    # Клетка прямо на прямой линии к цели: без overlay путь идёт через неё.
    straight = plan_grid_path(snapshot, 1.0, 0.0, footprint_m=0.42)
    assert len(straight["path"]) >= 2
    reserved = {_cell_of(snapshot, x, 0.0) for x in (0.4, 0.5, 0.6, 0.7)}
    detour = plan_grid_path(snapshot, 1.0, 0.0, footprint_m=0.42,
                            extra_blocked=reserved)
    blocked = set(reserved)
    for a, b in zip(detour["path"], detour["path"][1:]):
        steps = max(1, int(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / 0.02))
        for n in range(steps + 1):
            t = n / steps
            cell = _cell_of(snapshot, a["x"] + (b["x"] - a["x"]) * t,
                            a["y"] + (b["y"] - a["y"]) * t)
            assert cell not in blocked, detour["path"]
    assert detour["distanceM"] > straight["distanceM"]


def test_planner_refuses_a_goal_inside_an_unknown_obstacle():
    snapshot = _blank_grid()
    goal = _cell_of(snapshot, 1.0, 0.0)
    with pytest.raises(NavigationError) as caught:
        plan_grid_path(snapshot, 1.0, 0.0, footprint_m=0.42, extra_blocked={goal})
    assert caught.value.code == "goal_obstacle"


def test_route_forwards_extra_blocked_cells():
    snapshot = _blank_grid()
    reserved = {_cell_of(snapshot, 1.0, 0.0)}
    with pytest.raises(NavigationError) as caught:
        plan_route(snapshot, [{"x": 1.0, "y": 0.0}], footprint_m=0.42,
                   extra_blocked=reserved)
    assert caught.value.code == "goal_obstacle"


# --------------------------------------------------------------------------- #
# Натурный замер оператора: колесо 215,9 мм, Servo 30, ≈0,625 м/с, тормоз ≈15 см
# --------------------------------------------------------------------------- #
#: Замер: Servo.write(30) → 30 / 1,8 ≈ 16,7 % тяги → 0,625 м/с.
MEASURED_SERVO = 30
MEASURED_SPEED_MPS = 0.625
MEASURED_MPS_PER_PERCENT = MEASURED_SPEED_MPS / (MEASURED_SERVO / 1.8)


def test_measured_calibration_drives_to_the_goal(rig):
    """Калибровка из замера проходит проверку и робот доезжает до цели."""
    service, world, source, _lidar, _robot = rig(
        mps_per_percent=MEASURED_MPS_PER_PERCENT, speed_mps=0.05)
    checks = service.status()["diagnostics"]["checks"]
    assert checks["speedCalibrated"] is True
    # 0,05 м/с при 0,0375 м/с на 1% — это 1% тяги, предел 4% не нарушен.
    assert checks["speedWithinLimit"] is True

    payload, code = start_to_goal(service, 1.0, 0.0)
    assert code == 202
    wait_idle(service)

    status = service.status()
    assert status["phase"] == "succeeded", status
    assert world.pose()[0] > 0.85
    assert ("drive", 0) in source.commands


def test_measured_calibration_rejects_a_too_fast_setpoint(rig):
    """При этой же калибровке 0,10 м/с требует 3% — всё ещё в пределах лимита."""
    service, _world, _source, _lidar, _robot = rig(
        mps_per_percent=MEASURED_MPS_PER_PERCENT, speed_mps=0.10)
    checks = service.status()["diagnostics"]["checks"]
    assert checks["speedWithinLimit"] is True
    # А вот 0,5 м/с потребовали бы 13% — за пределом 4%, старт запрещён.
    fast = NavigationService(service.map_service, service.source, enabled=True,
                             mps_per_percent=MEASURED_MPS_PER_PERCENT,
                             speed_mps=0.10, max_drive_percent=1, clock=service.clock)
    assert fast.status()["diagnostics"]["checks"]["speedWithinLimit"] is False


def test_calibration_is_not_assumed_when_value_is_missing(rig):
    service, _world, _source, _lidar, _robot = rig(mps_per_percent=None)
    checks = service.status()["diagnostics"]["checks"]
    assert checks["speedCalibrated"] is False
    with pytest.raises(NavigationError) as caught:
        start_to_goal(service, 1.0, 0.0)
    assert caught.value.code == "not_ready"
