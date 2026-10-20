"""Перезапуск вставшего драйвера в ROS-супервизоре (``zapret_lidar.node``).

Супервизор на роботе поднимает рабочий драйвер, а не начинает перебор плана
с начала: иначе каждый обрыв превращался в секунды тишины и попытки чужих
драйверов на своём же порту — на экране это выглядело как «с перерывами, то
200 точек, то 15». Узел не импортируется без rclpy, поэтому здесь — заглушки
ROS 2 и прямые проверки решения ``_restart_same``.

Стабы живут только внутри теста: после него ``sys.modules`` возвращается в
исходное состояние, чтобы другие файлы видели «без ROS 2» как есть.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

ROS2_PKG_ROOT = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "zapret_lidar"


def _make_ros_stubs() -> dict[str, types.ModuleType]:
    """Минимальные заглушки rclpy/sensor_msgs/std_msgs/std_srvs для импорта узла."""

    class Node:
        def __init__(self, name: str) -> None:
            self._node_name = name

        def get_logger(self):
            class _Log:
                def info(self, *args, **kwargs) -> None: ...
                def warn(self, *args, **kwargs) -> None: ...
                def warning(self, *args, **kwargs) -> None: ...
                def error(self, *args, **kwargs) -> None: ...

            return _Log()

    class QoSProfile:
        def __init__(self, *args, **kwargs) -> None: ...

    class _Policy:
        BEST_EFFORT = 1
        RELIABLE = 2
        KEEP_LAST = 1
        VOLATILE = 1

    def _module(name: str, **attrs) -> types.ModuleType:
        mod = types.ModuleType(name)
        for key, value in attrs.items():
            setattr(mod, key, value)
        return mod

    return {
        "rclpy": _module("rclpy", ok=lambda: True, init=lambda args=None: None),
        "rclpy.node": _module("rclpy.node", Node=Node),
        "rclpy.qos": _module(
            "rclpy.qos",
            QoSProfile=QoSProfile,
            QoSDurabilityPolicy=_Policy,
            QoSHistoryPolicy=_Policy,
            QoSReliabilityPolicy=_Policy,
            qos_profile_sensor_data=QoSProfile(),
        ),
        "sensor_msgs": _module("sensor_msgs"),
        "sensor_msgs.msg": _module("sensor_msgs.msg", LaserScan=type("LaserScan", (), {})),
        "std_msgs": _module("std_msgs"),
        "std_msgs.msg": _module("std_msgs.msg", String=type("String", (), {})),
        "std_srvs": _module("std_srvs"),
        "std_srvs.srv": _module("std_srvs.srv", Trigger=type("Trigger", (), {})),
    }


@pytest.fixture()
def node_module():
    stubs = _make_ros_stubs()
    saved = {name: sys.modules.get(name) for name in stubs}
    saved_zapret = {name: mod for name, mod in sys.modules.items()
                    if name == "zapret_lidar" or name.startswith("zapret_lidar.")}
    for name, mod in stubs.items():
        sys.modules[name] = mod
    sys.path.insert(0, str(ROS2_PKG_ROOT))
    try:
        for name in list(sys.modules):
            if name == "zapret_lidar" or name.startswith("zapret_lidar."):
                del sys.modules[name]
        import zapret_lidar.node as node
        yield node
    finally:
        sys.path.remove(str(ROS2_PKG_ROOT))
        for name in list(sys.modules):
            if name == "zapret_lidar" or name.startswith("zapret_lidar."):
                del sys.modules[name]
        sys.modules.update(saved_zapret)
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod


def _bare_supervisor(node_module, **attrs):
    """Экземпляр без __init__ (ROS-конструктор не нужен): только рабочие поля."""
    sup = object.__new__(node_module.LidarSupervisor)
    sup.state = "restarting"
    sup.reason = ""
    sup._working_item = None
    sup._current_item = None
    sup._same_restarts = 0
    sup.launch = lambda item: True
    for key, value in attrs.items():
        setattr(sup, key, value)
    return sup


def test_restart_same_limit_is_two(node_module):
    assert node_module.RESTART_SAME_LIMIT == 2


def test_supervisor_restarts_working_driver_same_package(node_module):
    """Вставший рабочий драйвер поднимается тем же пакетом и портом, дважды."""
    item = types.SimpleNamespace(
        driver=types.SimpleNamespace(title="LD06/LD19 (230400)"),
        port="/dev/ttyUSB0",
        baud=230400,
    )
    launched = []
    sup = _bare_supervisor(node_module, _working_item=item,
                           launch=lambda it: launched.append(it) or True)

    assert sup._restart_same("нет точек") is True
    assert sup._restart_same("нет точек") is True
    assert launched == [item, item]
    assert sup._same_restarts == 2
    assert "перезапускаю тот же" in sup.reason

    # Лимит исчерпан — дальше решает старый цикл плана (probe + перебор).
    assert sup._restart_same("нет точек") is False
    assert len(launched) == 2


def test_supervisor_restart_same_needs_running_driver(node_module):
    """Липкий перезапуск только для драйвера, который уже отдавал /scan."""
    item = types.SimpleNamespace(driver=types.SimpleNamespace(title="X"),
                                 port="/dev/ttyUSB0", baud=230400)
    sup = _bare_supervisor(node_module, _working_item=item)
    sup.state = "probing"
    assert sup._restart_same("нет точек") is False

    sup = _bare_supervisor(node_module, _working_item=None)
    assert sup._restart_same("нет точек") is False


def test_supervisor_restart_same_falls_back_when_launch_fails(node_module):
    """Если тот же драйвер не запустился — возвращаемся к полному перебору."""
    item = types.SimpleNamespace(driver=types.SimpleNamespace(title="X"),
                                 port="/dev/ttyUSB0", baud=230400)
    sup = _bare_supervisor(node_module, _working_item=item, launch=lambda it: False)
    assert sup._restart_same("порт занят") is False
