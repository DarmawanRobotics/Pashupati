"""Unit tests for DogSocket against a fake SDK module (no hardware needed)."""

import sys
import types

import pytest

from robot_drivers import dog_socket


class FakeHighLevel:
    """Stand-in for the compiled SDK's HighLevel class."""

    def __init__(self):
        """Seed fake state for assertions."""
        self.moved = None
        self.gait = None
        self.battery = 87.0
        self.mode = 1

    def initRobot(self, local_ip, local_port, dog_ip):
        """Record the connection args."""
        self.init_args = (local_ip, local_port, dog_ip)

    def checkConnect(self):
        """Report connected."""
        return True

    def move(self, vx, vy, yaw_rate):
        """Record the last requested velocity."""
        self.moved = (vx, vy, yaw_rate)
        return True

    def getBatteryPower(self):
        """Return the fake battery percentage."""
        return self.battery

    def getCurrentCtrlmode(self):
        """Return the fake control mode id."""
        return self.mode

    def switchGait(self, gait_id):
        """Record the requested gait id."""
        self.gait = gait_id
        return True


@pytest.fixture
def socket(monkeypatch, tmp_path):
    """Build a DogSocket wired to the fake SDK module."""
    sdk_dir = tmp_path / "sdk"
    (sdk_dir / "lib" / "x86_64").mkdir(parents=True)
    fake_module = types.ModuleType("mc_sdk_l1_py")
    fake_module.HighLevel = FakeHighLevel
    monkeypatch.setitem(sys.modules, "mc_sdk_l1_py", fake_module)
    monkeypatch.setattr(dog_socket.platform, "machine", lambda: "x86_64")
    return dog_socket.DogSocket(str(sdk_dir), "mc_sdk_l1_py", "127.0.0.1", 1234, "127.0.0.2")


def test_move_forwards_values(socket):
    """move() should pass its args straight to the SDK."""
    socket.move(0.1, 0.0, 0.2)
    assert socket._dog.moved == (0.1, 0.0, 0.2)


def test_set_gait_forwards_id(socket):
    """set_gait() should pass the gait id straight to the SDK."""
    socket.set_gait(2)
    assert socket._dog.gait == 2


def test_get_battery_returns_percentage(socket):
    """get_battery() should return the SDK's raw percentage."""
    assert socket.get_battery() == 87.0


def test_get_ctrl_mode_returns_id(socket):
    """get_ctrl_mode() should return the SDK's mode id."""
    assert socket.get_ctrl_mode() == 1


def test_get_battery_returns_none_on_error(socket, monkeypatch):
    """get_battery() should swallow SDK errors and return None."""
    def boom():
        raise RuntimeError("no link")

    monkeypatch.setattr(socket._dog, "getBatteryPower", boom)
    assert socket.get_battery() is None


def test_is_connected_returns_false_on_error(socket, monkeypatch):
    """is_connected() should swallow SDK errors and return False."""
    def boom():
        raise RuntimeError("no link")

    monkeypatch.setattr(socket._dog, "checkConnect", boom)
    assert socket.is_connected() is False
