"""Thin wrapper around the Genisom L1 SDK connection, move, gait and state calls."""

import importlib
import os
import platform
import sys

CTRL_MODE_NAMES = {
    0: "DAMPING",
    1: "STANDING",
    10: "LIE_DOWN_FREE",
    18: "MOVING",
    21: "ACTION",
    51: "LIE_DOWN",
}


def find_sdk_dir(configured_dir=""):
    """Resolve the SDK root dir from the param, env var, or a common root path."""
    for candidate in (configured_dir, os.environ.get("GENISOM_SDK_DIR", "")):
        if candidate and os.path.isdir(candidate):
            return candidate
    for guess in ("/root/genisom_l1_sdk_old", "/genisom_l1_sdk_old", "/home/nvidia/genisom_l1_sdk_old"):
        if os.path.isdir(guess):
            return guess
    raise FileNotFoundError("genisom SDK dir not found: set the 'sdk_dir' param or GENISOM_SDK_DIR")


def load_sdk_module(sdk_dir, module_name):
    """Add the SDK's arch-specific lib dir to sys.path and import the compiled module."""
    arch = platform.machine().replace("amd64", "x86_64").replace("arm64", "aarch64")
    lib_path = os.path.join(sdk_dir, "lib", arch)
    if not os.path.isdir(lib_path):
        raise FileNotFoundError(f"SDK lib dir not found: {lib_path}")
    sys.path.insert(0, lib_path)
    os.environ["LD_LIBRARY_PATH"] = lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    return importlib.import_module(module_name)


class DogSocket:
    """Wraps the SDK HighLevel handle: connect, move, gait switch, and state reads."""

    def __init__(self, sdk_dir, module_name, local_ip, local_port, dog_ip):
        """Load the SDK module and construct the HighLevel handle."""
        self._sdk_dir = find_sdk_dir(sdk_dir)
        self._module = load_sdk_module(self._sdk_dir, module_name)
        self._dog = self._module.HighLevel()
        self._local_ip = local_ip
        self._local_port = local_port
        self._dog_ip = dog_ip

    def connect(self):
        """Init the UDP link to the robot."""
        self._dog.initRobot(self._local_ip, self._local_port, self._dog_ip)

    def is_connected(self):
        """Return True if the SDK reports a live connection."""
        try:
            return bool(self._dog.checkConnect())
        except Exception:
            return False

    def stand_up(self):
        """Ask the robot to stand up."""
        return self._dog.standUp()

    def lie_down(self):
        """Ask the robot to lie down."""
        return self._dog.lieDown()

    def move(self, vx, vy, yaw_rate):
        """Forward a planar velocity command to the robot."""
        return self._dog.move(vx, vy, yaw_rate)

    def stop(self):
        """Send a zero-velocity command."""
        return self._dog.move(0.0, 0.0, 0.0)

    def set_gait(self, gait_id):
        """Switch the robot's walking gait (confirm the real method name in the SDK header)."""
        return self._dog.switchGait(gait_id)

    def get_battery(self):
        """Return battery percentage (0-100) or None on read error."""
        try:
            return float(self._dog.getBatteryPower())
        except Exception:
            return None

    def get_ctrl_mode(self):
        """Return the SDK's current control mode id, or None on read error."""
        try:
            return int(self._dog.getCurrentCtrlmode())
        except Exception:
            return None
