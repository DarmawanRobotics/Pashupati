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


def check_sdk_dir(sdk_dir: str) -> str:
    """Check that the SDK root dir exists and contains the expected lib subdir."""
    if not sdk_dir or not os.path.isdir(sdk_dir):
        raise FileNotFoundError(f"SDK dir not found: {sdk_dir}")

    arch = (
        platform.machine()
        .replace("amd64", "x86_64")
        .replace("arm64", "aarch64")
    )
    lib_path = os.path.join(sdk_dir, "lib", arch)
    if not os.path.isdir(lib_path):
        raise FileNotFoundError(f"SDK lib dir not found: {lib_path}")
    return lib_path


def load_sdk_module(lib_dir: str, module_name: str):
    """Add the SDK arch-specific lib dir to sys.path and import the compiled module."""
    if lib_dir not in sys.path:
        sys.path.append(lib_dir)
    try:
        return importlib.import_module(module_name)
    except ImportError as e:
        raise ImportError(
            f"Failed to import SDK module '{module_name}' from '{lib_dir}': {e}"
        ) 
class DogSocket:
    """Wraps the SDK HighLevel handle: connect, move, gait switch, and state reads."""
    def __init__(
        self,
        sdk_dir: str,
        module_name: str,
        local_ip: str,
        local_port: int,
        dog_ip: str,
    ):
        """Load the SDK module and construct the HighLevel handle."""
        self._lib_dir = check_sdk_dir(sdk_dir)
        self._module = load_sdk_module(
            self._lib_dir,
            module_name,
        )
        self._dog = self._module.HighLevel()

        self._local_ip = local_ip
        self._local_port = local_port
        self._dog_ip = dog_ip

    def connect(self):
        """Init the UDP link to the robot."""
        self._dog.initRobot(
            self._local_ip,
            self._local_port,
            self._dog_ip,
        )

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
        """Switch the robot's walking gait."""
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