"""ROS2 node: forwards cmd_vel to the dog SDK and exposes mode/gait/estop services."""

import time
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from robot_interface.enum_utils import generateEnumDict
from robot_interface.msg import GaitMode
from robot_interface.srv import SetGait
from std_srvs.srv import SetBool, Trigger

from robot_drivers.dog_socket import DogSocket

GaitModeEnum = Enum('GaitModeEnum', generateEnumDict(GaitMode))


class SenderNode(Node):
    """Owns the SDK write side: cmd_vel forwarding, gait switch, mode and estop services."""

    def __init__(self):
        """Declare params, connect the socket, and set up the sub/services/timer."""
        super().__init__("robot_driver_sender")

        self.declare_parameter("sdk_dir", "")
        self.declare_parameter("sdk_module_name", "mc_sdk_l1_py")
        self.declare_parameter("dog_ip", "192.168.234.1")
        self.declare_parameter("local_ip", "192.168.234.13")
        self.declare_parameter("local_port", 43988)
        self.declare_parameter("control_frequency", 10.0)
        self.declare_parameter("watchdog_timeout", 1.0)
        self.declare_parameter("max_linear_vel", 0.4)
        self.declare_parameter("max_angular_vel", 0.6)
        self.declare_parameter("default_gait", GaitMode.STAND)

        self._watchdog_timeout = self.get_parameter("watchdog_timeout").value
        self._max_linear_vel = self.get_parameter("max_linear_vel").value
        self._max_angular_vel = self.get_parameter("max_angular_vel").value

        self._dog = DogSocket(
            self.get_parameter("sdk_dir").value,
            self.get_parameter("sdk_module_name").value,
            self.get_parameter("local_ip").value,
            self.get_parameter("local_port").value,
            self.get_parameter("dog_ip").value,
        )
        self._dog.connect()

        self._auto_mode = False
        self._latest_cmd = None
        self._last_cmd_time = 0.0

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_service(SetBool, "~/set_auto_mode", self._on_set_auto_mode)
        self.create_service(SetGait, "~/set_gait", self._on_set_gait)
        self.create_service(Trigger, "~/emergency_stop", self._on_emergency_stop)

        freq = self.get_parameter("control_frequency").value
        self.create_timer(1.0 / freq, self._control_loop)

        try:
            self._dog.set_gait(self.get_parameter("default_gait").value)
        except Exception as error:
            self.get_logger().warn(f"default gait switch failed: {error}")

        self.get_logger().info("robot_driver sender ready (manual mode)")

    def _on_cmd_vel(self, msg):
        """Cache the latest velocity command and its arrival time."""
        self._latest_cmd = msg
        self._last_cmd_time = time.time()

    def _on_set_auto_mode(self, request, response):
        """Toggle between auto (drive from cmd_vel) and manual (ignore cmd_vel)."""
        self._auto_mode = request.data
        if not self._auto_mode:
            self._dog.stop()
        response.success = True
        response.message = "auto" if self._auto_mode else "manual"
        self.get_logger().info(f"mode set to {response.message}")
        return response

    def _on_set_gait(self, request, response):
        """Switch gait via the SDK, resolving the id through GaitModeEnum for logging."""
        try:
            self._dog.set_gait(request.gait_id)
            gait_name = GaitModeEnum(request.gait_id).name
            response.success = True
            response.message = f"gait set to {gait_name}({request.gait_id})"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def _on_emergency_stop(self, request, response):
        """Force manual mode and send an immediate stop."""
        self._auto_mode = False
        self._latest_cmd = None
        try:
            self._dog.stop()
            response.success = True
            response.message = "stopped"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def _control_loop(self):
        """Forward the latest cmd_vel to the dog while in auto mode, else do nothing."""
        if not self._auto_mode:
            return
        stale = self._latest_cmd is None or (time.time() - self._last_cmd_time) > self._watchdog_timeout
        if stale:
            self._dog.stop()
            return
        vx = max(-self._max_linear_vel, min(self._max_linear_vel, self._latest_cmd.linear.x))
        vy = max(-self._max_linear_vel, min(self._max_linear_vel, self._latest_cmd.linear.y))
        yaw = max(-self._max_angular_vel, min(self._max_angular_vel, self._latest_cmd.angular.z))
        self._dog.move(vx, vy, yaw)


def main(args=None):
    """Spin the sender node."""
    rclpy.init(args=args)
    node = SenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
