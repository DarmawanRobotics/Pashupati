#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from robot_drivers.dog_socket import CTRL_MODE_NAMES, DogSocket

MOVABLE_MODES = {1, 18, 21}  # STANDING, MOVING, ACTION


class RobotDriverNode(Node):
    """Owns the single SDK connection: cmd_vel forwarding, mode/estop services, and state polling."""
    def __init__(self):
        """Declare params, create the SDK handle (unbound), and set up subscriptions/services/timers."""
        super().__init__("robot_driver_node")
        self.declare_parameter("sdk_dir", "/opt/genisom_l1_sdk")
        self.declare_parameter("sdk_module_name", "mc_sdk_zsl_1w_py")
        self.declare_parameter("dog_ip", "192.168.234.1")
        self.declare_parameter("local_ip", "192.168.234.234")
        self.declare_parameter("local_port", 43988)

        self.declare_parameter("control_frequency", 10.0)
        self.declare_parameter("watchdog_timeout", 1.0)
        self.declare_parameter("max_linear_vel", 0.4)
        self.declare_parameter("max_angular_vel", 0.6)
        self.declare_parameter("poll_rate", 1.0)

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
        self._is_auto = False
        self._latest_cmd = None
        self._last_cmd_time = 0.0
        self._battery_percentage = 0.0

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_service(SetBool, "drivers/set_auto_mode", self.mode_callback)
        self.create_service(SetBool, "drivers/stand_up", self.stand_up_callback)
        self.create_service(Trigger, "drivers/emergency_stop", self.emergency_stop_callback)

        self._battery_pub = self.create_publisher(BatteryState, "drivers/battery", 10)
        self._state_pub = self.create_publisher(String, "drivers/robot_state", 10)

        frequency = self.get_parameter("control_frequency").value
        self.create_timer(1.0 / frequency, self.update)

        poll_rate = self.get_parameter("poll_rate").value
        self.create_timer(1.0 / poll_rate, self.poll_state)

    def _ensure_standing(self):
        """Stand up only if the robot isn't already in a movable state. Must be connected."""
        mode = self._dog.get_ctrl_mode()
        if mode in MOVABLE_MODES:
            self.get_logger().info(f"already movable (mode={mode}), skip stand_up")
            return True, "already standing"
        self.get_logger().info(f"mode={mode}, standing up")
        success, message = self._dog.stand_up()
        if not success:
            self.get_logger().error(f"stand_up failed: {message}")
        return success, message

    def cmd_vel_callback(self, msg):
        """Cache the latest velocity command and its arrival time."""
        self._latest_cmd = msg
        self._last_cmd_time = time.time()

    def mode_callback(self, request, response):
        """Bind on auto-enable, unbind on auto-disable so manual/remote control regains authority."""
        if request.data:
            if not self._dog.is_connected():
                try:
                    self._dog.connect()
                except Exception as error:
                    response.success = False
                    response.message = f"connect failed: {error}"
                    self.get_logger().error(response.message)
                    return response

            battery = self._dog.get_battery()
            if battery is not None:
                self._battery_percentage = battery / 100.0

            if self._battery_percentage < 20.0:
                response.success = False
                response.message = "battery low"
                self.get_logger().error("cannot set auto mode: battery low")
                self._dog.disconnect()
                return response

            stand_ok, stand_msg = self._ensure_standing()
            if not stand_ok:
                response.success = False
                response.message = f"stand_up failed: {stand_msg}"
                self._dog.disconnect()
                return response

            self._is_auto = True
            response.success = True
            response.message = "auto mode enabled"
            self.get_logger().info(response.message)
            return response

        self._is_auto = False
        self._latest_cmd = None
        if self._dog.is_connected():
            self._dog.stop()
            self._dog.disconnect()
        response.success = True
        response.message = "auto mode disabled, unbound for manual control"
        self.get_logger().info(response.message)
        return response

    def stand_up_callback(self, request, response):
        """Stand up the robot if true or lie down if false. Requires an active bind (auto mode)."""
        if not self._dog.is_connected():
            response.success = False
            response.message = "not bound (enable auto mode first)"
            return response
        if request.data:
            success, message = self._ensure_standing()
        else:
            self.get_logger().info("lie_down service called")
            success, message = self._dog.lie_down()
        response.success = success
        response.message = message
        return response

    def emergency_stop_callback(self, request, response):
        """Force manual mode, stop + passive, then unbind if currently bound."""
        self._is_auto = False
        self._latest_cmd = None
        try:
            if self._dog.is_connected():
                self._dog.stop()
                success, message = self._dog.passive()
                self._dog.disconnect()
                if not success:
                    self.get_logger().warning(f"passive() returned: {message}")
            response.success = True
            response.message = "robot stopped, set passive, and unbound"
            self.get_logger().warning("emergency stop triggered")
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"emergency stop failed: {error}")
        return response

    def update(self):
        """Forward cmd_vel to the robot while auto mode is enabled."""
        if not self._is_auto:
            return
        stale = self._latest_cmd is None or (time.time() - self._last_cmd_time > self._watchdog_timeout)
        if stale:
            self._dog.stop()
            self.get_logger().debug("stale cmd_vel: stopping")
            return
        vx = max(-self._max_linear_vel, min(self._max_linear_vel, self._latest_cmd.linear.x))
        vy = max(-self._max_linear_vel, min(self._max_linear_vel, self._latest_cmd.linear.y))
        yaw = max(-self._max_angular_vel, min(self._max_angular_vel, self._latest_cmd.angular.z))
        success, message = self._dog.move(vx, vy, yaw)
        if not success:
            self.get_logger().warning(f"move() returned: {message}")

    def poll_state(self):
        """Publish battery + mode only while bound; otherwise report unbound status."""
        if not self._dog.is_connected():
            state_msg = String()
            state_msg.data = "connected=False mode=MANUAL(unbound)"
            self._state_pub.publish(state_msg)
            return

        battery = self._dog.get_battery()
        mode = self._dog.get_ctrl_mode()

        if battery is not None:
            self._battery_percentage = battery / 100.0
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.percentage = self._battery_percentage
            battery_msg.present = True
            self._battery_pub.publish(battery_msg)

        mode_name = CTRL_MODE_NAMES.get(mode, "UNKNOWN")
        state_msg = String()
        state_msg.data = f"connected=True mode={mode_name}({mode}) battery={battery}"
        self._state_pub.publish(state_msg)

    def destroy_node(self):
        """Stop and unbind before shutting down, if currently bound."""
        if self._dog.is_connected():
            self._dog.stop()
            self._dog.disconnect()
        super().destroy_node()


def main(args=None):
    """Spin the driver node."""
    rclpy.init(args=args)
    node = RobotDriverNode()
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