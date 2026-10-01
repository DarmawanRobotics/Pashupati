#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from robot_bridge.mux import ControlMux, MODES
from robot_interfaces.srv import SetMode
from std_msgs.msg import String
from std_srvs.srv import SetBool


class ControlMuxNode(Node):
    """Forwards navigation or teleop velocity to the robot depending on the control mode."""

    def __init__(self):
        """Declare params, wire topics and the mode service."""
        super().__init__('control_mux_node')
        self.declare_parameter('mode', 'auto')
        self.declare_parameter('remote_timeout_sec', 0.3)
        self.declare_parameter('pause_nav_on_remote', True)
        self._mux = ControlMux(
            self.get_parameter('mode').value,
            float(self.get_parameter('remote_timeout_sec').value),
        )
        self._pause_nav = bool(self.get_parameter('pause_nav_on_remote').value)
        self._paused_by_us = False

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._mode_pub = self.create_publisher(String, 'bridge/mode', latched)
        self.create_subscription(Twist, 'bridge/nav_cmd_vel', self.nav_callback, 10)
        self.create_subscription(Twist, 'bridge/remote_cmd_vel', self.remote_callback, 10)
        self.create_service(SetMode, 'bridge/set_mode', self.set_mode_callback)
        self._pause_client = self.create_client(SetBool, 'navigation/pause')
        self.create_timer(0.05, self.watchdog)
        self.publish_mode()

    def now(self) -> float:
        """Return seconds from the node clock."""
        return self.get_clock().now().nanoseconds / 1e9

    def publish_mode(self):
        """Publish the current mode (latched)."""
        self._mode_pub.publish(String(data=self._mux.mode))

    def nav_callback(self, msg: Twist):
        """Forward navigation commands in auto mode."""
        out = self._mux.on_nav(msg)
        if out is not None:
            self._cmd_pub.publish(out)

    def remote_callback(self, msg: Twist):
        """Forward teleop commands in remote mode."""
        out = self._mux.on_remote(msg, self.now())
        if out is not None:
            self._cmd_pub.publish(out)

    def watchdog(self):
        """Send one zero command when teleop goes silent or the mode just changed."""
        if self._mux.watchdog(self.now()):
            self._cmd_pub.publish(Twist())

    def set_mode_callback(self, request, response):
        """Switch mode, pausing navigation while the remote drives."""
        if request.mode not in MODES:
            response.success = False
            response.message = f'unknown mode {request.mode!r}, use {list(MODES)}'
            return response
        changed = self._mux.set_mode(request.mode)
        self._cmd_pub.publish(Twist())
        if changed and self._pause_nav:
            if request.mode == 'remote':
                self._paused_by_us = self.call_pause(True)
            elif self._paused_by_us:
                self.call_pause(False)
                self._paused_by_us = False
        self.publish_mode()
        self.get_logger().info(f'control mode: {request.mode}')
        response.success = True
        response.message = f'mode {request.mode}'
        return response

    def call_pause(self, pause: bool) -> bool:
        """Ask navigation to pause or resume without waiting; False when it is not running."""
        if not self._pause_client.service_is_ready():
            return False
        self._pause_client.call_async(SetBool.Request(data=pause))
        return True


def main(args=None):
    """Spin the control mux node."""
    rclpy.init(args=args)
    node = ControlMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
