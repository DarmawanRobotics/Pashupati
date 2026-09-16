#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

from robot_interface.srv import SetGait
from robot_drivers.dog_socket import DogSocket

class SenderNode(Node):
    """Owns the SDK write side: cmd_vel forwarding, gait, mode and emergency stop."""
    def __init__(self):
        """Declare params, connect the SDK, and set up subscriptions/services."""
        super().__init__("sender_sender_node")
        self.declare_parameter("sdk_dir", "")
        self.declare_parameter("sdk_module_name", "mc_sdk_l1_py")
        self.declare_parameter("dog_ip", "192.168.234.1")
        self.declare_parameter("local_ip", "192.168.234.13")
        self.declare_parameter("local_port", 43988)

        self.declare_parameter("control_frequency", 10.0)
        self.declare_parameter("watchdog_timeout", 1.0)
        self.declare_parameter("max_linear_vel", 0.4)
        self.declare_parameter("max_angular_vel", 0.6)

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

        self._is_auto = True
        self._latest_cmd = None
        self._last_cmd_time = 0.0

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_service(SetBool, "drivers/set_auto_mode", self.mode_callback)
        self.create_service(SetGait, "drivers/set_gait", self.gait_callback)
        self.create_service(Trigger, "drivers/emergency_stop", self.emergency_stop_callback)
        
        frequency = self.get_parameter("control_frequency").value
        self.create_timer(1.0 / frequency, self._control_loop)

    def cmd_vel_callback(self, msg):
        """Cache the latest velocity command and its arrival time."""
        self._latest_cmd = msg
        self._last_cmd_time = time.time()

    def mode_callback(self, request, response):
        """Enable or disable automatic cmd_vel control."""
        self._is_auto = request.data
        if not self._is_auto:
            self._latest_cmd = None
            self._dog.stop()
        response.success = True
        response.message = (
            "auto mode disabled set to manual"
            if not self._is_auto
            else "auto mode enabled"
        )
        self.get_logger().info(f"control mode: {response.message}")
        return response

    def gait_callback(self, request, response):
        """Switch the robot gait using the SDK."""
        try:
            self._dog.set_gait(request.gait_id)
            response.success = True
            response.message = (f"gait set to {request.gait_id}")
            self.get_logger().info( response.message)
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"failed to set gait: {error}")
        return response

    def emergency_stop_callback(self, request, response):
        """Force manual mode and immediately stop the robot."""
        self._is_auto = False
        self._latest_cmd = None
        try:
            self._dog.stop()
            response.success = True
            response.message = "robot stopped"
            self.get_logger().warning( "emergency stop triggered")
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"emergency stop failed: {error}")
        return response

    def update(self):
        """Forward cmd_vel to the robot while auto mode is enabled."""
        if not self._is_auto:
            return
        stale = (
            self._latest_cmd is None
            or (
                time.time() - self._last_cmd_time
                > self._watchdog_timeout
            )
        )
        if stale:
            self._dog.stop()
            self.get_logger().debug("stale cmd_vel: stopping")
            return

        vx = max(-self._max_linear_vel,min(self._max_linear_vel,self._latest_cmd.linear.x,),)
        vy = max(-self._max_linear_vel,min(self._max_linear_vel,self._latest_cmd.linear.y,),)
        yaw = max(-self._max_angular_vel,min(self._max_angular_vel,self._latest_cmd.angular.z,),)
        self._dog.move(vx, vy, yaw)

    def destroy_node(self):
        """Stop the robot before shutting down."""
        self._dog.stop()
        super().destroy_node()

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