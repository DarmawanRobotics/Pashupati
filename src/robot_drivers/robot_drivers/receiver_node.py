"""ROS2 node: polls and publishes the dog's battery and control-mode state."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from robot_drivers.dog_socket import CTRL_MODE_NAMES, DogSocket


class ReceiverNode(Node):
    """Publishes battery percentage and a human-readable control-mode string."""

    def __init__(self):
        """Declare params, connect the socket, and start the poll timer."""
        super().__init__("robot_driver_receiver")
        self.declare_parameter("sdk_dir", "")
        self.declare_parameter("sdk_module_name", "mc_sdk_l1W_py")
        self.declare_parameter("dog_ip", "192.168.234.1")
        self.declare_parameter("local_ip", "192.168.234.13")
        self.declare_parameter("local_port", 43989)
        self.declare_parameter("poll_rate", 1.0)

        self._dog = DogSocket(
            self.get_parameter("sdk_dir").value,
            self.get_parameter("sdk_module_name").value,
            self.get_parameter("local_ip").value,
            self.get_parameter("local_port").value,
            self.get_parameter("dog_ip").value,
        )
        self._dog.connect()


        self._battery_pub = self.create_publisher(BatteryState, "drivers/battery", 10)
        self._state_pub = self.create_publisher(String, "drivers/robot_state", 10)

        rate = self.get_parameter("poll_rate").value
        self.create_timer(1.0 / rate, self._update_callback)

    def _update_callback(self):
        """Read battery + control mode from the SDK and publish both."""
        connected = self._dog.is_connected()
        battery = self._dog.get_battery()
        mode = self._dog.get_ctrl_mode()

        if battery is not None:
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.percentage = battery / 100.0
            battery_msg.present = connected
            self._battery_pub.publish(battery_msg)

        mode_name = CTRL_MODE_NAMES.get(mode, "UNKNOWN")
        state_msg = String()
        state_msg.data = f"connected={connected} mode={mode_name}({mode}) battery={battery}"
        self._state_pub.publish(state_msg)

def main(args=None):
    """Spin the receiver node."""
    rclpy.init(args=args)
    node = ReceiverNode()
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
