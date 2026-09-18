#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult

from robot_interfaces.msg import AvoidanceCommand, SectorScan

from robot_navigation.utils.avoidance.registry import create_avoidance


class ObstacleAvoidanceNode(Node):
    """Runs a selectable reactive avoidance algorithm on SectorScan, publishing an AvoidanceCommand."""

    def __init__(self):
        """Declare params, build the initial avoidance algorithm, and wire the scan subscription/output."""
        super().__init__('obstacle_avoidance_node')
        self.declare_parameter('algorithm', 'braitenberg')
        self.declare_parameter('safe_distance', 3.0)
        self.declare_parameter('emergency_distance', 0.4)
        self.declare_parameter('emergency_cone_deg', 60.0)

        self.declare_parameter('braitenberg.steering_gain', 1.5)
        self.declare_parameter('braitenberg.velocity_gain', 1.0)
        self.declare_parameter('braitenberg.steering_zone_center_deg', 45.0)
        self.declare_parameter('braitenberg.steering_zone_width_deg', 60.0)
        self.declare_parameter('braitenberg.velocity_zone_center_deg', 60.0)
        self.declare_parameter('braitenberg.velocity_zone_width_deg', 90.0)
        self.declare_parameter('braitenberg.following_zone_width_deg', 30.0)
        self.declare_parameter('braitenberg.smoothing', 0.85)

        self.declare_parameter('vfh.min_gap_width_deg', 20.0)
        self.declare_parameter('vfh.steering_gain', 1.0)
        self.declare_parameter('vfh.velocity_gain', 1.0)
        self.declare_parameter('vfh.smoothing', 0.85)

        self._algorithm_name = self.get_parameter('algorithm').value
        self._avoidance = create_avoidance(self._algorithm_name, self.build_avoidance_params(self._algorithm_name))

        self.create_subscription(SectorScan, 'perception/sector_scan', self.scan_callback, 10)
        self._avoidance_pub = self.create_publisher(AvoidanceCommand, 'navigation/avoidance', 10)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

    def build_avoidance_params(self, name: str) -> dict:
        """Collect the constructor kwargs for an avoidance algorithm name from its declared parameters."""
        safe_distance = float(self.get_parameter('safe_distance').value)
        emergency_distance = float(self.get_parameter('emergency_distance').value)
        emergency_cone_deg = float(self.get_parameter('emergency_cone_deg').value)

        if name == 'braitenberg':
            return dict(
                safe_distance=safe_distance,
                emergency_distance=emergency_distance,
                emergency_cone_deg=emergency_cone_deg,
                steering_gain=float(self.get_parameter('braitenberg.steering_gain').value),
                velocity_gain=float(self.get_parameter('braitenberg.velocity_gain').value),
                steering_zone_center_deg=float(self.get_parameter('braitenberg.steering_zone_center_deg').value),
                steering_zone_width_deg=float(self.get_parameter('braitenberg.steering_zone_width_deg').value),
                velocity_zone_center_deg=float(self.get_parameter('braitenberg.velocity_zone_center_deg').value),
                velocity_zone_width_deg=float(self.get_parameter('braitenberg.velocity_zone_width_deg').value),
                following_zone_width_deg=float(self.get_parameter('braitenberg.following_zone_width_deg').value),
                smoothing=float(self.get_parameter('braitenberg.smoothing').value),
            )
        elif name == 'vfh':
            return dict(
                safe_distance=safe_distance,
                emergency_distance=emergency_distance,
                emergency_cone_deg=emergency_cone_deg,
                min_gap_width_deg=float(self.get_parameter('vfh.min_gap_width_deg').value),
                steering_gain=float(self.get_parameter('vfh.steering_gain').value),
                velocity_gain=float(self.get_parameter('vfh.velocity_gain').value),
                smoothing=float(self.get_parameter('vfh.smoothing').value),
            )
        raise ValueError(f'unknown avoidance algorithm: {name!r}')

    def switch_algorithm(self, name: str):
        """Instantiate the requested avoidance algorithm."""
        self._avoidance = create_avoidance(name, self.build_avoidance_params(name))
        self._algorithm_name = name
        self.get_logger().info(f'switched avoidance algorithm to {name}')

    def on_parameters_changed(self, params):
        """Apply a runtime change to the avoidance algorithm selection immediately."""
        for param in params:
            if param.name == 'algorithm':
                try:
                    self.switch_algorithm(param.value)
                except ValueError as error:
                    return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def scan_callback(self, msg: SectorScan):
        """Run one avoidance update from the sector scan and publish the result."""
        result = self._avoidance.update(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max)

        out = AvoidanceCommand()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.steering_bias = result.steering_bias
        out.velocity_scale = result.velocity_scale
        out.emergency = result.emergency
        self._avoidance_pub.publish(out)


def main(args=None):
    """Spin the obstacle avoidance node."""
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
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
