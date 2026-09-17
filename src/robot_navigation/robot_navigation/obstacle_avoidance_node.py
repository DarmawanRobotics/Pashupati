#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from robot_interfaces.msg import AvoidanceCommand, SectorScan

from robot_navigation.utils.braitenberg_avoidance import BraitenbergAvoidance


class ObstacleAvoidanceNode(Node):
    """Runs discrete-sector Braitenberg avoidance on SectorScan and publishes an AvoidanceCommand."""

    def __init__(self):
        """Declare params, build the avoidance model, and wire the scan subscription/output."""
        super().__init__('obstacle_avoidance_node')
        self.declare_parameter('safe_distance', 3.0)
        self.declare_parameter('emergency_distance', 0.4)
        self.declare_parameter('steering_gain', 1.5)
        self.declare_parameter('velocity_gain', 1.0)
        self.declare_parameter('steering_zone_center_deg', 45.0)
        self.declare_parameter('steering_zone_width_deg', 60.0)
        self.declare_parameter('velocity_zone_center_deg', 60.0)
        self.declare_parameter('velocity_zone_width_deg', 90.0)
        self.declare_parameter('following_zone_width_deg', 30.0)
        self.declare_parameter('emergency_cone_deg', 60.0)
        self.declare_parameter('smoothing', 0.85)

        self._avoidance = BraitenbergAvoidance(
            safe_distance=float(self.get_parameter('safe_distance').value),
            emergency_distance=float(self.get_parameter('emergency_distance').value),
            steering_gain=float(self.get_parameter('steering_gain').value),
            velocity_gain=float(self.get_parameter('velocity_gain').value),
            steering_zone_center_deg=float(self.get_parameter('steering_zone_center_deg').value),
            steering_zone_width_deg=float(self.get_parameter('steering_zone_width_deg').value),
            velocity_zone_center_deg=float(self.get_parameter('velocity_zone_center_deg').value),
            velocity_zone_width_deg=float(self.get_parameter('velocity_zone_width_deg').value),
            following_zone_width_deg=float(self.get_parameter('following_zone_width_deg').value),
            emergency_cone_deg=float(self.get_parameter('emergency_cone_deg').value),
            smoothing=float(self.get_parameter('smoothing').value),
        )
        self.create_subscription(SectorScan, 'perception/sector_scan', self.scan_callback, 10)
        self._avoidance_pub = self.create_publisher(AvoidanceCommand, 'navigation/avoidance', 10)

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
