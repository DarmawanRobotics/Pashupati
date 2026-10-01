#!/usr/bin/env python3
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import AvoidanceCommand, SectorScan
from robot_navigation.utils.avoidance.registry import create_avoidance

ALGORITHM_PARAMS = {
    'braitenberg': {
        'steering_gain': 1.5,
        'velocity_gain': 1.0,
        'steering_zone_center_deg': 45.0,
        'steering_zone_width_deg': 60.0,
        'velocity_zone_center_deg': 60.0,
        'velocity_zone_width_deg': 90.0,
        'following_zone_width_deg': 30.0,
        'smoothing': 0.85,
    },
    'vfh': {
        'min_gap_width_deg': 20.0,
        'steering_gain': 1.0,
        'velocity_gain': 1.0,
        'smoothing': 0.85,
    },
    'potential_field': {
        'repulsion_gain': 0.5,
        'steering_gain': 0.8,
        'lateral_gain': 0.3,
        'max_lateral': 0.2,
        'velocity_gain': 1.0,
        'smoothing': 0.8,
    },
    'follow_gap': {
        'bubble_radius': 0.45,
        'steering_gain': 1.0,
        'velocity_gain': 1.0,
        'smoothing': 0.85,
    },
}


class ObstacleAvoidanceNode(Node):
    """Runs a selectable reactive avoidance algorithm on SectorScan."""

    def __init__(self):
        """Declare params, build the initial algorithm, and wire the scan subscription/output."""
        super().__init__('obstacle_avoidance_node')
        self.declare_parameter('algorithm', 'vfh')
        self.declare_parameter('safe_distance', 1.5)
        self.declare_parameter('emergency_distance', 0.75)
        self.declare_parameter('emergency_cone_deg', 60.0)
        for algorithm, params in ALGORITHM_PARAMS.items():
            for name, value in params.items():
                self.declare_parameter(f'{algorithm}.{name}', value)

        self._algorithm_name = self.get_parameter('algorithm').value
        self._avoidance = create_avoidance(
            self._algorithm_name, self.build_params(self._algorithm_name)
        )

        self.create_subscription(SectorScan, 'perception/sector_scan', self.scan_callback, 10)
        self._avoidance_pub = self.create_publisher(AvoidanceCommand, 'navigation/avoidance', 10)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

    def build_params(self, name: str) -> dict:
        """Build constructor kwargs: shared distances plus the algorithm's namespaced params."""
        if name not in ALGORITHM_PARAMS:
            raise ValueError(
                f'unknown avoidance algorithm {name!r}, options are {list(ALGORITHM_PARAMS)}'
            )
        params = {
            key: float(self.get_parameter(key).value)
            for key in ('safe_distance', 'emergency_distance', 'emergency_cone_deg')
        }
        for key in ALGORITHM_PARAMS[name]:
            params[key] = float(self.get_parameter(f'{name}.{key}').value)
        return params

    def on_parameters_changed(self, params):
        """Switch algorithm at runtime."""
        for param in params:
            if param.name == 'algorithm':
                try:
                    self._avoidance = create_avoidance(param.value, self.build_params(param.value))
                except ValueError as error:
                    return SetParametersResult(successful=False, reason=str(error))
                self._algorithm_name = param.value
                self.get_logger().info(f'switched avoidance algorithm to {param.value}')
        return SetParametersResult(successful=True)

    def scan_callback(self, msg: SectorScan):
        """Run one avoidance update from the sector scan and publish the result."""
        result = self._avoidance.update(
            list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_max
        )
        out = AvoidanceCommand()
        out.header = msg.header
        out.steering_bias = float(result.steering_bias)
        out.lateral_bias = float(result.lateral_bias)
        out.velocity_scale = float(result.velocity_scale)
        out.emergency = bool(result.emergency)
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
