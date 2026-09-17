#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PathLoaderNode(Node):
    """Loads a CSV waypoint file and publishes it once as a transient-local nav_msgs/Path."""
    def __init__(self):
        """Declare params, load the waypoint file, and start the periodic republish timer."""
        super().__init__('path_loader_node')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('republish_period', 2.0)

        self._waypoints_file = self.get_parameter('waypoints_file').value
        republish_period = float(self.get_parameter('republish_period').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._path_pub = self.create_publisher(Path, 'navigation/path', qos)
        self._path_msg = self.load_waypoints(self._waypoints_file)
        self.publish_path()
        self.create_timer(republish_period, self.publish_path)

    def load_waypoints(self, filepath: str) -> Path:
        """Read x,y,yaw_deg rows from filepath into a Path message, skipping comments."""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        if not filepath or not os.path.isfile(filepath):
            self.get_logger().warn(f'waypoints_file not found: "{filepath}", publishing empty path.')
            return path_msg

        with open(filepath, 'r') as f:
            for row in csv.reader(f):
                if not row or row[0].strip().startswith('#'):
                    continue
                x, y = float(row[0]), float(row[1])
                yaw = math.radians(float(row[2])) if len(row) > 2 else 0.0

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path_msg.poses.append(pose)

        return path_msg

    def publish_path(self):
        """Stamp and publish the loaded path."""
        self._path_msg.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(self._path_msg)


def main(args=None):
    """Spin the path loader node."""
    rclpy.init(args=args)
    node = PathLoaderNode()
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
