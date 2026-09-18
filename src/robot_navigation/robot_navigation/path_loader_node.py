#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from robot_interfaces.msg import Waypoint, WaypointPath


class PathLoaderNode(Node):
    """Loads a CSV waypoint file (x,y,yaw_deg[,dwell_sec]) and publishes it as a Path (for RViz)
    and a WaypointPath (with per-point yaw and dwell metadata, for path_follower_node)."""

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
        self._waypoints_pub = self.create_publisher(WaypointPath, 'navigation/waypoints', qos)

        self._waypoints = self.load_waypoints(self._waypoints_file)
        self.publish_all()
        self.create_timer(republish_period, self.publish_all)

    def load_waypoints(self, filepath: str) -> list:
        """Read x,y,yaw_deg[,dwell_sec] rows from filepath, skipping comments and blank lines."""
        waypoints = []

        if not filepath or not os.path.isfile(filepath):
            self.get_logger().warn(f'waypoints_file not found: "{filepath}", publishing empty path.')
            return waypoints

        with open(filepath, 'r') as f:
            for row in csv.reader(f):
                if not row or row[0].strip().startswith('#'):
                    continue
                x, y = float(row[0]), float(row[1])
                yaw_deg = float(row[2]) if len(row) > 2 else 0.0
                dwell_sec = float(row[3]) if len(row) > 3 else 0.0
                waypoints.append((x, y, yaw_deg, dwell_sec))

        self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {filepath}')
        return waypoints

    def build_path_msg(self) -> Path:
        """Build the nav_msgs/Path used purely for RViz visualization."""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for x, y, yaw_deg, _ in self._waypoints:
            yaw = math.radians(yaw_deg)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)
        return path_msg

    def build_waypoint_path_msg(self) -> WaypointPath:
        """Build the WaypointPath carrying the yaw and dwell_sec metadata path_follower_node needs."""
        msg = WaypointPath()
        msg.header.frame_id = 'map'
        for x, y, yaw_deg, dwell_sec in self._waypoints:
            wp = Waypoint()
            wp.x = x
            wp.y = y
            wp.yaw = yaw_deg
            wp.dwell_sec = dwell_sec
            msg.waypoints.append(wp)
        return msg

    def publish_all(self):
        """Stamp and publish both the Path and WaypointPath representations."""
        stamp = self.get_clock().now().to_msg()

        path_msg = self.build_path_msg()
        path_msg.header.stamp = stamp
        self._path_pub.publish(path_msg)

        waypoint_msg = self.build_waypoint_path_msg()
        waypoint_msg.header.stamp = stamp
        self._waypoints_pub.publish(waypoint_msg)


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
