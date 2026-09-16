#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from livox_ros_driver2.msg import CustomMsg
from visualization_msgs.msg import Marker, MarkerArray

import tf_transformations
from tf2_ros import Buffer, TransformListener

from robot_interfaces.msg import SectorScan


class LidarSectorNode(Node):
    """Bins Livox points into fixed angular sectors and publishes a SectorScan."""
    def __init__(self):
        """Declare params, set up TF, and wire the subscription/publishers."""
        super().__init__('lidar_sector_perception_node')
        self.declare_parameter('num_sectors', 32)
        self.declare_parameter('fov_deg', 180.0)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('obstacle_z_min', 0.05)
        self.declare_parameter('obstacle_z_max', 1.20)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_lifetime_sec', 0.3)

        self._num_sectors = int(self.get_parameter('num_sectors').value)
        self._fov = math.radians(float(self.get_parameter('fov_deg').value))
        self._range_max = float(self.get_parameter('range_max').value)
        self._z_min = float(self.get_parameter('obstacle_z_min').value)
        self._z_max = float(self.get_parameter('obstacle_z_max').value)
        self._publish_markers = bool(self.get_parameter('publish_markers').value)
        self._marker_lifetime = Duration(seconds=float(self.get_parameter('marker_lifetime_sec').value)).to_msg()

        self._angle_min = -self._fov / 2.0
        self._angle_increment = self._fov / self._num_sectors

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._extrinsic_rotation = None
        self._extrinsic_translation = None

        self.create_subscription(CustomMsg, "/livox/lidar", self.livox_callback, 10)
        self._sectorscan_pub = self.create_publisher(SectorScan, "perception/sector_scan", 10)
        self._markers_pub = self.create_publisher(MarkerArray,"perception/markers", 10) if self._publish_markers else None

    def lookup_extrinsic(self) -> bool:
        """Look up and cache the static livox_frame -> base_link transform from TF."""
        try:
            tf = self._tf_buffer.lookup_transform("base_link", "livox_frame", rclpy.time.Time())
        except Exception as error:
            self.get_logger().warn(f'waiting for TF {"livox_frame"}->{("base_link")}: {error}', throttle_duration_sec=2.0)
            return False

        q = tf.transform.rotation
        self._extrinsic_rotation = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        t = tf.transform.translation
        self._extrinsic_translation = np.array([t.x, t.y, t.z])
        return True

    def range_to_color(self, r: float) -> ColorRGBA:
        """Map a range value to a red-to-green ColorRGBA for visualization."""
        t = max(0.0, min(1.0, r / self._range_max)) if self._range_max > 0 else 1.0
        if t < 0.5:
            ratio = t / 0.5
            return ColorRGBA(r=1.0, g=ratio, b=0.0, a=0.9)
        ratio = (t - 0.5) / 0.5
        return ColorRGBA(r=1.0 - ratio, g=1.0, b=0.0, a=0.9)

    def livox_callback(self, msg: CustomMsg):
        """Transform incoming points into base_link, bin them per sector, and publish."""
        if self._extrinsic_rotation is None and not self.lookup_extrinsic():
            return

        n = msg.point_num
        ranges = np.full(self._num_sectors, self._range_max, dtype=np.float32)

        if n > 0:
            x = np.fromiter((p.x for p in msg.points), dtype=np.float32, count=n)
            y = np.fromiter((p.y for p in msg.points), dtype=np.float32, count=n)
            z = np.fromiter((p.z for p in msg.points), dtype=np.float32, count=n)

            points = np.stack([x, y, z], axis=1) @ self._extrinsic_rotation.T + self._extrinsic_translation
            base_x, base_y, base_z = points[:, 0], points[:, 1], points[:, 2]

            height_mask = (base_z >= self._z_min) & (base_z <= self._z_max)
            base_x, base_y = base_x[height_mask], base_y[height_mask]

            if base_x.size > 0:
                r = np.hypot(base_x, base_y)
                theta = np.arctan2(base_y, base_x)

                valid = (r <= self._range_max) & (theta >= self._angle_min) & (theta < self._angle_min + self._fov)
                r, theta = r[valid], theta[valid]
                if r.size > 0:
                    idx = np.clip(
                        ((theta - self._angle_min) / self._angle_increment).astype(int), 0, self._num_sectors - 1
                    )
                    np.minimum.at(ranges, idx, r.astype(np.float32))
        self.publish_scan(msg.header, ranges)
        if self._publish_markers:
            self.publish_marker_array(msg.header, ranges)

    def publish_scan(self, header, ranges: np.ndarray):
        """Fill and publish a SectorScan message from the binned ranges."""
        out = SectorScan()
        out.header.stamp = header.stamp
        out.header.frame_id = "base_link"
        out.num_sectors = self._num_sectors
        out.angle_min = self._angle_min
        out.angle_increment = self._angle_increment
        out.range_max = self._range_max
        out.ranges = ranges.tolist()
        self._sectorscan_pub.publish(out)

    def publish_marker_array(self, header, ranges: np.ndarray):
        """Build and publish ray and point markers for RViz visualization."""
        ray_marker = Marker()
        ray_marker.header.stamp = header.stamp
        ray_marker.header.frame_id = "base_link"
        ray_marker.ns = 'sector_rays'
        ray_marker.id = 0
        ray_marker.type = Marker.LINE_LIST
        ray_marker.action = Marker.ADD
        ray_marker.pose.orientation.w = 1.0
        ray_marker.scale.x = 0.02
        ray_marker.lifetime = self._marker_lifetime

        point_marker = Marker()
        point_marker.header = ray_marker.header
        point_marker.ns = 'sector_points'
        point_marker.id = 1
        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = point_marker.scale.y = point_marker.scale.z = 0.08
        point_marker.lifetime = self._marker_lifetime

        origin = Point(x=0.0, y=0.0, z=0.0)
        for i, r in enumerate(ranges):
            theta = self._angle_min + (i + 0.5) * self._angle_increment
            color = self.range_to_color(float(r))
            ray_end = Point(
                x=float(self._range_max * math.cos(theta)),
                y=float(self._range_max * math.sin(theta)),
                z=0.0,
            )
            ray_marker.points.append(origin)
            ray_marker.points.append(ray_end)
            ray_marker.colors.append(color)
            ray_marker.colors.append(color)

            detected = Point(x=float(r * math.cos(theta)), y=float(r * math.sin(theta)), z=0.0)
            point_marker.points.append(detected)
            point_marker.colors.append(color)

        markers = MarkerArray()
        markers.markers.append(ray_marker)
        markers.markers.append(point_marker)
        self._markers_pub.publish(markers)


def main(args=None):
    """Spin the lidar sector perception node."""
    rclpy.init(args=args)
    node = LidarSectorNode()
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