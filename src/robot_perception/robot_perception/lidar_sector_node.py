#!/usr/bin/env python3
import math

from geometry_msgs.msg import Point
from livox_ros_driver2.msg import CustomMsg
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from robot_interfaces.msg import SectorScan
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray


class LidarSectorNode(Node):
    """Bins lidar points into fixed angular sectors and publishes a SectorScan."""

    def __init__(self):
        """Declare params, set up TF, and wire the subscription/publishers."""
        super().__init__('lidar_sector_node')
        self.declare_parameter('input_type', 'livox')
        self.declare_parameter('pointcloud_topic', '/cloud_registered_body')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'livox_frame')
        self.declare_parameter('num_sectors', 32)
        self.declare_parameter('fov_deg', 180.0)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('obstacle_z_min', -0.25)
        self.declare_parameter('obstacle_z_max', 1.20)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_lifetime_sec', 0.3)
        self.declare_parameter('corridor_x_min', 0.0)
        self.declare_parameter('corridor_x_max', 5.0)
        self.declare_parameter('corridor_y_min', -1.0)
        self.declare_parameter('corridor_y_max', 1.0)
        self.declare_parameter('self_filter_x_min', -0.40)
        self.declare_parameter('self_filter_x_max', 0.40)
        self.declare_parameter('self_filter_y_min', -0.28)
        self.declare_parameter('self_filter_y_max', 0.28)

        self._num_sectors = int(self.get_parameter('num_sectors').value)
        self._fov = math.radians(float(self.get_parameter('fov_deg').value))
        self._range_max = float(self.get_parameter('range_max').value)
        self._range_min = float(self.get_parameter('range_min').value)
        self._z_min = float(self.get_parameter('obstacle_z_min').value)
        self._z_max = float(self.get_parameter('obstacle_z_max').value)
        self._publish_markers = bool(self.get_parameter('publish_markers').value)
        self._marker_lifetime = Duration(
            seconds=float(self.get_parameter('marker_lifetime_sec').value)
        ).to_msg()
        self._corridor_x_min = float(self.get_parameter('corridor_x_min').value)
        self._corridor_x_max = float(self.get_parameter('corridor_x_max').value)
        self._corridor_y_min = float(self.get_parameter('corridor_y_min').value)
        self._corridor_y_max = float(self.get_parameter('corridor_y_max').value)
        self._self_box = [
            float(self.get_parameter(f'self_filter_{k}').value)
            for k in ('x_min', 'x_max', 'y_min', 'y_max')
        ]

        self._angle_min = -self._fov / 2.0
        self._angle_increment = self._fov / self._num_sectors

        self._base_frame = self.get_parameter('base_frame').value
        self._fallback_frame = self.get_parameter('lidar_frame').value
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._extrinsics: dict[str, tuple[np.ndarray, np.ndarray]] = {}

        input_type = self.get_parameter('input_type').value
        if input_type == 'pointcloud2':
            topic = self.get_parameter('pointcloud_topic').value
            self.create_subscription(PointCloud2, topic, self.pointcloud_callback, 10)
        elif input_type == 'livox':
            self.create_subscription(CustomMsg, 'livox/lidar', self.livox_callback, 10)
        else:
            raise ValueError(f"input_type must be 'livox' or 'pointcloud2', got {input_type!r}")
        self._sectorscan_pub = self.create_publisher(SectorScan, 'perception/sector_scan', 10)
        self._markers_pub = (
            self.create_publisher(MarkerArray, 'perception/obstacles', 10)
            if self._publish_markers
            else None
        )

    def extrinsic(self, frame_id: str):
        """Return the cached (rotation, translation) from frame_id to base_frame, or None."""
        if frame_id not in self._extrinsics:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._base_frame, frame_id, rclpy.time.Time()
                )
            except TransformException as error:
                self.get_logger().warn(
                    f'waiting for TF {frame_id} -> {self._base_frame}: {error}',
                    throttle_duration_sec=2.0,
                )
                return None
            q, t = tf.transform.rotation, tf.transform.translation
            rotation = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            self._extrinsics[frame_id] = (rotation, np.array([t.x, t.y, t.z]))
        return self._extrinsics[frame_id]

    def range_to_color(self, r: float) -> ColorRGBA:
        """Map a range value to a red-to-green ColorRGBA for visualization."""
        t = max(0.0, min(1.0, r / self._range_max)) if self._range_max > 0 else 1.0
        if t < 0.5:
            ratio = t / 0.5
            return ColorRGBA(r=1.0, g=ratio, b=0.0, a=0.9)
        ratio = (t - 0.5) / 0.5
        return ColorRGBA(r=1.0 - ratio, g=1.0, b=0.0, a=0.9)

    def corridor_ray_length(self, theta: float) -> float:
        """Clamp range_max to the rectangular corridor at this angle (RViz rays only)."""
        length = self._range_max
        sin_t, cos_t = math.sin(theta), math.cos(theta)

        if sin_t != 0.0:
            if length * sin_t > self._corridor_y_max:
                length = self._corridor_y_max / sin_t
            elif length * sin_t < self._corridor_y_min:
                length = self._corridor_y_min / sin_t

        if cos_t != 0.0:
            if length * cos_t > self._corridor_x_max:
                length = self._corridor_x_max / cos_t
            elif length * cos_t < self._corridor_x_min:
                length = self._corridor_x_min / cos_t

        return max(0.0, length)

    def livox_callback(self, msg: CustomMsg):
        """Convert a Livox CustomMsg to an Nx3 array and process it."""
        n = msg.point_num
        xyz = np.empty((n, 3), dtype=np.float32)
        if n > 0:
            xyz[:, 0] = np.fromiter((p.x for p in msg.points), dtype=np.float32, count=n)
            xyz[:, 1] = np.fromiter((p.y for p in msg.points), dtype=np.float32, count=n)
            xyz[:, 2] = np.fromiter((p.z for p in msg.points), dtype=np.float32, count=n)
        self.process(msg.header, xyz)

    def pointcloud_callback(self, msg: PointCloud2):
        """Read x/y/z straight from the PointCloud2 buffer (no per-point Python objects)."""
        offsets = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= offsets.keys():
            self.get_logger().error('PointCloud2 has no x/y/z fields', throttle_duration_sec=5.0)
            return
        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': ['<f4' if not msg.is_bigendian else '>f4'] * 3,
            'offsets': [offsets['x'], offsets['y'], offsets['z']],
            'itemsize': msg.point_step,
        })
        cloud = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        xyz = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(np.float32)
        self.process(msg.header, xyz[np.isfinite(xyz).all(axis=1)])

    def process(self, header, xyz: np.ndarray):
        """Transform points into base_frame, bin them per sector, and publish."""
        extrinsic = self.extrinsic(header.frame_id or self._fallback_frame)
        if extrinsic is None:
            return
        rotation, translation = extrinsic
        ranges = np.full(self._num_sectors, self._range_max, dtype=np.float32)

        if xyz.shape[0] > 0:
            points = xyz @ rotation.T + translation
            base_x, base_y, base_z = points[:, 0], points[:, 1], points[:, 2]

            sx0, sx1, sy0, sy1 = self._self_box
            on_body = (base_x > sx0) & (base_x < sx1) & (base_y > sy0) & (base_y < sy1)
            keep = (base_z >= self._z_min) & (base_z <= self._z_max) & ~on_body
            base_x, base_y = base_x[keep], base_y[keep]

            if base_x.size > 0:
                r = np.hypot(base_x, base_y)
                theta = np.arctan2(base_y, base_x)

                valid = (
                    (r >= self._range_min)
                    & (r <= self._range_max)
                    & (theta >= self._angle_min)
                    & (theta < self._angle_min + self._fov)
                )
                r, theta = r[valid], theta[valid]
                if r.size > 0:
                    idx = np.clip(
                        ((theta - self._angle_min) / self._angle_increment).astype(int),
                        0,
                        self._num_sectors - 1,
                    )
                    np.minimum.at(ranges, idx, r.astype(np.float32))

        self.publish_scan(header, ranges)
        if self._publish_markers:
            self.publish_marker_array(header, ranges)

    def publish_scan(self, header, ranges: np.ndarray):
        """Fill and publish a SectorScan message from the binned ranges."""
        out = SectorScan()
        out.header.stamp = header.stamp
        out.header.frame_id = self._base_frame
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
        ray_marker.header.frame_id = self._base_frame
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
        detection_eps = 1e-3
        for i, r in enumerate(ranges):
            r = float(r)
            theta = self._angle_min + (i + 0.5) * self._angle_increment
            color = self.range_to_color(r)

            ray_length = self.corridor_ray_length(theta)
            ray_end = Point(
                x=float(ray_length * math.cos(theta)),
                y=float(ray_length * math.sin(theta)),
                z=0.0,
            )
            ray_marker.points.append(origin)
            ray_marker.points.append(ray_end)
            ray_marker.colors.append(color)
            ray_marker.colors.append(color)

            has_detection = r < (self._range_max - detection_eps)
            within_corridor = r <= ray_length + detection_eps
            if has_detection and within_corridor:
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
