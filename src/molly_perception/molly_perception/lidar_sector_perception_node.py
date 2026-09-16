import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from livox_ros_driver2.msg import CustomMsg
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from molly_navigation_msgs.msg import SectorScan


def _color_for_range(r: float, range_max: float) -> ColorRGBA:
    """Dekat = merah, jauh = hijau, transisi lewat kuning. Cuma buat
    visualisasi -- gak dipakai algoritma apapun."""
    t = max(0.0, min(1.0, r / range_max)) if range_max > 0 else 1.0
    if t < 0.5:
        ratio = t / 0.5
        return ColorRGBA(r=1.0, g=ratio, b=0.0, a=0.9)
    ratio = (t - 0.5) / 0.5
    return ColorRGBA(r=1.0 - ratio, g=1.0, b=0.0, a=0.9)


class LidarSectorPerceptionNode(Node):
    def __init__(self):
        super().__init__('lidar_sector_perception_node')

        self.declare_parameter('livox_custom_topic', '/livox/lidar')
        self.declare_parameter('sectorscan_topic', '/molly/perception/sector_scan')
        self.declare_parameter('marker_topic', '/molly/perception/markers')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('num_sectors', 32)
        self.declare_parameter('fov_deg', 180.0)  # -90..+90 deg, depan aja, sama kek ICar's obstacle_1
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('obstacle_z_min', 0.05)
        self.declare_parameter('obstacle_z_max', 1.20)
        self.declare_parameter('lidar_offset_x', 0.0)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('lidar_offset_z', 0.0)
        self.declare_parameter('lidar_yaw_deg', 0.0)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_lifetime_sec', 0.3)

        self.livox_custom_topic = self.get_parameter('livox_custom_topic').value
        self.sectorscan_topic = self.get_parameter('sectorscan_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.num_sectors = int(self.get_parameter('num_sectors').value)
        self.fov = math.radians(float(self.get_parameter('fov_deg').value))
        self.range_max = float(self.get_parameter('range_max').value)
        self.z_min = float(self.get_parameter('obstacle_z_min').value)
        self.z_max = float(self.get_parameter('obstacle_z_max').value)
        self.publish_markers_enabled = bool(self.get_parameter('publish_markers').value)
        self.marker_lifetime = Duration(seconds=float(self.get_parameter('marker_lifetime_sec').value)).to_msg()

        self.offset_x = float(self.get_parameter('lidar_offset_x').value)
        self.offset_y = float(self.get_parameter('lidar_offset_y').value)
        self.offset_z = float(self.get_parameter('lidar_offset_z').value)
        yaw = math.radians(float(self.get_parameter('lidar_yaw_deg').value))
        self._cos_yaw = math.cos(yaw)
        self._sin_yaw = math.sin(yaw)

        self.angle_min = -self.fov / 2.0
        self.angle_increment = self.fov / self.num_sectors

        self.sub_cloud = self.create_subscription(
            CustomMsg, self.livox_custom_topic, self.cb_custom_msg, 10
        )
        self.pub_sector_scan = self.create_publisher(SectorScan, self.sectorscan_topic, 10)
        self.pub_markers = (
            self.create_publisher(MarkerArray, self.marker_topic, 10) if self.publish_markers_enabled else None
        )

        self.get_logger().info(
            f'lidar_sector_perception_node up. livox_custom_topic={self.livox_custom_topic} '
            f'num_sectors={self.num_sectors} fov_deg={math.degrees(self.fov):.0f} '
            f'markers={"on" if self.publish_markers_enabled else "off"}'
        )

    def cb_custom_msg(self, msg: CustomMsg):
        n = msg.point_num
        if n == 0:
            ranges = np.full(self.num_sectors, self.range_max, dtype=np.float32)
        else:
            # CustomPoint[] -> flat numpy arrays directly, no PointCloud2
            # byte-offset decoding needed.
            x = np.fromiter((p.x for p in msg.points), dtype=np.float32, count=n)
            y = np.fromiter((p.y for p in msg.points), dtype=np.float32, count=n)
            z = np.fromiter((p.z for p in msg.points), dtype=np.float32, count=n)

            # static 2D extrinsic: lidar frame -> base_link
            base_x = self._cos_yaw * x - self._sin_yaw * y + self.offset_x
            base_y = self._sin_yaw * x + self._cos_yaw * y + self.offset_y
            base_z = z + self.offset_z

            height_mask = (base_z >= self.z_min) & (base_z <= self.z_max)
            base_x, base_y = base_x[height_mask], base_y[height_mask]

            ranges = np.full(self.num_sectors, self.range_max, dtype=np.float32)
            if base_x.size > 0:
                r = np.hypot(base_x, base_y)
                theta = np.arctan2(base_y, base_x)

                valid = (r <= self.range_max) & (theta >= self.angle_min) & (theta < self.angle_min + self.fov)
                r, theta = r[valid], theta[valid]
                if r.size > 0:
                    idx = np.clip(
                        ((theta - self.angle_min) / self.angle_increment).astype(int), 0, self.num_sectors - 1
                    )
                    np.minimum.at(ranges, idx, r.astype(np.float32))  # nearest-per-sector, vectorized

        self.publish_scan(msg.header, ranges)
        if self.publish_markers_enabled:
            self.publish_marker_array(msg.header, ranges)

    def publish_scan(self, header, ranges: np.ndarray):
        out = SectorScan()
        out.header.stamp = header.stamp
        out.header.frame_id = self.base_frame
        out.num_sectors = self.num_sectors
        out.angle_min = self.angle_min
        out.angle_increment = self.angle_increment
        out.range_max = self.range_max
        out.ranges = ranges.tolist()
        self.pub_sector_scan.publish(out)

    def publish_marker_array(self, header, ranges: np.ndarray):
        ray_marker = Marker()
        ray_marker.header.stamp = header.stamp
        ray_marker.header.frame_id = self.base_frame
        ray_marker.ns = 'sector_rays'
        ray_marker.id = 0
        ray_marker.type = Marker.LINE_LIST
        ray_marker.action = Marker.ADD
        ray_marker.pose.orientation.w = 1.0
        ray_marker.scale.x = 0.02
        ray_marker.lifetime = self.marker_lifetime

        point_marker = Marker()
        point_marker.header = ray_marker.header
        point_marker.ns = 'sector_points'
        point_marker.id = 1
        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = point_marker.scale.y = point_marker.scale.z = 0.08
        point_marker.lifetime = self.marker_lifetime

        origin = Point(x=0.0, y=0.0, z=0.0)
        for i, r in enumerate(ranges):
            theta = self.angle_min + (i + 0.5) * self.angle_increment
            p = Point(x=float(r * math.cos(theta)), y=float(r * math.sin(theta)), z=0.0)
            color = _color_for_range(float(r), self.range_max)

            ray_marker.points.append(origin)
            ray_marker.points.append(p)
            ray_marker.colors.append(color)
            ray_marker.colors.append(color)

            point_marker.points.append(p)
            point_marker.colors.append(color)

        markers = MarkerArray()
        markers.markers.append(ray_marker)
        markers.markers.append(point_marker)
        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSectorPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
