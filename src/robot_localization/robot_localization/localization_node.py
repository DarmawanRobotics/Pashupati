#!/usr/bin/env python3
import json
import math
import os
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_localization.utils.pose_localization import (
    map_base_from_tag,
    map_odom_from_map_base,
    matrix_to_transform,
    planar_delta,
    pose_msg_to_matrix,
    pose_values_to_matrix,
    transform_to_matrix,
)
from robot_localization.utils.tag_recording import average_poses, describe_pose, merge_into_file
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
import tf_transformations as tft
from visualization_msgs.msg import Marker, MarkerArray


class LocalizationNode(Node):
    """Bridges FAST-LIO frames into map->odom->base_link and corrects map->odom from AprilTags."""

    def __init__(self):
        """Declare params, load tag poses, and start the frame bridge."""
        super().__init__('localization_node')
        self.declare_parameter('tags_config_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lio_odom_frame', 'camera_init')
        self.declare_parameter('lio_body_frame', 'body')
        self.declare_parameter('imu_frame', 'livox_imu_frame')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_max_age_sec', 0.5)
        self.declare_parameter('auto_localize_on_start', True)
        self.declare_parameter('assume_start_origin', False)
        self.declare_parameter('tag_frames', ['base_map', 'charging_dock'])
        self.declare_parameter('tag_record_duration_sec', 2.0)
        self.declare_parameter('tags_record_file', '')
        self.declare_parameter('auto_correct_period_sec', 0.0)
        self.declare_parameter('max_correction_m', 0.5)
        self.declare_parameter('max_correction_deg', 15.0)

        p = self.get_parameter
        self._map = p('map_frame').value
        self._odom = p('odom_frame').value
        self._base = p('base_frame').value
        self._lio_odom = p('lio_odom_frame').value
        self._lio_body = p('lio_body_frame').value
        self._imu = p('imu_frame').value
        self._camera = p('camera_frame').value
        self._tag_max_age = Duration(seconds=float(p('tag_max_age_sec').value))
        self._max_corr_m = float(p('max_correction_m').value)
        self._max_corr_rad = math.radians(float(p('max_correction_deg').value))

        self._tag_poses = self.load_tag_poses(p('tags_config_file').value)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._m_base_imu = None
        self._m_map_odom = np.eye(4)
        self._localized = False
        self._active_source = 'none'

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, 'localization/status', latched)
        self._assume_start_origin = bool(p('assume_start_origin').value)
        self.publish_status()

        self._tags_file = p('tags_config_file').value
        self._tag_markers_pub = self.create_publisher(
            MarkerArray, 'localization/tag_markers', latched
        )
        self.publish_tag_markers()

        self.create_service(Trigger, 'localization/start', self.localization_callback)
        self.create_service(Trigger, 'localization/record_tags', self.record_tags_callback)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.pose_estimate_callback, 10
        )

        self._bridge_timer = self.create_timer(0.5, self.try_start_bridge)
        self._auto_start = bool(p('auto_localize_on_start').value)
        period = float(p('auto_correct_period_sec').value)
        if period > 0.0:
            self.create_timer(period, self.auto_correct)

        self.get_logger().info(f'localization_node ready with {len(self._tag_poses)} tag(s)')

    def load_tag_poses(self, filepath: str) -> dict:
        """Load a {tag_frame: {x,y,z,qx,qy,qz,qw}} JSON file into tag_frame -> 4x4 matrix."""
        if not filepath:
            self.get_logger().warn('tags_config_file not set, AprilTag localization disabled')
            return {}
        with open(filepath, 'r') as f:
            raw = json.load(f)
        return {
            frame: pose_values_to_matrix(
                [v['x'], v['y'], v['z'], v['qx'], v['qy'], v['qz'], v['qw']]
            )
            for frame, v in raw.items()
        }

    def try_start_bridge(self):
        """Wait for base_link->imu from the URDF, then publish the static frame bridge once."""
        try:
            tf = self._tf_buffer.lookup_transform(self._base, self._imu, Time())
        except TransformException as error:
            self.get_logger().warn(
                f'waiting for URDF TF {self._base}->{self._imu}: {error}',
                throttle_duration_sec=5.0,
            )
            return
        self._m_base_imu = transform_to_matrix(tf)
        self._bridge_timer.cancel()
        self.broadcast_static()
        self.get_logger().info('frame bridge up: map->odom->camera_init, body->base_link')
        if self._assume_start_origin:
            self._localized = True
            self._active_source = 'start position'
            self.publish_status()
        if self._auto_start and self._tag_poses:
            self._auto_timer = self.create_timer(1.0, self.auto_start_tick)

    def publish_status(self):
        """Publish the active map->odom source ('none' until localized), latched."""
        self._status_pub.publish(String(data=self._active_source if self._localized else 'none'))

    def broadcast_static(self):
        """Publish map->odom, odom->camera_init (= base->imu) and body->base_link (= imu->base)."""
        stamp = self.get_clock().now().to_msg()
        self._static_broadcaster.sendTransform(
            [
                matrix_to_transform(self._m_map_odom, self._map, self._odom, stamp),
                matrix_to_transform(self._m_base_imu, self._odom, self._lio_odom, stamp),
                matrix_to_transform(
                    np.linalg.inv(self._m_base_imu), self._lio_body, self._base, stamp
                ),
            ]
        )

    def set_map_odom(self, m_map_odom: np.ndarray, source: str):
        """Store and broadcast a new map->odom."""
        self._m_map_odom = m_map_odom
        self._localized = True
        self._active_source = source
        self.broadcast_static()
        self.publish_status()
        t = m_map_odom[:3, 3]
        yaw = math.degrees(math.atan2(m_map_odom[1, 0], m_map_odom[0, 0]))
        self.get_logger().info(f'map->odom from {source}: x={t[0]:.3f} y={t[1]:.3f} yaw={yaw:.1f}')

    def fresh_tag(self):
        """Return (tag_frame, cam->tag TF) for the first tag seen within tag_max_age_sec."""
        now = self.get_clock().now()
        for frame in self._tag_poses:
            try:
                tf = self._tf_buffer.lookup_transform(self._camera, frame, Time())
            except TransformException:
                continue
            if now - Time.from_msg(tf.header.stamp) <= self._tag_max_age:
                return frame, tf
        return None, None

    def map_odom_from_visible_tag(self):
        """Compute map->odom from a fresh tag: (matrix, tag_frame) or (None, reason)."""
        if self._m_base_imu is None:
            return None, 'frame bridge not ready'
        if not self._tag_poses:
            return None, 'no tags configured'
        frame, cam_tag = self.fresh_tag()
        if frame is None:
            return None, f'no fresh detection of {list(self._tag_poses)}'
        stamp = Time.from_msg(cam_tag.header.stamp)
        try:
            base_cam = self._tf_buffer.lookup_transform(self._base, self._camera, Time())
            odom_base = self._tf_buffer.lookup_transform(
                self._odom, self._base, stamp, Duration(seconds=0.2)
            )
        except TransformException as error:
            return None, f'TF lookup failed: {error}'
        m_map_base = map_base_from_tag(
            self._tag_poses[frame], transform_to_matrix(base_cam), transform_to_matrix(cam_tag)
        )
        return map_odom_from_map_base(m_map_base, transform_to_matrix(odom_base)), frame

    def localization_callback(self, request, response):
        """Service: set map->odom from whichever configured tag is currently visible."""
        m, info = self.map_odom_from_visible_tag()
        if m is None:
            response.success = False
            response.message = info
            self.get_logger().error(info)
            return response
        self.set_map_odom(m, f'apriltag "{info}"')
        response.success = True
        response.message = f'map->odom set from tag "{info}"'
        return response

    def auto_start_tick(self):
        """Retry tag localization every second until the first success."""
        m, info = self.map_odom_from_visible_tag()
        if m is None:
            self.get_logger().info(f'auto localize waiting: {info}', throttle_duration_sec=10.0)
            return
        self.set_map_odom(m, f'apriltag "{info}" (auto start)')
        self._auto_timer.cancel()

    def auto_correct(self):
        """Periodically correct drift from any visible tag, rejecting implausible jumps."""
        if not self._localized:
            return
        m, info = self.map_odom_from_visible_tag()
        if m is None:
            return
        dist, dyaw = planar_delta(m, self._m_map_odom)
        if dist > self._max_corr_m or dyaw > self._max_corr_rad:
            self.get_logger().warn(
                f'rejected correction from "{info}": {dist:.2f} m / {math.degrees(dyaw):.1f} deg'
            )
            return
        self.set_map_odom(m, f'apriltag "{info}" (auto correct)')

    def record_tags_callback(self, request, response):
        """Average the map pose of every visible tag in tag_frames and save it to the tag file."""
        duration = float(self.get_parameter('tag_record_duration_sec').value)
        frames = [f for f in self.get_parameter('tag_frames').value if f]
        samples = {frame: [] for frame in frames}
        end = time.monotonic() + duration
        while time.monotonic() < end:
            for frame in frames:
                m = self.fresh_map_tag(frame)
                if m is not None:
                    samples[frame].append(m)
            time.sleep(0.1)

        poses, lines = {}, []
        for frame, values in samples.items():
            if len(values) < 3:
                lines.append(f'{frame}: not seen')
                continue
            mean, pos_std, yaw_std = average_poses(values)
            poses[frame] = mean
            lines.append(
                f'{frame}: {describe_pose(mean)} n={len(values)} '
                f'std={pos_std * 100:.1f}cm/{yaw_std:.1f}deg'
            )
        if not poses:
            response.success = False
            response.message = 'no tag seen: ' + '; '.join(lines)
            return response

        path = self.get_parameter('tags_record_file').value or self._tags_file
        path = path or os.path.join(os.getcwd(), 'tag_config.json')
        merged = merge_into_file(path, poses)
        self._tag_poses = {
            f: pose_values_to_matrix([v['x'], v['y'], v['z'], v['qx'], v['qy'], v['qz'], v['qw']])
            for f, v in merged.items()
        }
        self.publish_tag_markers()
        response.success = True
        response.message = f'saved {len(poses)} tag(s) to {path}\n' + '\n'.join(lines)
        self.get_logger().info(response.message)
        return response

    def fresh_map_tag(self, frame: str):
        """Return map->tag as a 4x4 matrix if the tag was detected within tag_max_age_sec."""
        try:
            cam_tag = self._tf_buffer.lookup_transform(self._camera, frame, Time())
            if self.get_clock().now() - Time.from_msg(cam_tag.header.stamp) > self._tag_max_age:
                return None
            return transform_to_matrix(self._tf_buffer.lookup_transform(self._map, frame, Time()))
        except TransformException:
            return None

    def publish_tag_markers(self):
        """Publish the known tag poses as arrows + labels in the map frame (latched)."""
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, (frame, m) in enumerate(sorted(self._tag_poses.items())):
            qx, qy, qz, qw = (float(v) for v in tft.quaternion_from_matrix(m))
            for kind, offset in ((Marker.ARROW, 0), (Marker.TEXT_VIEW_FACING, 1000)):
                mk = Marker()
                mk.header.frame_id = self._map
                mk.ns = 'tags'
                mk.id = i + offset
                mk.type = kind
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, m[:3, 3])
                mk.pose.orientation.x, mk.pose.orientation.y = qx, qy
                mk.pose.orientation.z, mk.pose.orientation.w = qz, qw
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.3, 0.9, 1.0
                if kind == Marker.ARROW:
                    mk.scale.x, mk.scale.y, mk.scale.z = 0.3, 0.04, 0.04
                else:
                    mk.pose.position.z += 0.25
                    mk.scale.z = 0.15
                    mk.text = frame
                markers.markers.append(mk)
        self._tag_markers_pub.publish(markers)

    def pose_estimate_callback(self, msg: PoseWithCovarianceStamped):
        """Set map->odom so base_link lands on an RViz 2D Pose Estimate."""
        if self._m_base_imu is None:
            self.get_logger().error('2D Pose Estimate ignored, frame bridge not ready')
            return
        try:
            odom_base = self._tf_buffer.lookup_transform(self._odom, self._base, Time())
        except TransformException as error:
            self.get_logger().error(f'2D Pose Estimate ignored, TF not ready: {error}')
            return
        m = map_odom_from_map_base(
            pose_msg_to_matrix(msg.pose.pose), transform_to_matrix(odom_base)
        )
        self.set_map_odom(m, 'RViz 2D Pose Estimate')


def main(args=None):
    """Spin the localization node."""
    rclpy.init(args=args)
    node = LocalizationNode()
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
