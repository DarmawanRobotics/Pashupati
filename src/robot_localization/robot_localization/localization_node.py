#!/usr/bin/env python3
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from robot_localization.utils.pose_localization import (
    map_base_from_tag,
    map_odom_from_map_base,
    matrix_to_transform,
    planar_delta,
    pose_msg_to_matrix,
    pose_values_to_matrix,
    transform_to_matrix,
)


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

        self.create_service(Trigger, 'localization/start', self.localization_callback)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.pose_estimate_callback, 10)

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
            frame: pose_values_to_matrix([v['x'], v['y'], v['z'], v['qx'], v['qy'], v['qz'], v['qw']])
            for frame, v in raw.items()
        }

    def try_start_bridge(self):
        """Wait for base_link->imu from the URDF, then publish the static frame bridge once."""
        try:
            tf = self._tf_buffer.lookup_transform(self._base, self._imu, Time())
        except TransformException as error:
            self.get_logger().warn(f'waiting for URDF TF {self._base}->{self._imu}: {error}',
                                   throttle_duration_sec=5.0)
            return
        self._m_base_imu = transform_to_matrix(tf)
        self._bridge_timer.cancel()
        self.broadcast_static()
        self.get_logger().info('frame bridge up: map->odom->camera_init, body->base_link')
        if self._auto_start and self._tag_poses:
            self._auto_timer = self.create_timer(1.0, self.auto_start_tick)

    def broadcast_static(self):
        """Publish map->odom, odom->camera_init (= base->imu) and body->base_link (= imu->base)."""
        stamp = self.get_clock().now().to_msg()
        self._static_broadcaster.sendTransform([
            matrix_to_transform(self._m_map_odom, self._map, self._odom, stamp),
            matrix_to_transform(self._m_base_imu, self._odom, self._lio_odom, stamp),
            matrix_to_transform(np.linalg.inv(self._m_base_imu), self._lio_body, self._base, stamp),
        ])

    def set_map_odom(self, m_map_odom: np.ndarray, source: str):
        """Store and broadcast a new map->odom."""
        self._m_map_odom = m_map_odom
        self._localized = True
        self._active_source = source
        self.broadcast_static()
        t = m_map_odom[:3, 3]
        yaw = math.degrees(math.atan2(m_map_odom[1, 0], m_map_odom[0, 0]))
        self.get_logger().info(f'map->odom from {source}: x={t[0]:.3f} y={t[1]:.3f} yaw={yaw:.1f}')

    def fresh_tag(self):
        """Return (tag_frame, cam->tag TF) for the first configured tag seen within tag_max_age_sec."""
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
        """Compute map->odom from a fresh tag detection; returns (matrix, tag_frame) or (None, reason)."""
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
            odom_base = self._tf_buffer.lookup_transform(self._odom, self._base, stamp, Duration(seconds=0.2))
        except TransformException as error:
            return None, f'TF lookup failed: {error}'
        m_map_base = map_base_from_tag(
            self._tag_poses[frame], transform_to_matrix(base_cam), transform_to_matrix(cam_tag))
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
                f'rejected correction from "{info}": {dist:.2f} m / {math.degrees(dyaw):.1f} deg')
            return
        self.set_map_odom(m, f'apriltag "{info}" (auto correct)')

    def pose_estimate_callback(self, msg: PoseWithCovarianceStamped):
        """RViz 2D Pose Estimate: set map->odom so base_link lands on the clicked pose."""
        if self._m_base_imu is None:
            self.get_logger().error('2D Pose Estimate ignored, frame bridge not ready')
            return
        try:
            odom_base = self._tf_buffer.lookup_transform(self._odom, self._base, Time())
        except TransformException as error:
            self.get_logger().error(f'2D Pose Estimate ignored, TF not ready: {error}')
            return
        m = map_odom_from_map_base(pose_msg_to_matrix(msg.pose.pose), transform_to_matrix(odom_base))
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
