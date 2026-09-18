#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from robot_localization.utils.pose_localization import (
    map_odom_from_pose_estimate,
    map_odom_from_tag,
    matrix_to_transform,
    pose_values_to_matrix,
    transform_to_matrix,
)


class LocalizationNode(Node):
    """Determines map->odom from one of three sources: (1) AprilTag detection, (2) RViz 2D Pose Estimate, or (3) FAST-LIO's own odometry."""
    def __init__(self):
        """Declare params, load tag poses, set up TF, broadcast fixed links, and wire both triggers."""
        super().__init__('localization_node')
        self.declare_parameter('tags_config_file', '')

        self._tag_poses = self.load_tag_poses(self.get_parameter('tags_config_file').value)

        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.broadcast_fixed_links()
        self.create_service(Trigger, 'localization/start', self.localization_callback)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.pose_estimate_callback, 10
        )

        self.get_logger().info(
            f"localization_node ready with {len(self._tag_poses)} configured tag(s)."
        )

    def load_tag_poses(self, filepath: str) -> dict:
        """Load a {tag_frame: {x,y,z,qx,qy,qz,qw}} JSON file into a tag_frame -> matrix map."""
        if not filepath:
            self.get_logger().warn('tags_config_file not set, no tags configured.')
            return {}

        with open(filepath, 'r') as f:
            raw = json.load(f)

        poses = {}
        for tag_frame, pose in raw.items():
            values = [pose['x'], pose['y'], pose['z'], pose['qx'], pose['qy'], pose['qz'], pose['qw']]
            poses[tag_frame] = pose_values_to_matrix(values)
        return poses

    def identity_transform(self, parent_frame: str, child_frame: str) -> TransformStamped:
        """Build a zero-translation, zero-rotation TransformStamped between two frames."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.rotation.w = 1.0
        return t

    def broadcast_fixed_links(self):
        """Publish the two links that are always identity (odom->camera_init, body->base_link),
        plus map->odom as identity too -- mode 1, "camera-based fixed origin": the robot's start
        position (wherever it's placed on the floor tape marker) IS the map origin, until an
        AprilTag or RViz trigger overrides it with something more precise."""
        self._static_broadcaster.sendTransform([
            self.identity_transform("map", "odom"),
            self.identity_transform("odom", "camera_init"),
            self.identity_transform("body", "base_link"),
        ])
        self._active_source = 'camera (fixed start-position origin)'

    def find_visible_tag(self, now, timeout):
        """Try each configured tag frame in turn, returning (tag_frame, cam_to_tag TF) for the first visible one."""
        for tag_frame in self._tag_poses:
            try:
                cam_to_tag = self._tf_buffer.lookup_transform("camera_color_optical_frame", tag_frame, now, timeout)
                return tag_frame, cam_to_tag
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        return None, None

    def lookup_caminit_to_body(self, now, timeout):
        """Look up FAST-LIO's camera_init->body transform, shared by both localization sources."""
        return self._tf_buffer.lookup_transform("camera_init", "body", now, timeout)

    def publish_map_to_odom(self, m_map_odom, source: str):
        """Broadcast the new map->odom (plus the two fixed links) and record which source set it."""
        stamp = self.get_clock().now().to_msg()
        map_to_odom = matrix_to_transform(m_map_odom, "map", "odom", stamp)

        self._static_broadcaster.sendTransform([
            map_to_odom,
            self.identity_transform("odom", "camera_init"),
            self.identity_transform("body", "base_link"),
        ])

        self._active_source = source
        t = map_to_odom.transform.translation
        self.get_logger().info(f'map->odom set from {source}: x={t.x:.3f} y={t.y:.3f}')

    def localization_callback(self, request, response):
        """AprilTag trigger: compute and rebroadcast map->odom from whichever configured tag is visible."""
        now = Time()
        timeout = Duration(seconds=1.0)

        if not self._tag_poses:
            message = 'No tags configured (tags_config_file not set or empty).'
            self.get_logger().error(message)
            response.success = False
            response.message = message
            return response

        tag_frame, cam_to_tag = self.find_visible_tag(now, timeout)
        if tag_frame is None:
            message = f'None of the configured tags are currently visible: {list(self._tag_poses)}'
            self.get_logger().error(message)
            response.success = False
            response.message = message
            return response

        try:
            base_to_cam = self._tf_buffer.lookup_transform("base_link", "camera_color_optical_frame", now, timeout)
            caminit_to_body = self.lookup_caminit_to_body(now, timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            message = f'Failed to look up TF for localization: {error}'
            self.get_logger().error(message)
            response.success = False
            response.message = message
            return response

        m_map_odom = map_odom_from_tag(
            m_map_tag=self._tag_poses[tag_frame],
            m_base_cam=transform_to_matrix(base_to_cam),
            m_cam_tag=transform_to_matrix(cam_to_tag),
            m_caminit_body=transform_to_matrix(caminit_to_body),
        )
        self.publish_map_to_odom(m_map_odom, f'apriltag "{tag_frame}"')

        response.success = True
        response.message = f'map->odom set from tag "{tag_frame}".'
        return response

    def pose_estimate_callback(self, msg: PoseWithCovarianceStamped):
        """RViz '2D Pose Estimate' trigger: set map->odom so base_link matches the clicked pose."""
        now = Time()
        timeout = Duration(seconds=1.0)
        try:
            caminit_to_body = self.lookup_caminit_to_body(now, timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            self.get_logger().error(f'2D Pose Estimate ignored, TF not ready: {error}')
            return

        m_map_odom = map_odom_from_pose_estimate(msg, transform_to_matrix(caminit_to_body))
        self.publish_map_to_odom(m_map_odom, 'RViz 2D Pose Estimate')


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