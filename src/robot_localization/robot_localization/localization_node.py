#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger

import tf2_ros
import tf_transformations as tft
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

class LocalizationNode(Node):
    """Determines the map->odom transform from whichever configured AprilTag is currently visible."""
    def __init__(self):
        """Declare params, load tag poses, set up TF, broadcast fixed links, and expose the trigger service."""
        super().__init__('localization_node')
        self.declare_parameter('tags_config_file', '')

        self._tag_poses = self.load_tag_poses(self.get_parameter('tags_config_file').value)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.broadcast_fixed_links()
        self.create_service(Trigger, 'localization/start', self.localization_callback)

        self.get_logger().info(
            f"localization_node ready with {len(self._tag_poses)} configured tag(s). "
            f"Call '{self.get_name()}/localization/start' while any configured tag is visible."
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
            poses[tag_frame] = self.pose_values_to_matrix(values)
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
        """Publish the two links that are always identity: odom->camera_init and body->base_link."""
        self._static_broadcaster.sendTransform([
            self.identity_transform("odom", "camera_init"),
            self.identity_transform("body", "base_link"),
        ])

    @staticmethod
    def transform_to_matrix(t: TransformStamped):
        """Convert a TransformStamped into a 4x4 homogeneous transformation matrix."""
        trans = t.transform.translation
        rot = t.transform.rotation
        m = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        m[0, 3] = trans.x
        m[1, 3] = trans.y
        m[2, 3] = trans.z
        return m

    @staticmethod
    def matrix_to_transform(m, parent_frame, child_frame, stamp) -> TransformStamped:
        """Convert a 4x4 homogeneous transformation matrix into a TransformStamped."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(m[0, 3])
        t.transform.translation.y = float(m[1, 3])
        t.transform.translation.z = float(m[2, 3])
        q = tft.quaternion_from_matrix(m)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

    @staticmethod
    def pose_values_to_matrix(values):
        """Convert a [x, y, z, qx, qy, qz, qw] list into a 4x4 homogeneous matrix."""
        x, y, z, qx, qy, qz, qw = values
        m = tft.quaternion_matrix([qx, qy, qz, qw])
        m[0, 3] = x
        m[1, 3] = y
        m[2, 3] = z
        return m

    def find_visible_tag(self, now, timeout):
        """Try each configured tag frame in turn, returning (tag_frame, cam_to_tag TF) for the first visible one."""
        for tag_frame in self._tag_poses:
            try:
                cam_to_tag = self._tf_buffer.lookup_transform("camera_color_optical_frame", tag_frame, now, timeout)
                return tag_frame, cam_to_tag
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        return None, None

    def localization_callback(self, request, response):
        """Compute and rebroadcast map->odom from whichever configured AprilTag is currently visible."""
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
            caminit_to_body = self._tf_buffer.lookup_transform("camera_init", "body", now, timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            message = f'Failed to look up TF for localization: {error}'
            self.get_logger().error(message)
            response.success = False
            response.message = message
            return response

        m_base_cam = self.transform_to_matrix(base_to_cam)
        m_cam_tag = self.transform_to_matrix(cam_to_tag)
        m_caminit_body = self.transform_to_matrix(caminit_to_body)
        m_map_tag = self._tag_poses[tag_frame]

        m_base_tag = m_base_cam @ m_cam_tag
        m_map_base = m_map_tag @ tft.inverse_matrix(m_base_tag)
        m_map_odom = m_map_base @ tft.inverse_matrix(m_caminit_body)

        stamp = self.get_clock().now().to_msg()
        map_to_odom = self.matrix_to_transform(m_map_odom, "map", "odom", stamp)

        self._static_broadcaster.sendTransform([
            map_to_odom,
            self.identity_transform("odom", "camera_init"),
            self.identity_transform("body", "base_link")
        ])

        t = map_to_odom.transform.translation
        self.get_logger().info(
            f'Localization succeeded from tag "{tag_frame}". New map->odom: x={t.x:.3f} y={t.y:.3f} z={t.z:.3f}'
        )
        response.success = True
        response.message = f'map->odom set from tag "{tag_frame}".'
        return response

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