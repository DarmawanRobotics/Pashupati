#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
import tf_transformations as tft


class LocalizationNode(Node):
    """ Node for determining the map->odom transform based on AprilTag detection and FAST-LIO odometry. """
    def __init__(self):
        super().__init__('localization_node')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('tag_frame', 'tag36h11:0')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_init_frame', 'camera_init')
        self.declare_parameter('body_frame', 'body')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tag_pose_in_map', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]) 
        
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_init_frame = self.get_parameter('camera_init_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        self._static_broadcaster = StaticTransformBroadcaster(self)

        # buffer+listener untuk baca TF dinamis (camera->tag, camera_init->body)
        # dan TF statis robot (base_link->camera, dari URDF/robot_state_publisher)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # publish link2 identity yang memang selalu tetap
        self.broadcast_fixed_links()

        # service pemicu localization
        self._trigger_srv = self.create_service(
            Trigger, '~/trigger_localization', self.on_trigger_localization)

        self.get_logger().info(
            "localization_node siap. Panggil service "
            f"'{self.get_name()}/trigger_localization' saat robot menghadap tag "
            "untuk menentukan map->odom."
        )

    # ---------------- util transform ----------------

    def identity_transform(self, parent_frame: str, child_frame: str) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.rotation.w = 1.0
        return t

    def broadcast_fixed_links(self):
        """Link yang memang selalu identity: odom->camera_init dan body->base_link.
        map->odom TIDAK dipublish di sini, menunggu trigger pertama."""
        self._static_broadcaster.sendTransform([
            self.identity_transform(self.odom_frame, self.camera_init_frame),
            self.identity_transform(self.body_frame, self.base_frame),
        ])

    @staticmethod
    def _tf_to_matrix(t: TransformStamped):
        trans = t.transform.translation
        rot = t.transform.rotation
        m = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        m[0, 3] = trans.x
        m[1, 3] = trans.y
        m[2, 3] = trans.z
        return m

    @staticmethod
    def _matrix_to_transform(m, parent_frame, child_frame, stamp) -> TransformStamped:
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

    def _param_pose_to_matrix(self, values):
        x, y, z, qx, qy, qz, qw = values
        m = tft.quaternion_matrix([qx, qy, qz, qw])
        m[0, 3] = x
        m[1, 3] = y
        m[2, 3] = z
        return m

    # ---------------- inti: trigger localization ----------------

    def on_trigger_localization(self, request, response):
        try:
            now = Time()  # ambil transform paling baru yang tersedia
            timeout = Duration(seconds=1.0)

            # 1) base_link -> camera  (statis, dari robot_state_publisher)
            base_to_cam = self._tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, now, timeout)

            # 2) camera -> tag  (dinamis, dari node deteksi AprilTag)
            cam_to_tag = self._tf_buffer.lookup_transform(
                self.camera_frame, self.tag_frame, now, timeout)

            # 3) camera_init -> body  (dinamis, dari FAST-LIO)
            caminit_to_body = self._tf_buffer.lookup_transform(
                self.camera_init_frame, self.body_frame, now, timeout)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            msg = f"Gagal ambil TF untuk localization: {e}"
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        # matriks 4x4
        M_base_cam = self._tf_to_matrix(base_to_cam)      # base_link -> camera
        M_cam_tag = self._tf_to_matrix(cam_to_tag)         # camera -> tag
        M_caminit_body = self._tf_to_matrix(caminit_to_body)  # camera_init -> body
        M_map_tag = self._param_pose_to_matrix(
            self.get_parameter('tag_pose_in_map').value)   # map -> tag (biasanya identity)

        # base_link -> tag
        M_base_tag = M_base_cam @ M_cam_tag

        # map -> base_link  =  (map -> tag) * (base_link -> tag)^-1
        M_map_base = M_map_tag @ tft.inverse_matrix(M_base_tag)

        # map -> odom = (map->base_link) * (camera_init->body)^-1
        #   karena odom->camera_init = I dan body->base_link = I,
        #   sehingga map->base_link = map->odom * camera_init->body
        M_map_odom = M_map_base @ tft.inverse_matrix(M_caminit_body)

        stamp = self.get_clock().now().to_msg()
        map_to_odom = self._matrix_to_transform(
            M_map_odom, self.map_frame, self.odom_frame, stamp)

        # publish ulang SEMUA static transform (map->odom baru + 2 link tetap)
        self._static_broadcaster.sendTransform([
            map_to_odom,
            self.identity_transform(self.odom_frame, self.camera_init_frame),
            self.identity_transform(self.body_frame, self.base_frame),
        ])

        t = map_to_odom.transform.translation
        self.get_logger().info(
            f"Localization berhasil. map->odom baru: "
            f"x={t.x:.3f} y={t.y:.3f} z={t.z:.3f}"
        )
        response.success = True
        response.message = "map->odom berhasil diset dari deteksi AprilTag."
        return response


def main(args=None):
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