#!/usr/bin/env python3
import time

from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robot_perception.person_projection import feet_pixel, ground_point, pixel_ray
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import Buffer, TransformException, TransformListener
import tf_transformations as tft

try:
    import cv2
except ImportError:
    cv2 = None


class PersonDetectorNode(Node):
    """Detects people in the camera and places them on the floor in the map frame."""

    def __init__(self):
        """Declare params, load the model and wire topics."""
        super().__init__('person_detector_node')
        defaults = {
            'image_topic': '/camera/camera/color/image_raw/compressed',
            'compressed': True,
            'camera_info_topic': '/camera/camera/color/camera_info',
            'model_path': 'yolo11n.engine',
            'confidence': 0.4,
            'image_size': 640,
            'rate_hz': 3.0,
            'floor_z': -0.35,
            'max_range': 12.0,
            'map_frame': 'map',
            'base_frame': 'base_link',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._floor_z, self._max_range = float(p('floor_z')), float(p('max_range'))
        self._map_frame, self._base_frame = p('map_frame'), p('base_frame')
        self._period = 1.0 / max(float(p('rate_hz')), 0.1)
        self._last = 0.0
        self._k = None
        self._t_base_cam = {}

        self._detector = None
        try:
            from robot_perception.detector import PersonDetector

            self._detector = PersonDetector(
                p('model_path'), float(p('confidence')), int(p('image_size'))
            )
        except Exception as error:  # noqa: B902
            self.get_logger().error(f'person detector disabled: {error}')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._people_pub = self.create_publisher(PoseArray, 'perception/people', 10)
        self.create_subscription(CameraInfo, p('camera_info_topic'), self.on_info, 10)
        kind = CompressedImage if p('compressed') else Image
        self.create_subscription(kind, p('image_topic'), self.on_image, qos_profile_sensor_data)

    def on_info(self, msg: CameraInfo):
        """Keep the camera intrinsics."""
        self._k = np.array(msg.k, dtype=float).reshape(3, 3)

    def decode(self, msg) -> np.ndarray:
        """Return a BGR image from a compressed or rgb8/bgr8 message, or None."""
        if cv2 is None:
            return None
        if isinstance(msg, CompressedImage):
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if msg.encoding not in ('rgb8', 'bgr8'):
            return None
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)
        img = img[:, : msg.width]
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img

    def camera_extrinsic(self, frame_id: str):
        """Return the cached base->camera transform as a 4x4 matrix, or None."""
        if frame_id not in self._t_base_cam:
            try:
                t = self._tf_buffer.lookup_transform(self._base_frame, frame_id, Time())
            except TransformException:
                return None
            self._t_base_cam[frame_id] = to_matrix(t)
        return self._t_base_cam[frame_id]

    def on_image(self, msg):
        """Detect people at rate_hz and publish their floor positions in the map frame."""
        now = time.monotonic()
        if self._detector is None or self._k is None or now - self._last < self._period:
            return
        self._last = now
        t_base_cam = self.camera_extrinsic(msg.header.frame_id)
        image = self.decode(msg)
        if t_base_cam is None or image is None:
            return
        try:
            t_map_base = to_matrix(
                self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
            )
        except TransformException:
            return

        people = PoseArray()
        people.header.stamp = msg.header.stamp
        people.header.frame_id = self._map_frame
        for box in self._detector.detect(image):
            point = ground_point(
                pixel_ray(*feet_pixel(box), self._k), t_base_cam, self._floor_z, self._max_range
            )
            if point is None:
                continue
            x, y, _ = (t_map_base @ np.append(point, 1.0))[:3]
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.w = box[4]  # detection score, poses carry no heading
            people.poses.append(pose)
        self._people_pub.publish(people)


def to_matrix(t) -> np.ndarray:
    """Return a 4x4 matrix from a TransformStamped."""
    q, v = t.transform.rotation, t.transform.translation
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    m[:3, 3] = [v.x, v.y, v.z]
    return m


def main(args=None):
    """Spin the person detector node."""
    rclpy.init(args=args)
    node = PersonDetectorNode()
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
