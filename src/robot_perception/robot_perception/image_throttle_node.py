#!/usr/bin/env python3
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class ImageThrottleNode(Node):
    """
    Pass a camera stream on at a lower rate, as serialized bytes (no image is deserialized).

    The AprilTag detector processes every frame it receives, but localization only needs a tag
    pose a few times per second. Every camera info is passed on, so each forwarded image finds
    the info with its stamp (image_transport pairs them by exact time).
    """

    def __init__(self):
        """Declare the rate and relay in/* to out/*."""
        super().__init__('image_throttle_node')
        self.declare_parameter('rate_hz', 5.0)
        rate = float(self.get_parameter('rate_hz').value)
        self._period = 1.0 / rate if rate > 0.0 else 0.0
        self._last = -1e9
        qos = qos_profile_sensor_data
        self._image_pub = self.create_publisher(Image, 'out/image', qos)
        self._info_pub = self.create_publisher(CameraInfo, 'out/camera_info', qos)
        self.create_subscription(Image, 'in/image', self.on_image, qos, raw=True)
        self.create_subscription(CameraInfo, 'in/camera_info', self.on_info, qos, raw=True)

    def on_image(self, data: bytes):
        """Forward an image when the period has passed since the last one."""
        now = time.monotonic()
        if now - self._last < self._period:
            return
        self._last = now
        self._image_pub.publish(data)

    def on_info(self, data: bytes):
        """Forward every camera info (small)."""
        self._info_pub.publish(data)


def main(args=None):
    """Spin the image throttle node."""
    rclpy.init(args=args)
    node = ImageThrottleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
