#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped

from tf2_ros import StaticTransformBroadcaster


class LocalizationNode(Node):
    """Stitches FAST-LIO's camera_init/body TF into a single map->odom->camera_init->body->base_link tree."""
    def __init__(self):
        """Broadcast the static identity links joining FAST-LIO's frames to map and base_link."""
        super().__init__('localization_node')
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_links()

    def identity_transform(self, parent_frame: str, child_frame: str) -> TransformStamped:
        """Build a zero-translation, zero-rotation TransformStamped between two frames."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.rotation.w = 1.0
        return t

    def broadcast_static_links(self):
        """Publish map->odom, odom->camera_init, and body->base_link, all as fixed identity links."""
        self._static_broadcaster.sendTransform([
            self.identity_transform('map', 'odom'),
            self.identity_transform('odom', 'camera_init'),
            self.identity_transform('body', 'base_link'),
        ])

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