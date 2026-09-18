#!/usr/bin/env python3
import py_trees

import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

from robot_interfaces.msg import MissionStatus

from robot_mission.tree_loader import load_tree


class MissionNode(Node):
    """Loads a behavior tree from an XML file, ticks it, publishes its live state on
    mission/status, and exposes mission/set_active to pause/resume it -- pausing also
    cancels any in-flight navigate_route goal (via NavigateRouteAction.terminate), freeing
    the action server for manual control instead of the two fighting over it."""

    def __init__(self):
        """Declare params, load and set up the tree, and start ticking it on a timer."""
        super().__init__('mission_node')
        self.declare_parameter('tick_rate', 5.0)
        self.declare_parameter('tree_file', '')

        tree_file = self.get_parameter('tree_file').value
        if not tree_file:
            raise RuntimeError('tree_file parameter is required, e.g. .../trees/patrol.xml')

        root = load_tree(tree_file, self)
        self.get_logger().info('Loaded tree:\n' + py_trees.display.ascii_tree(root))

        self._tree = py_trees.trees.BehaviourTree(root)
        try:
            self._tree.setup(timeout=15)
        except Exception as error:
            self.get_logger().error(f'tree setup failed: {error}')

        self._active = True
        self._status_pub = self.create_publisher(MissionStatus, 'mission/status', 10)
        self.create_service(SetBool, 'mission/set_active', self.set_active_callback)

        tick_rate = float(self.get_parameter('tick_rate').value)
        self.create_timer(1.0 / tick_rate, self.tick_and_publish)

    def set_active_callback(self, request, response):
        """Pause or resume ticking. Pausing stops the tree (INVALID), which cancels any
        in-flight navigate_route goal through the running behaviour's own terminate()."""
        if not request.data and self._active:
            self._tree.root.stop(py_trees.common.Status.INVALID)
        self._active = request.data
        response.success = True
        response.message = 'mission active' if self._active else 'mission paused'
        return response

    def tick_and_publish(self):
        """Tick the tree (unless paused) and publish its current root status and active leaf."""
        if self._active:
            self._tree.tick()

        tip = self._tree.root.tip()
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.active = self._active
        msg.root_status = self._tree.root.status.name if self._tree.root.status else 'INVALID'
        msg.active_behavior = tip.name if tip is not None else 'none'
        self._status_pub.publish(msg)


def main(args=None):
    """Spin the mission node."""
    rclpy.init(args=args)
    node = MissionNode()
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