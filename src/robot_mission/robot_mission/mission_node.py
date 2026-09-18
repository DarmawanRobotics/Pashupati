#!/usr/bin/env python3
import py_trees

import rclpy
from rclpy.node import Node

from robot_mission.tree_loader import load_tree


class MissionNode(Node):
    """Loads a behavior tree from an XML file and ticks it. The tree's structure (retry counts,
    infinite patrol looping, etc.) lives entirely in that XML file, not in this node's code --
    swap trees/patrol.xml for trees/single_run.xml (or your own) without touching this file."""

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

        tick_rate = float(self.get_parameter('tick_rate').value)
        self.create_timer(1.0 / tick_rate, self._tree.tick)


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
