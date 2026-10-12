"""Node subclasses must not shadow rclpy.node.Node attributes (handle, context, ...)."""

import ast
from pathlib import Path

RESERVED = {
    'handle', 'context', 'executor', 'clients', 'services', 'timers', 'guards', 'publishers',
    'subscriptions', 'waitables', 'default_callback_group', 'get_name', 'get_namespace',
    'get_clock', 'get_logger', 'destroy_node',
}


def test_no_shadowed_node_attributes():
    for path in (Path(__file__).parents[1] / 'robot_odometry').rglob('*.py'):
        tree = ast.parse(path.read_text())
        for cls in (n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)):
            if not any(getattr(b, 'id', None) == 'Node' for b in cls.bases):
                continue
            for item in cls.body:
                names = []
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    names = [item.name]
                elif isinstance(item, ast.Assign):
                    names = [t.id for t in item.targets if isinstance(t, ast.Name)]
                for name in names:
                    assert name not in RESERVED, f'{path.name}: {cls.name}.{name} shadows rclpy'
