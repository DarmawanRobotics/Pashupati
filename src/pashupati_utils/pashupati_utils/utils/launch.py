"""The launch contract every Pashupati package launch shares with the Dharma ones."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# name -> (default, description): params_file, robot_params, use_sim_time and log_level
COMMON = {
    'params_file': ('', 'parameters on top of the package defaults'),
    'robot_params': ('', 'robot profile (robot.* parameters)'),
    'use_sim_time': ('', 'true | false; empty: from the parameter files'),
    'log_level': ('info', 'debug, info, warn, error'),
}


def declare(extra: dict | None = None) -> list:
    """Return the DeclareLaunchArgument actions of COMMON plus extra {name: (default, doc)}."""
    items = {**COMMON, **(extra or {})}
    return [
        DeclareLaunchArgument(n, default_value=d, description=t) for n, (d, t) in items.items()
    ]


def performer(context):
    """Return arg(name) -> the evaluated, stripped launch argument."""
    return lambda name: LaunchConfiguration(name).perform(context).strip()


def parameter_files(arg, package: str, defaults: str) -> list[str]:
    """Return [package defaults, robot_params?, params_file?]; a set file must exist."""
    files = [os.path.join(get_package_share_directory(package), 'config', defaults)]
    for name in ('robot_params', 'params_file'):
        path = os.path.expanduser(arg(name))
        if path and path not in files:
            if not os.path.isfile(path):
                raise FileNotFoundError(f'{name}:={path} does not exist')
            files.append(path)
    return files


def overrides(arg, **values) -> dict:
    """Return the set launch values: use_sim_time when given, then the non-empty values."""
    out = {}
    sim_time = arg('use_sim_time').lower()
    if sim_time:
        out['use_sim_time'] = sim_time in ('true', '1', 'yes')
    out.update({k: v for k, v in values.items() if v not in ('', None)})
    return out


def ros_arguments(arg) -> list[str]:
    """Return the node's --log-level arguments."""
    return ['--log-level', arg('log_level') or 'info']


def truthy(value) -> bool:
    """Interpret a launch argument as a bool."""
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')
