r"""
The NETRA uplink (fleet_uplink_node).

    ros2 launch pashupati_fleet fleet.launch.py robot_ns:=robot \\
        server_url:=https://netra.example.com token:=<robot token>

Same contract as the Dharma package launches (params_file, robot_params, use_sim_time,
log_level); non-empty server_url, token and robot_id win over the parameter files.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from pashupati_utils.utils import launch as pl

import dharma_utils.ros.launch as dl

PACKAGE = 'pashupati_fleet'
NODE = 'fleet_uplink_node'
EXTRA = {
    'robot_ns': ('robot', 'robot driver namespace (/<ns>/battery, /<ns>/diagnostics)'),
    'server_url': ('', 'NETRA url; empty: from the parameter files'),
    'token': ('', 'robot token; empty: from the parameter files'),
    'robot_id': ('', 'fleet id; empty: from the parameter files'),
}


def nodes(context):
    arg = pl.performer(context)
    files = pl.parameter_files(arg, PACKAGE, 'fleet.yaml')
    ns = arg('robot_ns').strip('/') or 'robot'
    values = pl.overrides(
        arg, server_url=arg('server_url'), token=arg('token'), robot_id=arg('robot_id')
    )
    return [
        dl.node(
            package=PACKAGE,
            executable=NODE,
            name=NODE,
            output='screen',
            parameters=[*files, *dl.node_overrides(NODE, values)],
            remappings=[('battery', f'/{ns}/battery'), ('diagnostics', f'/{ns}/diagnostics')],
            ros_arguments=pl.ros_arguments(arg),
            respawn=True,
            respawn_delay=3.0,
        )
    ]


def generate_launch_description():
    return LaunchDescription([*pl.declare(EXTRA), OpaqueFunction(function=nodes)])
