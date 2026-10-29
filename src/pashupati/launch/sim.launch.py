r"""
The whole Pashupati stack on the simulated robot of dharma_sim: no robot, no hardware.

    ros2 launch pashupati sim.launch.py rviz:=true
    ros2 launch pashupati sim.launch.py scenario:=person_crossing
    ros2 run pashupati_fleet fake_netra &   # a stand-in NETRA
    ros2 launch pashupati sim.launch.py cc_url:=http://127.0.0.1:8765
    ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray \\
        "{poses: [{position: {x: 2.0, y: -3.0}}]}"

The site (mall_loop under /tmp/pashupati_sim/sites) is drawn from the world when it has no map
yet. config/sim.yaml adapts the robot to the simulator (PointCloud2 input, no camera, the
simulated dock); inspection and people are off unless asked for (inspection:=true with an
Ollama, or a stand-in, at ollama_url).
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from pashupati import bringup, stack
from pashupati import launch_utils as lu

SIM_ARGUMENTS = [n for n in lu.ARGUMENTS if n not in ('robot_driver', 'odometry', 'webrtc',
                                                       'mediamtx', 'params_file')]  # fmt: skip
SIM_DEFAULTS = {
    'site': 'mall_loop',
    'sites_dir': '/tmp/pashupati_sim/sites',
    'fiducials': 'false',
    'inspection': 'false',
    'people': 'false',
    'robot_id': 'sim-01',
}


def launch_setup(context):
    """Start the simulated robot, then the stack against it."""
    arg = lu.performer(context)
    options, _, notes = lu.resolve(arg, sim=True)
    sim = stack.sim_include(options, arg('world'), arg('scenario'), bringup.truthy(arg('rviz')))
    return [*notes, *(lu.include_action(i) for i in [sim, *stack.compose(options)])]


def generate_launch_description():
    """Declare the arguments of the robot launch (no driver, odometry) and of the simulator."""
    return LaunchDescription(
        [
            *lu.declare(SIM_ARGUMENTS, SIM_DEFAULTS, env=False),
            DeclareLaunchArgument(
                'params_file', default_value='', description='parameters on top of sim.yaml'
            ),
            DeclareLaunchArgument(
                'world', default_value='', description='world; empty: mall_loop'
            ),
            DeclareLaunchArgument(
                'scenario', default_value='', description='scenario; empty: none'
            ),
            DeclareLaunchArgument('rviz', default_value='false', description='RViz (dharma_rviz)'),
            OpaqueFunction(function=launch_setup),
        ]
    )
