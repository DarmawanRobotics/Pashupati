#!/usr/bin/env python3
"""
The whole Pashupati stack on the simulated quadruped of dharma_sim, no robot needed.

    ros2 launch pashupati sim.launch.py                    # localizes itself, waits for start_nav
    ros2 launch pashupati sim.launch.py autostart:=true    # patrols right away
    ros2 launch pashupati sim.launch.py rviz:=true         # with the RViz panels

Drop an obstacle on the route:
    ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray \
        "{poses: [{position: {x: 2.0, y: -3.0}}]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    """Start the simulated robot and the robot stack in simulation mode."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    share = get_package_share_directory('pashupati')
    sim = get_package_share_directory('dharma_sim')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'sim.launch.py')),
            launch_arguments={
                'world': arg('world'),
                'robot_ns': arg('robot_ns'),
                'autostart': arg('autostart'),
                'rviz': arg('rviz'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'robot.launch.py')),
            launch_arguments={
                'robot_id': 'sim-01',
                'robot_ns': arg('robot_ns'),
                'mode': 'patrol',
                'odometry': 'external',
                'route': arg('route'),
                'tags': '',
                'cc_url': arg('cc_url'),
                'cc_token': arg('cc_token'),
                'site_config': os.path.join(share, 'config', 'sim_site.yaml'),
                'robot_bringup': '',
                'people': 'false',
                'inspection': 'false',
                'health': 'true',
                'webrtc': 'false',  # no simulated camera to stream
            }.items(),
        ),
    ]


def generate_launch_description():
    """Declare the simulation arguments."""
    sim = get_package_share_directory('dharma_sim')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world', default_value=os.path.join(sim, 'config', 'worlds', 'mall_loop.yaml')
            ),
            DeclareLaunchArgument(
                'route', default_value=os.path.join(sim, 'config', 'routes', 'mall_loop.csv')
            ),
            DeclareLaunchArgument('robot_ns', default_value='robot'),
            DeclareLaunchArgument(
                'autostart', default_value='false', description='start the patrol when localized'
            ),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument(
                'cc_url', default_value='', description='NETRA url: the sim reports like a robot'
            ),
            DeclareLaunchArgument('cc_token', default_value=''),
            OpaqueFunction(function=launch_setup),
        ]
    )
