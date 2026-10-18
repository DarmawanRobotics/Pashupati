#!/usr/bin/env python3
"""
The whole Pashupati stack on a simulated quadruped, no robot needed.

    ros2 launch pashupati_sim sim.launch.py                  # localizes, waits for start_nav
    ros2 launch pashupati_sim sim.launch.py autostart:=true  # patrols right away
    ros2 launch pashupati_sim sim.launch.py rviz:=true       # with the RViz panels

Drop an obstacle on the route:
    ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray \
        "{poses: [{position: {x: 2.0, y: -3.0}}]}"
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """Start the simulator, the robot stack in simulation mode and optionally RViz."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    share = get_package_share_directory('pashupati_sim')
    actions = [
        Node(
            package='pashupati_sim',
            executable='sim_robot_node',
            name='sim_robot_node',
            output='screen',
            parameters=[
                {
                    'world_file': arg('world'),
                    'robot_ns': arg('robot_ns'),
                    'autostart_nav': arg('autostart').lower() == 'true',
                }
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('pashupati'), 'launch', 'robot.launch.py')
            ),
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
    if arg('rviz').lower() == 'true':
        try:
            config = os.path.join(
                get_package_share_directory('pashupati_rviz'), 'rviz', 'robot.rviz'
            )
        except PackageNotFoundError:
            config = ''
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', config] if config else [],
                remappings=[
                    ('battery', f'/{arg("robot_ns")}/battery'),
                    ('cmd_vel', f'/{arg("robot_ns")}/cmd_vel'),
                ],
                output='log',
            )
        )
    return actions


def generate_launch_description():
    """Declare the simulation arguments."""
    share = get_package_share_directory('pashupati_sim')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world', default_value=os.path.join(share, 'config', 'worlds', 'mall_loop.yaml')
            ),
            DeclareLaunchArgument(
                'route', default_value=os.path.join(share, 'config', 'routes', 'mall_loop.csv')
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
