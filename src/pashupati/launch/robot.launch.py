"""
The whole Pashupati robot in one launch (production: pashupati.service, scripts/run_robot.sh).

    ros2 launch pashupati robot.launch.py                                 # patrol, last site
    ros2 launch pashupati robot.launch.py robot_driver:=my_dog_bringup site:=lobby
    ros2 launch pashupati robot.launch.py site:=lobby mode:=teach        # map the site
    ros2 launch pashupati robot.launch.py camera:=rtsp camera_url:=rtsp://192.168.1.120:8554/test

The robot driver comes from the robot's own package (robot_driver:=, src/xtras/README.md).
Every package launch gets robot_params:=config/robot.yaml and one parameter file merged from
config/stack.yaml and the site file (params_file, /etc/pashupati/site.yaml; later wins), with
the started nodes added to the health monitor's nodes.expected. Most arguments default to the
environment of pashupati.service (/etc/pashupati/pashupati.env). The composition is in
pashupati/stack.py.
"""

from __future__ import annotations

from dataclasses import replace

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction

from pashupati import launch_utils as lu
from pashupati import stack, video, webrtc


def launch_setup(context):
    """Validate the arguments, then the robot driver, MediaMTX and the whole stack."""
    arg = lu.performer(context)
    options, data, notes = lu.resolve(arg)
    driver = lu.driver_action(arg('robot_driver'), options)
    mediamtx, note = webrtc.video_server(arg('webrtc'), arg('mediamtx'), options, data)
    if mediamtx is not None:
        options = replace(options, video_port=str(video.WEBRTC_PORT))
    actions = [*notes, LogInfo(msg=note)]
    actions += [a for a in (driver, mediamtx) if a is not None]
    return actions + [lu.include_action(i) for i in stack.compose(options)]


def generate_launch_description():
    """Declare the arguments (most also from the environment)."""
    return LaunchDescription([*lu.declare(lu.ARGUMENTS), OpaqueFunction(function=launch_setup)])
