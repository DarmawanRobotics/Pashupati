"""
The handheld's UDP gateway, and optionally an RTSP camera as the robot's camera.

    ros2 launch pashupati_bridge bridge.launch.py robot_ns:=robot
    ros2 launch pashupati_bridge bridge.launch.py camera:=rtsp camera_url:=rtsp://10.0.0.5/live

Same contract as the Dharma package launches (params_file, robot_params, use_sim_time,
log_level). The gateway sends the stick to control/remote/<remote_source>/cmd_vel: only the
control mux publishes the robot's cmd_vel. The robot driver's battery, diagnostics and the
mux's cmd_vel (shown on the handheld) are read from /<robot_ns>/.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from pashupati_utils.utils import launch as pl

import dharma_utils.ros.launch as dl

PACKAGE = 'pashupati_bridge'
CAMERA_SOURCES = ('driver', 'rtsp')
EXTRA = {
    'robot_ns': ('robot', 'robot driver namespace (/<ns>/battery, diagnostics, cmd_vel)'),
    'teleop': ('true', 'the handheld gateway (teleop_udp_node)'),
    'video_port': ('', 'MediaMTX WHEP port when the bringup runs it; empty: JPEG only'),
    'camera': ('driver', 'driver: the robot publishes the camera; rtsp: rtsp_camera_node does'),
    'camera_url': ('', 'rtsp:// url of the camera (camera:=rtsp)'),
    'camera_topics': ('/camera/camera/color', 'where the camera is published'),
}


def nodes(context):
    arg = pl.performer(context)
    files = pl.parameter_files(arg, PACKAGE, 'bridge.yaml')
    ns = arg('robot_ns').strip('/') or 'robot'
    camera = arg('camera').lower() or 'driver'
    if camera not in CAMERA_SOURCES:
        raise ValueError(f'camera:={camera!r}, use one of {CAMERA_SOURCES}')
    out = []
    if pl.truthy(arg('teleop')):
        values = pl.overrides(
            arg,
            robot_command_prefix=f'/{ns}/',
            video_port=int(arg('video_port')) if arg('video_port') else None,
            camera_topic=f'{arg("camera_topics")}/image_raw/compressed',
        )
        remaps = [(n, f'/{ns}/{n}') for n in ('battery', 'diagnostics', 'cmd_vel')]
        out.append(node('teleop_udp_node', 'teleop_udp_node', files, values, remaps, arg))
    if camera == 'rtsp':
        topics = arg('camera_topics').rstrip('/')
        remaps = [(n, f'{topics}/{n}') for n in ('image_raw', 'image_raw/compressed',
                                                 'camera_info')]  # fmt: skip
        values = pl.overrides(arg, url=arg('camera_url'))
        out.append(node('rtsp_camera_node', 'rtsp_camera', files, values, remaps, arg))
    return out


def node(executable: str, name: str, files: list, values: dict, remaps: list, arg):
    return dl.node(
        package=PACKAGE,
        executable=executable,
        name=name,
        output='screen',
        parameters=[*files, *dl.node_overrides(name, values)],
        remappings=remaps,
        ros_arguments=pl.ros_arguments(arg),
        respawn=True,
        respawn_delay=3.0,
    )


def generate_launch_description():
    return LaunchDescription([*pl.declare(EXTRA), OpaqueFunction(function=nodes)])
