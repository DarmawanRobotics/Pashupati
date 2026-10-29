"""
The inspection nodes: the VLM anomaly detector and the person detector, each optional.

    ros2 launch pashupati_inspection inspection.launch.py
    ros2 launch pashupati_inspection inspection.launch.py people:=false
    ros2 launch pashupati_inspection inspection.launch.py camera_image:=/front/image_raw

Same contract as the Dharma package launches (params_file, robot_params, use_sim_time,
log_level); the pashupati bringup starts it with its merged parameter file.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from pashupati_utils.utils import launch as pl

import dharma_utils.ros.launch as dl

PACKAGE = 'pashupati_inspection'
EXTRA = {
    'anomaly': ('true', 'VLM inspection (anomaly_detector_node)'),
    'people': ('true', 'person detection (person_detector_node)'),
    'camera_image': ('', 'raw camera image topic; empty: from the parameter files'),
    'camera_info': ('', 'camera info topic; empty: from the parameter files'),
    'ollama_url': ('', 'Ollama server; empty: from the parameter files'),
}


def nodes(context):
    arg = pl.performer(context)
    files = pl.parameter_files(arg, PACKAGE, 'inspection.yaml')
    image = arg('camera_image')
    out = []
    if pl.truthy(arg('anomaly')):
        values = pl.overrides(arg, image_topic=image, ollama_url=arg('ollama_url'))
        out.append(node('anomaly_detector_node', files, values, arg))
    if pl.truthy(arg('people')):
        values = pl.overrides(arg, image_topic=image, camera_info_topic=arg('camera_info'))
        out.append(node('person_detector_node', files, values, arg))
    return out


def node(name: str, files: list, values: dict, arg):
    return dl.node(
        package=PACKAGE,
        executable=name,
        name=name,
        output='screen',
        parameters=[*files, *dl.node_overrides(name, values)],
        ros_arguments=pl.ros_arguments(arg),
        respawn=True,
        respawn_delay=3.0,
    )


def generate_launch_description():
    return LaunchDescription([*pl.declare(EXTRA), OpaqueFunction(function=nodes)])
