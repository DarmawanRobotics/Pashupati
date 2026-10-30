"""The composition: package launches, arguments, node names (no ROS)."""

from __future__ import annotations

from pathlib import Path

from pashupati import bringup, stack

CONFIG = Path(__file__).resolve().parents[2] / 'config'
CONTRACT = {'params_file', 'robot_params', 'use_sim_time', 'log_level'}


def options(**kwargs) -> stack.Options:
    return stack.Options(robot_params='/r.yaml', params_file='/p.yaml', **kwargs)


def by_package(includes) -> dict:
    return {i.package: i for i in includes}


def test_every_component_in_order():
    packages = [i.package for i in stack.compose(options(cc_url='https://netra'))]
    assert packages == [
        'dharma_odometry',
        'dharma_localization',
        'dharma_perception',
        'dharma_mapping',
        'dharma_navigation',
        'dharma_control',
        'dharma_health',
        'pashupati_bridge',
        'pashupati_inspection',
        'pashupati_fleet',
    ]
    light = by_package(stack.compose(options(inspection=False, people=False, health=False)))
    assert not {'pashupati_inspection', 'pashupati_fleet', 'dharma_health'} & set(light)


def test_contract_arguments_reach_every_launch():
    o = options(use_sim_time='true', log_level='debug', cc_url='https://netra')
    for include in stack.compose(o):
        assert set(include.arguments) >= CONTRACT, include.package
        assert include.arguments['params_file'] == '/p.yaml'


def test_empty_arguments_are_never_passed():
    for include in stack.compose(options()):
        assert all(str(v).strip() for v in include.arguments.values())
    assert 'video_port' not in by_package(stack.compose(options()))['pashupati_bridge'].arguments


def test_only_the_control_mux_drives_the_robot():
    includes = by_package(stack.compose(options(robot_ns='dog')))
    assert includes['dharma_control'].arguments['cmd_vel_topic'] == '/dog/cmd_vel'
    bridge = includes['pashupati_bridge'].arguments
    assert bridge['robot_ns'] == 'dog' and 'cmd_vel_topic' not in bridge
    nav = includes['dharma_navigation'].arguments
    assert 'cmd_vel_topic' not in nav  # control/nav_cmd_vel by default
    assert (nav['battery_topic'], nav['odom_topic']) == ('/dog/battery', '/dog/odom')


def test_camera_source_reaches_every_consumer():
    o = options(camera='rtsp', camera_url='rtsp://cam/live', camera_topics='/front')
    includes = by_package(stack.compose(o))
    assert includes['pashupati_bridge'].arguments['camera'] == 'rtsp'
    assert includes['pashupati_bridge'].arguments['camera_url'] == 'rtsp://cam/live'
    assert includes['pashupati_inspection'].arguments['camera_image'] == '/front/image_raw'
    assert includes['dharma_perception'].arguments['camera_info'] == '/front/camera_info'
    assert '/rtsp_camera' in stack.node_names(o)
    camera_only = options(teleop=False, camera='rtsp', camera_url='rtsp://c')
    assert '/teleop_udp_node' not in stack.node_names(camera_only)
    assert 'pashupati_bridge' in by_package(stack.compose(camera_only))


def test_teach_maps_the_site():
    o = options(site='lobby', sites_dir='/s', start_mapping=True)
    includes = by_package(stack.compose(o))
    assert includes['dharma_odometry'].arguments['save_map'] == 'true'
    assert includes['dharma_mapping'].arguments['start_mapping'] == 'true'
    assert 'start_mapping' not in by_package(stack.compose(options()))['dharma_mapping'].arguments


def test_node_names():
    names = stack.node_names(options(cc_url='https://netra'))
    for node in ('/teleop_udp_node', '/fleet_uplink_node', '/anomaly_detector_node',
                 '/person_detector_node', '/control_mux_node', '/docking_node'):  # fmt: skip
        assert node in names
    sim = stack.node_names(options(odometry='external', fiducials=False, inspection=False,
                                   people=False), sim=True)  # fmt: skip
    assert '/sim_robot_node' in sim and '/anomaly_detector_node' not in sim
    assert '/fleet_uplink_node' not in sim


def test_shipped_files_only_configure_known_nodes():
    known = stack.known_sections()
    for name in ('stack.yaml', 'sim.yaml', 'site.example.yaml'):
        data = bringup.load_params(str(CONFIG / name))
        assert bringup.unknown_sections(data, known) == [], name
