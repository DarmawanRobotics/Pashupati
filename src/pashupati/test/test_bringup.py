"""Tests of the bringup helpers and of the launch description of robot.launch.py."""

import os

from pashupati.bringup import (
    BringupError,
    expected_nodes,
    load_site_config,
    non_empty,
    package_version,
    resolve_mode,
    resolve_namespace,
    resolve_odometry,
)
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)


def test_modes_and_odometry():
    """Patrol and teach (and the old name mapping) are the only modes."""
    assert resolve_mode('patrol') == 'patrol'
    assert resolve_mode(' Teach ') == 'teach'
    assert resolve_mode('mapping') == 'teach'
    assert resolve_odometry('leg_kilo') == 'leg_kilo'
    for bad in ('', 'auto', 'map'):
        with pytest.raises(BringupError):
            resolve_mode(bad)
    with pytest.raises(BringupError):
        resolve_odometry('slam')


def test_namespace():
    """Driver namespaces are plain ROS names, slashes around them are dropped."""
    assert resolve_namespace('/l1w/') == 'l1w'
    assert resolve_namespace('fleet/l1w_2') == 'fleet/l1w_2'
    for bad in ('', '1robot', 'l1w-2', 'a//b'):
        with pytest.raises(BringupError):
            resolve_namespace(bad)


def test_site_config(tmp_path):
    """Missing file: nothing; valid file: used, unknown nodes warned; broken file: error."""
    assert load_site_config('') == ([], [])
    assert load_site_config(str(tmp_path / 'missing.yaml')) == ([], [])
    good = tmp_path / 'site.yaml'
    good.write_text(
        'path_follower_node:\n  ros__parameters:\n    target_linear_velocity: 0.3\n'
        'path_folower_node:\n  ros__parameters:\n    x: 1\n'
    )
    files, warnings = load_site_config(str(good))
    assert files == [str(good)]
    assert len(warnings) == 1 and 'path_folower_node' in warnings[0]
    for text in ('- a\n- b\n', 'node:\n  x: 1\n', 'node: [\n'):
        bad = tmp_path / 'bad.yaml'
        bad.write_text(text)
        with pytest.raises(BringupError):
            load_site_config(str(bad))


def test_example_site_config_is_valid():
    """The shipped example passes the same checks without warnings."""
    files, warnings = load_site_config(os.path.join(PACKAGE, 'config', 'site.example.yaml'))
    assert files and warnings == []


def test_non_empty_and_version():
    """Unset launch arguments never override the site file; version comes from package.xml."""
    assert non_empty({'a': '', 'b': 'x', 'c': None}) == {'b': 'x'}
    assert package_version(os.path.join(PACKAGE, 'package.xml')) != 'unknown'
    assert package_version('/nonexistent/package.xml') == 'unknown'


def load_launch_module():
    """Import launch/robot.launch.py as a module."""
    import importlib.util

    path = os.path.join(PACKAGE, 'launch', 'robot.launch.py')
    spec = importlib.util.spec_from_file_location('robot_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_context(module, **overrides):
    """Return a LaunchContext with the declared defaults plus overrides."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value
            )
    context.launch_configurations.update(overrides)
    return context


def test_launch_starts_every_node_once():
    """robot.launch.py builds every Pashupati node (sensors off: no robot needed)."""
    pytest.importorskip('launch_ros')
    from launch_ros.actions import Node
    from launch.utilities import perform_substitutions

    module = load_launch_module()
    context = launch_context(module, sensors='false', mode='teach', site_config='')
    actions = module.launch_setup(context)

    def text(value):  # a str (Jazzy) or a list of substitutions (Humble)
        return value if isinstance(value, str) else perform_substitutions(context, value)

    executables = sorted(text(a.node_executable) for a in actions if isinstance(a, Node))
    assert executables == sorted([
        'localization_node', 'lidar_sector_node', 'apriltag_node', 'path_loader_node',
        'obstacle_avoidance_node', 'path_follower_node', 'control_mux_node', 'teleop_udp_node',
        'fleet_uplink_node', 'person_detector_node', 'anomaly_detector_node',
        'health_monitor_node',
    ])


def test_launch_rejects_bad_arguments():
    """A wrong mode or namespace stops the launch with a clear error."""
    pytest.importorskip('launch_ros')
    module = load_launch_module()
    for overrides in ({'mode': 'auto'}, {'odometry': 'gmapping'}, {'robot_ns': 'l1w-2'}):
        with pytest.raises(BringupError):
            module.launch_setup(launch_context(module, sensors='false', **overrides))


def test_health_watches_every_started_node():
    """The health monitor expects every launched node plus the odometry ones."""
    names = expected_nodes(['path_follower_node', 'control_mux_node'], 'fast_lio')
    assert names == sorted(names) and len(names) == len(set(names))
    assert {'path_follower_node', 'fastlio_mapping_node', 'health_monitor_node'} <= set(names)
    assert 'legkilo' in expected_nodes([], 'leg_kilo')
