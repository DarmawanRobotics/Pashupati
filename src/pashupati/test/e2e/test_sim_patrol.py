"""
End to end: the whole Pashupati robot on dharma_sim, the way an operator uses it.

Starts `ros2 launch pashupati sim.launch.py` with the VLM inspection (a stand-in Ollama) and the
NETRA uplink (pashupati_fleet's fake_netra), publishes a camera, then: the robot localizes, a
route with three stop points is taught (driven with planner/go_to while route/start records),
the taught route is patrolled to its end with the inspection hook answered at every stop, the
handheld takes over through the control mux in the middle and gives it back, and NETRA receives
the telemetry, the stop events, the findings and the taught route. Finally the launch must stop
cleanly with no node died. Needs the workspace installed (colcon test); PASHUPATI_SKIP_SIM=1
skips it.
"""

from __future__ import annotations

import math
import os
import warnings

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dharma_interfaces.msg import (
    ControlStatus,
    LocalizationStatus,
    NavigationStatus,
    PlannerStatus,
    StopPointEvent,
)
from dharma_interfaces.srv import GoTo, LoadPath, MarkStopPoint, PlanPath
from dharma_utils.ros import EVENTS, LATCHED, SENSOR, compat
from dharma_utils.testing import Collector, Spinner, wait_for

pytestmark = pytest.mark.skipif(os.environ.get('PASHUPATI_SKIP_SIM') == '1', reason='skipped')
pytest.importorskip('cv2')  # the anomaly detector encodes frames
pytest.importorskip('websocket')  # the NETRA uplink
from pashupati_fleet.tools.fake_netra import FakeNetra  # noqa: E402
from sim_rig import Handheld, Launch, OllamaHandler, problems, serve  # noqa: E402

DOMAIN = 60 + os.getpid() % 30
SITES = '/tmp/pashupati_sim/sites'
START = (-3.5, -3.0, 0.0)  # the bottom corridor of mall_loop (the robot starts at -4.5)
PASHUPATI_NODES = tuple(  # how their lines start in the launch log
    f'{prefix}{name}-'
    for name in (
        'teleop_udp_node',
        'video_stream_node',
        'rtsp_camera',
        'anomaly_detector_node',
        'person_detector_node',
        'fleet_uplink_node',
    )
    for prefix in ('[', '[ERROR] [')
)
STOPS = [(-2.0, -3.0, 0.0), (0.5, -3.0, 0.0), (3.0, -3.0, 0.0)]
NAVIGATION_NODES = tuple(
    f'[{name}-'
    for name in ('path_follower_node', 'path_loader_node', 'control_mux_node', 'planner_node')
)


class Operator:
    """The test's ROS side: a camera, the services an operator calls, what the robot reports."""

    def __init__(self, node):
        self.node = node
        self.stops = Collector(node, StopPointEvent, 'navigation/stop_point', EVENTS)
        self.nav = Collector(node, NavigationStatus, 'navigation/status', 10)
        self.planner = Collector(node, PlannerStatus, 'planner/status', LATCHED)
        self.localization = Collector(node, LocalizationStatus, 'localization/status', LATCHED)
        self.control = Collector(node, ControlStatus, 'control/status', LATCHED)
        self.saved = Collector(node, String, 'route/saved', EVENTS)
        self.camera = node.create_publisher(Image, '/camera/camera/color/image_raw', SENSOR)
        node.create_timer(0.2, self.publish_frame)
        self.clients = {
            name: node.create_client(kind, name)
            for name, kind in (
                ('route/start', Trigger),
                ('route/stop', Trigger),
                ('route/mark_stop_point', MarkStopPoint),
                ('planner/go_to', GoTo),
                ('navigation/load_path', LoadPath),
                ('navigation/start', Trigger),
                ('planner/plan', PlanPath),
            )
        }

    def publish_frame(self):
        """A grey camera frame (the stand-in VLM answers trash on any)."""
        msg = Image(height=48, width=64, encoding='bgr8', step=64 * 3)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.data = np.full((48, 64, 3), 128, np.uint8).tobytes()
        self.camera.publish(msg)

    def call(self, spinner, name: str, request=None, timeout: float = 20.0):
        """Call a service of the stack."""
        client = self.clients[name]
        return spinner.call(client, request or client.srv_type.Request(), timeout)

    def planner_ready(self, spinner, x: float, y: float) -> bool:
        """Return True once the planner has the map and the robot pose (a plan to x, y)."""
        request = PlanPath.Request(x=x, y=y, yaw=0.0)
        return wait_for(lambda: self.call(spinner, 'planner/plan', request).success, 60.0, 1.0)

    def go_to(self, spinner, x: float, y: float, yaw: float) -> bool:
        """Drive to a pose with the planner; True when it arrived."""
        before = len(self.planner.messages)
        request = GoTo.Request(x=x, y=y, yaw=yaw)
        replies = []

        def answered() -> bool:
            # the planner waits for the pose, the follower's navigation/start does not: on a
            # busy machine its TF can still lag, then ask again
            replies.append(self.call(spinner, 'planner/go_to', request))
            return replies[-1].success or 'pose unknown' not in replies[-1].message

        wait_for(answered, 15.0, 0.5)
        assert replies[-1].success, replies[-1].message

        def arrived() -> bool:
            # this goal's ARRIVED: the previous goal's can still come in late on a busy machine
            return any(
                m.state == PlannerStatus.ARRIVED and math.hypot(m.goal_x - x, m.goal_y - y) < 0.01
                for m in self.planner.messages[before:]
            )

        return wait_for(arrived, 120.0)


@pytest.fixture(scope='module')
def world():
    """The simulation, its stand-in VLM and NETRA, the operator; checks the shutdown."""
    ollama = serve(OllamaHandler)
    netra = FakeNetra().start()
    launch = Launch(DOMAIN, [
        'inspection:=true',
        f'ollama_url:=http://127.0.0.1:{ollama.server_port}',
        f'cc_url:={netra.url}',
    ])  # fmt: skip
    os.environ['ROS_DOMAIN_ID'] = str(DOMAIN)  # this process talks to the launched one
    compat.init()
    node = rclpy.create_node('pashupati_e2e')
    operator = Operator(node)
    operator.launch_log = launch.log.name
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = Spinner(executor)
    try:
        yield operator, spinner, netra
    finally:
        spinner.stop()
        node.destroy_node()
        compat.try_shutdown()
        clean, log = launch.stop()
        netra.close()
        ollama.shutdown()
        ollama.server_close()
        tail = '\n'.join(log.splitlines()[-30:])
        assert clean, f'ros2 launch did not stop within 45 s:\n{tail}\nlog: {launch.log.name}'
        running, stopping = problems(log)
        assert not running, '\n'.join(running) + f'\nlog: {launch.log.name}'
        ours = [line for line in stopping if line.startswith(PASHUPATI_NODES)]
        assert not ours, '\n'.join(ours) + f'\nlog: {launch.log.name}'
        if stopping:  # a Dharma node that failed while stopping: reported, not ours to fix here
            warnings.warn('while stopping: ' + ' | '.join(stopping), stacklevel=1)


def test_the_robot_localizes(world):
    operator, _, netra = world
    assert wait_for(
        lambda: (
            operator.localization.last is not None
            and operator.localization.last.state == LocalizationStatus.LOCALIZED
        ),
        90.0,
    ), 'not localized'
    connected = wait_for(lambda: netra.of_type('hello'), 30.0)
    assert connected, 'NETRA never connected\n' + launch_lines(operator, ('[fleet_uplink_node-',))


def test_a_route_with_three_stop_points_is_taught(world):
    operator, spinner, netra = world
    assert operator.planner_ready(spinner, *START[:2]), 'the planner has no map or pose'
    assert operator.go_to(spinner, *START), 'did not reach the start of the route'
    assert operator.call(spinner, 'route/start').success
    for x, y, yaw in STOPS:
        assert operator.go_to(spinner, x, y, yaw), f'did not reach ({x}, {y})'
        request = MarkStopPoint.Request(dwell_sec=2.0)
        replies = []

        def marked(replies=replies, request=request) -> bool:
            replies.append(operator.call(spinner, 'route/mark_stop_point', request))
            return replies[-1].success or 'not localized' not in replies[-1].message

        wait_for(marked, 15.0, 0.5)
        assert replies[-1].success, replies[-1].message
    reply = operator.call(spinner, 'route/stop')
    assert reply.success, reply.message
    assert operator.saved.wait(1, 10.0), 'route/saved never came'
    with open(operator.saved.last.data, encoding='utf-8') as f:
        rows = [line.split(',') for line in f.read().splitlines()[1:] if line.strip()]
    assert sum(float(r[3]) > 0.0 for r in rows) == 3, rows
    # the taught route goes to NETRA's route database
    assert wait_for(lambda: any(p == '/api/v1/robot/routes' for p, _ in netra.uploads), 30.0)


def patrol_report(operator, events) -> str:
    """The stop point events and the navigation lines of the launch log, for a failed patrol."""
    seen = [(e.event, e.index, round(e.x, 2), round(e.y, 2), e.message) for e in events]
    return f'{seen}\n' + launch_lines(operator, NAVIGATION_NODES)


def launch_lines(operator, prefixes: tuple, last: int = 60) -> str:
    """Return the last lines of the launch log from the nodes with these prefixes."""
    with open(operator.launch_log, encoding='utf-8', errors='replace') as f:
        lines = [ln for ln in f.read().splitlines() if ln.startswith(prefixes)]
    return '\n'.join(lines[-last:])


def drive_with_the_handheld(operator, handheld: Handheld):
    """REMOTE from the handheld: the follower pauses, the stick moves the robot, back to AUTO."""
    assert handheld.hello(), 'the handheld was not welcomed'
    ack = handheld.command('set_mode', mode='remote')
    assert ack and ack['ok'], ack
    assert wait_for(lambda: operator.control.last.mode == ControlStatus.MODE_REMOTE, 5.0)
    assert wait_for(lambda: 'remote' in list(operator.nav.last.pause_reasons), 5.0)
    handheld.stick(0.0, 0.5)  # neutral first: the mux arms the source
    handheld.stick(0.2, 1.5)
    states = handheld.states()
    assert any(s['mode'] == 'remote' and abs(s['cmd'][0]) > 0.05 for s in states), states[-3:]
    ack = handheld.command('set_mode', mode='auto')
    assert ack and ack['ok'], ack
    assert wait_for(lambda: operator.control.last.mode == ControlStatus.MODE_AUTO, 5.0)


def test_the_taught_route_is_patrolled_with_inspections_and_a_handheld_override(world):
    operator, spinner, _ = world
    path = os.path.relpath(operator.saved.last.data, SITES)
    assert operator.go_to(spinner, *START), 'did not get back to the start'
    reply = operator.call(spinner, 'navigation/load_path', LoadPath.Request(path=path))
    assert reply.success, reply.message
    before = len(operator.stops.messages)
    assert operator.call(spinner, 'navigation/start').success

    def events():
        return list(operator.stops.messages[before:])

    assert wait_for(lambda: any(e.event == StopPointEvent.DEPARTED for e in events()), 120.0)
    handheld = Handheld()
    try:
        drive_with_the_handheld(operator, handheld)
    finally:
        handheld.close()
    assert wait_for(lambda: operator.nav.last.state == NavigationStatus.GOAL_REACHED, 240.0), (
        f'patrol did not finish: {operator.nav.last}'
    )
    inspected = [e for e in events() if e.event == StopPointEvent.INSPECTED]
    assert len(inspected) == 3, patrol_report(operator, events())
    for event in inspected:
        assert event.message.startswith('/anomaly_detector_node/check_now: ok trash'), (
            event.message
        )
    yaws = [round(math.degrees(e.yaw)) for e in events() if e.event == StopPointEvent.ARRIVED]
    assert len(yaws) == 3, yaws
    assert len(OllamaHandler.images) >= 3


def test_netra_saw_the_patrol(world):
    _, _, netra = world
    assert wait_for(lambda: len(netra.of_type('stop_event')) >= 9, 20.0)
    assert wait_for(  # telemetry goes out once a second
        lambda: any(t['nav']['state'] == 'GOAL_REACHED' for t in netra.of_type('telemetry')), 10.0
    ), {t['nav']['state'] for t in netra.of_type('telemetry')}
    telemetry = netra.of_type('telemetry')
    assert {t['mode'] for t in telemetry} == {'auto', 'remote'}
    assert any(t['localization'] != 'none' for t in telemetry)
    assert any(t['health'] and 'cpu_percent' in t['health'] for t in telemetry)
    # the stop-point findings with their evidence frames
    assert wait_for(
        lambda: sum(p == '/api/v1/robot/anomalies' for p, _ in netra.uploads) >= 3, 30.0
    ), [p for p, _ in netra.uploads]
    upload = next(body for p, body in netra.uploads if p == '/api/v1/robot/anomalies')
    assert b'"category": "trash"' in upload and b'image/jpeg' in upload
