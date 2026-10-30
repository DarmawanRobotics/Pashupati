"""fleet_uplink_node against the fake NETRA: telemetry, events, commands, uploads."""

from __future__ import annotations

import json
import os

from pashupati_fleet.tools.fake_netra import FakeNetra
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dharma_interfaces.msg import (
    ControlStatus,
    NavigationStatus,
    SiteStatus,
    StopPointEvent,
)
from dharma_interfaces.srv import LoadPath
from dharma_utils.ros import EVENTS, LATCHED, compat
from dharma_utils.testing import Spinner, wait_for

pytest.importorskip('websocket')
pytest.importorskip('requests')
FleetUplinkNode = pytest.importorskip('pashupati_fleet.fleet_uplink_node').FleetUplinkNode


class Stack:
    """The services and topics of the Dharma stack the uplink talks to."""

    def __init__(self, node):
        self.node = node
        self.calls, self.estop_requests = [], []
        for name in ('navigation/start', 'control/estop/engage'):
            node.create_service(Trigger, name, lambda req, res, n=name: self.trigger(n, res))
        node.create_service(LoadPath, 'navigation/load_path', self.load)
        node.create_subscription(String, 'control/estop_request',
                                 lambda m: self.estop_requests.append(m.data), EVENTS)  # fmt: skip
        self.control = node.create_publisher(ControlStatus, 'control/status', LATCHED)
        self.nav = node.create_publisher(NavigationStatus, 'navigation/status', 10)
        self.stop = node.create_publisher(StopPointEvent, 'navigation/stop_point', EVENTS)
        self.site = node.create_publisher(SiteStatus, 'site/status', LATCHED)
        self.saved = node.create_publisher(String, 'route/saved', EVENTS)

    def trigger(self, name, response):
        self.calls.append(name)
        response.success, response.message = True, f'{name} ok'
        return response

    def load(self, request, response):
        self.calls.append(f'load {request.path}')
        response.success = True
        return response


@pytest.fixture
def rig(tmp_path):
    netra = FakeNetra().start()
    compat.init(args=['--ros-args', '-p', f'server_url:={netra.url}', '-p', 'token:=t0k',
                      '-p', 'robot_id:=sim-01', '-p', f'spool_dir:={tmp_path / "spool"}',
                      '-p', f'sites_dir:={tmp_path / "sites"}',
                      '-p', 'upload_period_sec:=0.5'])  # fmt: skip
    node = FleetUplinkNode()
    helper = rclpy.create_node('fleet_test_stack')
    stack = Stack(helper)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    spinner = Spinner(executor)
    yield netra, stack, node, tmp_path
    spinner.stop()
    node.destroy_node()
    helper.destroy_node()
    compat.try_shutdown()
    netra.close()


def ack(netra, cid):
    acks = [m for m in netra.of_type('ack') if m['id'] == cid]
    return acks[-1] if acks else None


def test_hello_and_telemetry_from_the_typed_status(rig):
    netra, stack, _, _ = rig
    assert wait_for(lambda: netra.of_type('hello'), 10.0)
    assert (
        netra.headers['Authorization'] == 'Bearer t0k' and netra.headers['X-Robot-Id'] == 'sim-01'
    )
    stack.control.publish(ControlStatus(mode=ControlStatus.MODE_REMOTE,
                                        field=ControlStatus.FIELD_CLEAR))  # fmt: skip
    assert wait_for(lambda: netra.of_type('telemetry') and
                    netra.of_type('telemetry')[-1]['mode'] == 'remote', 10.0)  # fmt: skip
    telemetry = netra.of_type('telemetry')[-1]
    assert telemetry['field'] == 'clear' and telemetry['nav']['state'] == 'NAV_INACTIVE'
    assert {'pose', 'localization', 'battery', 'health', 'patrol', 'ts'} <= set(telemetry)


def test_estop_alert_and_command(rig):
    netra, stack, _, _ = rig
    assert wait_for(lambda: netra.of_type('hello'), 10.0)
    assert wait_for(lambda: netra.command(1, 'estop'), 5.0)
    assert wait_for(lambda: ack(netra, 1) is not None, 10.0)
    assert ack(netra, 1)['ok'] and stack.estop_requests == ['command center']
    stack.control.publish(ControlStatus(estop=True, estop_reasons=['command center']))
    assert wait_for(lambda: [a for a in netra.of_type('alert') if a['state'] == 'ESTOP'], 10.0)
    netra.command(2, 'fly')
    assert wait_for(lambda: ack(netra, 2) is not None, 10.0)
    assert not ack(netra, 2)['ok'] and 'unknown' in ack(netra, 2)['msg']


def test_start_patrol_downloads_into_the_site_and_reports(rig):
    netra, stack, _, tmp_path = rig
    netra.routes['42'] = b'x,y,yaw_deg,dwell_sec\n0,0,0,0\n1,0,0,5\n'
    assert wait_for(lambda: netra.of_type('hello'), 10.0)
    netra.command(1, 'start_patrol', route_id='42', patrol_id='p9')
    assert wait_for(lambda: ack(netra, 1) is not None, 10.0)
    assert not ack(netra, 1)['ok'] and 'site' in ack(netra, 1)['msg']  # no active site yet
    stack.site.publish(SiteStatus(site='lobby'))
    wait_for(lambda: False, 0.5)
    netra.command(2, 'start_patrol', route_id='42', patrol_id='p9')
    assert wait_for(lambda: ack(netra, 2) is not None, 10.0)
    assert ack(netra, 2)['ok'], ack(netra, 2)
    assert stack.calls == ['load lobby/netra/42.csv', 'navigation/start']
    assert (tmp_path / 'sites' / 'lobby' / 'netra' / '42.csv').read_bytes() == netra.routes['42']
    assert wait_for(lambda: netra.of_type('patrol_event'), 5.0)
    assert netra.of_type('patrol_event')[0]['patrol_id'] == 'p9'
    stack.stop.publish(StopPointEvent(event=StopPointEvent.ARRIVED, index=1, x=1.0, yaw=1.5708))
    assert wait_for(lambda: netra.of_type('stop_event'), 5.0)
    event = netra.of_type('stop_event')[0]
    assert event['event'] == 'arrived' and event['yaw_deg'] == 90.0 and event['patrol_id'] == 'p9'
    stack.nav.publish(NavigationStatus(state=NavigationStatus.GOAL_REACHED))
    assert wait_for(lambda: len(netra.of_type('patrol_event')) == 2, 5.0)
    assert netra.of_type('patrol_event')[1]['event'] == 'finished'


def test_taught_routes_are_uploaded(rig):
    netra, stack, _, tmp_path = rig
    route = tmp_path / 'r.csv'
    route.write_text('x,y,yaw_deg,dwell_sec\n0,0,0,0\n3,4,0,5\n')
    assert wait_for(lambda: netra.of_type('hello'), 10.0)
    stack.saved.publish(String(data=str(route)))
    assert wait_for(lambda: netra.uploads, 10.0)
    path, body = netra.uploads[0]
    assert path == '/api/v1/robot/routes' and b'route.csv' in body
    meta = json.loads(body.split(b'name="meta"')[1].split(b'\r\n\r\n', 1)[1].split(b'\r\n')[0])
    assert meta['stops'] == 1 and meta['length_m'] == 5.0
    assert wait_for(lambda: not os.listdir(tmp_path / 'spool') or
                    os.listdir(tmp_path / 'spool') == ['.failed'], 5.0)  # fmt: skip
