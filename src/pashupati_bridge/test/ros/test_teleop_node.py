"""In-process tests of teleop_udp_node: a real UDP client against the running node."""

from __future__ import annotations

import json
import time

import pytest
from teleop_rig import acks_of


def test_hello_is_welcomed(rig):
    """A client with the right token gets the welcome message."""
    a = rig.client()
    rig.send(a, {'t': 'hello', 'name': 'test'})
    assert any(m['t'] == 'welcome' for m in rig.received(a))


def test_wrong_token_is_denied(rig):
    """A hello without the right token gets `denied` (once a second); nothing else, no motion."""
    a = rig.client()
    rig.send(a, json.dumps({'t': 'hello'}).encode(), raw=True)
    rig.send(a, json.dumps({'t': 'hello', 'k': 'wrong'}).encode(), raw=True)
    rig.send(a, json.dumps({'t': 'joy', 'vx': 0.5, 'k': 'wrong'}).encode(), raw=True)
    assert [m['t'] for m in rig.received(a)] == ['denied']
    assert rig.twists == []


def test_joystick_is_clamped_and_garbage_stops(rig):
    """Speeds are clamped to the limits; NaN, inf and strings become zero."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.send(a, {'t': 'joy', 'vx': 5.0, 'vy': -5.0, 'wz': 0.3})
    t = rig.twists[-1]
    assert (t.linear.x, t.linear.y, t.angular.z) == (1.0, -0.5, 0.3)
    for bad in (b'NaN', b'Infinity', b'"fast"', b'[1]', b'true'):
        rig.send(a, b'{"k": "secret", "t": "joy", "vx": ' + bad + b'}', raw=True)
        assert rig.twists[-1].linear.x == 0.0, bad


def test_malformed_messages_do_not_kill_the_node(rig):
    """Broken datagrams are dropped and the node keeps serving."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    for junk in (
        b'\xff\xfe',
        b'[]',
        b'{"k": "secret"}',
        b'{"k": "secret", "t": "cmd", "id": [1]}',
        b'{"k": "secret", "t": "hello", "streams": 5}',
    ):
        rig.send(a, junk, raw=True)
    rig.send(a, {'t': 'ping', 'ts': 7})
    assert {'t': 'pong', 'ts': 7} in rig.received(a)


def test_second_device_is_busy_while_first_drives(rig):
    """Another IP cannot take over a live session; the same device may reconnect."""
    a, b = rig.client('127.0.0.1'), rig.client('127.0.0.2')
    rig.send(a, {'t': 'hello'})
    rig.received(a)
    rig.send(b, {'t': 'hello'})
    assert [m['t'] for m in rig.received(b)] == ['busy']
    rig.send(b, {'t': 'joy', 'vx': 0.5})
    assert not rig.twists or rig.twists[-1].linear.x == 0.0
    a2 = rig.client('127.0.0.1')  # app restart: same device, new port
    rig.send(a2, {'t': 'hello'})
    assert any(m['t'] == 'welcome' for m in rig.received(a2))


def test_route_outside_the_sites_is_refused(rig):
    """load_route only accepts route files under sites_dir, with a reason."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.received(a)
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'load_route', 'args': {'file': '../../etc/x.csv'}})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'load_route', 'args': {'file': '/etc/passwd'}})
    acks = acks_of(rig, a)
    assert not acks[1]['ok'] and 'under' in acks[1]['msg']
    assert not acks[2]['ok']


def test_handheld_estop_latches_without_waiting_for_services(rig):
    """Handheld estop and emergency_stop publish an e-stop request at once (no mux needed)."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'emergency_stop'})
    rig.spin(0.3)
    assert rig.estop_requests == ['handheld estop', 'handheld emergency_stop']
    acks = {m['id']: m for m in rig.received(a) if m['t'] == 'ack'}
    assert acks[1]['ok'] is False  # no mux in this test: the service is reported unavailable


def test_command_ids_are_per_session(rig):
    """A resent (id, name) runs once; after an app restart the same id runs again."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    assert rig.estop_requests == ['handheld estop']
    a2 = rig.client('127.0.0.1')  # app restart: new port, ids start over
    rig.send(a2, {'t': 'hello'})
    rig.send(a2, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    rig.spin(0.2)
    assert rig.estop_requests == ['handheld estop', 'handheld estop']


def test_state_carries_apriltags_while_the_camera_is_on(rig):
    """Tags the detector sees reach the handheld as image fractions, with the camera stream."""
    tag_msgs = pytest.importorskip('apriltag_msgs.msg')
    camera_info = pytest.importorskip('sensor_msgs.msg').CameraInfo
    info_pub = rig.listener.create_publisher(camera_info, '/perception/apriltag/camera_info', 10)
    tag_pub = rig.listener.create_publisher(
        tag_msgs.AprilTagDetectionArray, '/perception/apriltag/detections', 10
    )
    a = rig.client()
    rig.send(a, {'t': 'hello', 'streams': {'camera': True}})
    tag = tag_msgs.AprilTagDetection(id=3)
    tag.corners = [
        tag_msgs.Point(x=float(x), y=float(y)) for x, y in ((0, 0), (320, 0), (320, 240), (0, 240))
    ]
    tags, end = [], time.monotonic() + 5.0
    while not tags and time.monotonic() < end:
        info_pub.publish(camera_info(width=640, height=480))
        tag_pub.publish(tag_msgs.AprilTagDetectionArray(detections=[tag]))
        rig.spin(0.2)
        states = [m for m in rig.received(a) if m.get('t') == 'state']
        tags = states[-1]['tags'] if states else []
    assert tags == [[3, 0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.0, 0.5]]


def test_the_stick_drives_through_the_mux_source(rig):
    """The stick reaches control/remote/handheld/cmd_vel only (the mux publishes cmd_vel)."""
    published = {name for name, _ in rig.node.get_publisher_names_and_types_by_node(
        'teleop_udp_node', '/')}  # fmt: skip
    assert '/control/remote/handheld/cmd_vel' in published
    assert '/cmd_vel' not in published and not any(p.endswith('/cmd_vel') and 'remote' not in p
                                                    for p in published)  # fmt: skip


def test_streams_are_subscribed_only_while_wanted(rig):
    """The camera and TF are subscribed while a client wants them, and dropped when it leaves."""
    camera = '/camera/camera/color/image_raw/compressed'

    def topics():
        return {s.topic_name for s in rig.node.subscriptions}

    assert camera not in topics() and '/tf' not in topics()
    a = rig.client()
    rig.send(a, {'t': 'hello', 'streams': {'camera': True}})
    assert camera in topics() and '/tf' in topics()
    rig.send(a, {'t': 'streams', 'camera': False})
    assert camera not in topics() and '/tf' in topics()
    rig.send(a, {'t': 'bye'})
    assert '/tf' not in topics()


def test_no_webrtc_video_without_mediamtx(rig):
    """Without MediaMTX (video_port 0) the welcome offers no WebRTC: JPEG frames only."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    welcome = next(m for m in rig.received(a) if m['t'] == 'welcome')
    assert 'video' not in welcome


def test_webrtc_video_is_offered_when_mediamtx_runs(video_rig):
    """The welcome names the WHEP port and path when robot.launch.py started MediaMTX."""
    a = video_rig.client()
    video_rig.send(a, {'t': 'hello'})
    welcome = next(m for m in video_rig.received(a) if m['t'] == 'welcome')
    assert welcome['video'] == {'port': 8890, 'path': 'camera'}


def test_webrtc_video_keeps_the_tags_without_jpeg_frames(video_rig):
    """While the handheld plays WebRTC video the tags still come, the JPEG camera does not."""
    pytest.importorskip('apriltag_msgs.msg')
    camera = '/camera/camera/color/image_raw/compressed'
    tags = '/perception/apriltag/detections'

    def topics():
        return {s.topic_name for s in video_rig.node.subscriptions}

    a = video_rig.client()
    video_rig.send(a, {'t': 'hello', 'streams': {'camera': False, 'video': True}})
    assert tags in topics() and camera not in topics()
    video_rig.send(a, {'t': 'streams', 'video': False, 'camera': True})
    assert tags in topics() and camera in topics()
    video_rig.send(a, {'t': 'streams', 'camera': False})
    assert tags not in topics()


def test_discovery_needs_no_token_and_states_count_up(rig):
    """A discover probe gets a beacon; the states of a session carry an increasing seq."""
    a = rig.client()
    rig.send(a, json.dumps({'t': 'discover'}).encode(), raw=True)
    (beacon,) = rig.received(a)
    assert beacon['t'] == 'beacon' and beacon['secured'] is True and beacon['in_use_by'] == ''
    rig.send(a, {'t': 'hello', 'name': 'phone'})
    rig.spin(0.5)
    seqs = [m['seq'] for m in rig.received(a) if m['t'] == 'state']
    assert len(seqs) >= 2 and seqs == sorted(seqs)
    b = rig.client('127.0.0.2')
    rig.send(b, json.dumps({'t': 'discover'}).encode(), raw=True)
    assert rig.received(b)[0]['in_use_by'] == 'phone'
