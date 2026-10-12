"""In-process tests of teleop_udp_node: a real UDP client against the running node."""

import json
import socket
import time

import pytest

rclpy = pytest.importorskip('rclpy')
Twist = pytest.importorskip('geometry_msgs.msg').Twist
String = pytest.importorskip('std_msgs.msg').String
SingleThreadedExecutor = pytest.importorskip('rclpy.executors').SingleThreadedExecutor
TeleopUdpNode = pytest.importorskip('robot_bridge.teleop_udp_node').TeleopUdpNode

TOKEN = 'secret'


def free_port() -> int:
    """Return a UDP port nobody listens on."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class Rig:
    """The node, a listener on bridge/remote_cmd_vel and UDP clients."""

    def __init__(self):
        self.port = free_port()
        rclpy.init(args=['--ros-args', '-p', f'port:={self.port}', '-p', f'token:={TOKEN}'])
        try:
            self.node = TeleopUdpNode()
        except Exception:
            rclpy.shutdown()  # keep the next test's rclpy.init() working
            raise
        self.listener = rclpy.create_node('teleop_test_listener')
        self.twists, self.estop_requests = [], []
        self.listener.create_subscription(
            Twist, 'bridge/remote_cmd_vel', self.twists.append, 10
        )
        self.listener.create_subscription(
            String, 'bridge/estop_request', lambda m: self.estop_requests.append(m.data), 10
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)
        self.clients = []

    def client(self, ip: str = '127.0.0.1') -> socket.socket:
        """Open a client socket bound to ip (127.0.0.2 counts as another device)."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, 0))
        sock.settimeout(0.05)
        self.clients.append(sock)
        return sock

    def send(self, sock, message, raw: bool = False):
        """Send one datagram to the node and let it process it."""
        data = message if raw else json.dumps({'k': TOKEN, **message}).encode()
        sock.sendto(data, ('127.0.0.1', self.port))
        self.spin(0.2)

    def spin(self, seconds: float):
        """Spin both nodes for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def received(self, sock) -> list:
        """Return the JSON messages waiting on a client socket."""
        out = []
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except (socket.timeout, BlockingIOError):
                return out
            if data[:1] == b'{':
                out.append(json.loads(data))

    def close(self):
        """Tear everything down."""
        for sock in self.clients:
            sock.close()
        self.executor.shutdown()
        self.listener.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def rig():
    """Provide a running teleop node."""
    r = Rig()
    yield r
    r.close()


def test_hello_is_welcomed(rig):
    """A client with the right token gets the welcome message."""
    a = rig.client()
    rig.send(a, {'t': 'hello', 'name': 'test'})
    assert any(m['t'] == 'welcome' for m in rig.received(a))


def test_wrong_token_is_ignored(rig):
    """Without the token the robot neither answers nor moves."""
    a = rig.client()
    rig.send(a, json.dumps({'t': 'hello'}).encode(), raw=True)
    rig.send(a, json.dumps({'t': 'joy', 'vx': 0.5, 'k': 'wrong'}).encode(), raw=True)
    assert rig.received(a) == []
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
    for junk in (b'\xff\xfe', b'[]', b'{"k": "secret"}', b'{"k": "secret", "t": "cmd", "id": [1]}',
                 b'{"k": "secret", "t": "hello", "streams": 5}'):
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


def test_route_outside_routes_dir_is_refused(rig):
    """load_route only accepts files under routes_dir."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.received(a)
    with pytest.raises(ValueError):
        rig.node.route_path('/etc/passwd')
    with pytest.raises(ValueError):
        rig.node.route_path('../../etc/passwd')


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
