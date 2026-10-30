"""A teleop_udp_node with UDP clients, for the in-process tests."""

from __future__ import annotations

import json
import socket
import time

from geometry_msgs.msg import Twist
from pashupati_bridge.teleop_udp_node import TeleopUdpNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from dharma_utils.ros import compat

TOKEN = 'secret'
ROBOT = ['robot.limits.max_vx:=1.0', 'robot.limits.max_vy:=0.5', 'robot.limits.max_wz:=1.5']


def free_port() -> int:
    """Return a UDP port nobody listens on."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class Rig:
    """The node, a listener on the stick source and UDP clients."""

    def __init__(self, params=()):
        self.port = free_port()
        args = ['--ros-args', '-p', f'port:={self.port}', '-p', f'token:={TOKEN}']
        for param in [*ROBOT, *params]:
            args += ['-p', param]
        compat.init(args=args)
        try:
            self.node = TeleopUdpNode()
        except Exception:
            compat.try_shutdown()  # keep the next test's init working
            raise
        self.listener = rclpy.create_node('teleop_test_listener')
        self.twists, self.estop_requests = [], []
        self.listener.create_subscription(
            Twist, 'control/remote/handheld/cmd_vel', self.twists.append, 10
        )
        self.listener.create_subscription(
            String, 'control/estop_request', lambda m: self.estop_requests.append(m.data), 10
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
        compat.try_shutdown()


def wait_until(rig, condition, timeout: float = 5.0) -> bool:
    """Spin until condition holds (service discovery, latched messages) or timeout passes."""
    end = time.monotonic() + timeout
    while not condition() and time.monotonic() < end:
        rig.spin(0.05)
    return condition()


def fake_node(rig, name: str, params: dict):
    """Add a node with the given parameters (and so their parameter services) to the rig."""
    node = rclpy.create_node(name)
    for param, value in params.items():
        node.declare_parameter(param, value)
    rig.executor.add_node(node)
    return node


def acks_of(rig, sock) -> dict:
    """Return the acknowledgements waiting on a client socket, by command id."""
    return {m['id']: m for m in rig.received(sock) if m['t'] == 'ack'}
