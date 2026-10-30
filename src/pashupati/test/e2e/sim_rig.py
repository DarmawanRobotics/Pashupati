"""The pieces of the end-to-end test: the launched simulation, a stand-in VLM, a handheld."""

from __future__ import annotations

import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import signal
import socket
import subprocess
import tempfile
import threading
import time

ANSWER = 'trash'  # what the stand-in VLM sees on every frame
TOKEN = 'sim'  # teleop_udp_node token in config/sim.yaml
TELEOP_PORT = 9870


class OllamaHandler(BaseHTTPRequestHandler):
    """Ollama's HTTP API, just enough for anomaly_detector_node."""

    images = []  # sizes of the images asked about

    def log_message(self, *args):
        """Keep the console quiet."""

    def reply(self, body: dict):
        """Answer 200 with a JSON body."""
        data = json.dumps(body).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        """List the one model."""
        self.reply({'models': [{'name': 'qwen3-vl:2b-instruct'}]})

    def do_POST(self):
        """Show capabilities, load the model, or answer an image."""
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        if self.path == '/api/show':
            self.reply({'capabilities': ['completion', 'vision']})
        elif body.get('images'):
            OllamaHandler.images.append(len(base64.b64decode(body['images'][0])))
            self.reply({'response': json.dumps({'type': ANSWER})})
        else:
            self.reply({'done': True})


def serve(handler) -> ThreadingHTTPServer:
    """Serve handler on a free local port in the background."""
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


class Launch:
    """ros2 launch pashupati sim.launch.py in its own session, its output in a file."""

    def __init__(self, domain: int, arguments: list[str]):
        self.log = tempfile.NamedTemporaryFile(  # noqa: SIM115  kept until the end of the module
            'w+', prefix='pashupati_sim_', suffix='.log', delete=False
        )
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
        # a background job runs with SIGINT ignored, and the child would inherit that: let it
        # exec with the default action, so SIGINT stops the launch like systemd does
        previous = signal.signal(signal.SIGINT, signal.default_int_handler)
        self.proc = subprocess.Popen(
            ['ros2', 'launch', 'pashupati', 'sim.launch.py', *arguments],
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=self.log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        signal.signal(signal.SIGINT, previous)

    def stop(self, timeout: float = 45.0) -> tuple[bool, str]:
        """Stop the launch like systemd (one signal to ros2 launch); return (clean, log)."""
        clean = True
        if self.proc.poll() is None:
            os.kill(self.proc.pid, signal.SIGINT)  # ros2 launch only: it stops every node once
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                clean = False
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.log.seek(0)
        text = self.log.read()
        self.log.close()
        return clean, text


class Handheld:
    """A Pashupati Controller over UDP (PROTOCOL.md): hello, commands, stick."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.05)
        self.next_id = 100
        self.messages: list[dict] = []

    def send(self, message: dict):
        """Send one JSON datagram with the token."""
        data = json.dumps({'k': TOKEN, **message}).encode()
        self.sock.sendto(data, ('127.0.0.1', TELEOP_PORT))

    def receive(self):
        """Read every waiting JSON datagram."""
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (socket.timeout, BlockingIOError):
                return
            if data[:1] == b'{':
                self.messages.append(json.loads(data))

    def hello(self, timeout: float = 20.0) -> bool:
        """Connect; True once welcomed."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.send({'t': 'hello', 'name': 'e2e', 'streams': {}})
            time.sleep(0.2)
            self.receive()
            if any(m.get('t') == 'welcome' for m in self.messages):
                return True
        return False

    def command(self, name: str, timeout: float = 10.0, **args) -> dict | None:
        """Run a command until its ack arrives (resent like the app); return the ack."""
        self.next_id += 1
        cid = self.next_id
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.send({'t': 'cmd', 'id': cid, 'name': name, 'args': args})
            time.sleep(0.2)
            self.receive()
            acks = [m for m in self.messages if m.get('t') == 'ack' and m.get('id') == cid]
            if acks and acks[-1]['ok'] is not None:
                return acks[-1]
        return None

    def stick(self, vx: float, seconds: float):
        """Hold the stick at 20 Hz."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send({'t': 'joy', 'vx': vx, 'vy': 0.0, 'wz': 0.0})
            time.sleep(0.05)

    def states(self) -> list[dict]:
        """Return the state messages received so far."""
        self.receive()
        return [m for m in self.messages if m.get('t') == 'state']

    def close(self):
        """Leave."""
        self.send({'t': 'bye'})
        self.sock.close()


def problems(log: str) -> tuple[list[str], list[str]]:
    """Return the crash lines of a launch log: (while running, while stopping)."""
    bad = ('process has died', 'Traceback')
    lines = log.splitlines()
    stop = next(
        (i for i, line in enumerate(lines) if "sending signal 'SIGINT'" in line), len(lines)
    )
    crashes = [(i, line) for i, line in enumerate(lines) if any(b in line for b in bad)]
    return [line for i, line in crashes if i < stop], [line for i, line in crashes if i >= stop]
