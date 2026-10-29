"""
teleop_udp_node: the handheld's UDP gateway (PROTOCOL.md), no ROS on the handheld.

The stick goes to the control mux as an operator source (control/remote/<source>/cmd_vel): the
mux decides what reaches the robot. Commands call the stack's services, telemetry comes from
the typed status topics.
"""

from __future__ import annotations

import socket
import time

from geometry_msgs.msg import Twist
from pashupati_utils.utils.calls import call_service
from std_msgs.msg import String

from dharma_utils.ros import COMMAND, EVENTS, DharmaNode, run
from pashupati_bridge.core import commands, protocol
from pashupati_bridge.core.config import TeleopConfig
from pashupati_bridge.core.routes import list_routes
from pashupati_bridge.core.session import Session
from pashupati_bridge.utils.clients import CommandClients, PatrolSpeed
from pashupati_bridge.utils.link import UdpLink
from pashupati_bridge.utils.streams import Streams
from pashupati_bridge.utils.telemetry import Telemetry

SPEED_POLL_SEC = 3.0  # the follower's patrol speed is asked this often while connected


class TeleopUdpNode(DharmaNode):
    """UDP gateway for the handheld: stick, commands, telemetry, camera, cloud and route."""

    def __init__(self, **kwargs):
        super().__init__('teleop_udp_node', **kwargs)
        self.config = self.declare_config(TeleopConfig)
        self.robot = self.declare_robot()
        cfg, limits = self.config.value, self.robot.limits
        vy = limits.max_vy if self.robot.kinematics == 'omni' else 0.0
        self.limits = (min(cfg.max_vx, limits.max_vx), min(cfg.max_vy, vy),
                       min(cfg.max_wz, limits.max_wz))  # fmt: skip
        self.session = Session(cfg.token, cfg.client_timeout_sec)
        self.robot_name = cfg.robot_name or socket.gethostname()
        self.seq = 0
        self.video = {'port': cfg.video_port, 'path': cfg.video_path} if cfg.video_port else None
        source = f'control/remote/{cfg.remote_source}/cmd_vel'
        self.remote_pub = self.create_publisher(Twist, source, COMMAND)
        self.estop_pub = self.create_publisher(String, 'control/estop_request', EVENTS)
        self.calls = CommandClients(self, cfg, self.speed_range)
        self.patrol = PatrolSpeed(self.calls.speed_client, self.speed_range)
        self.telemetry = Telemetry(self, cfg.health_diagnostics_topic, self.send_route)
        self.streams = Streams(self, cfg, self.send_stream)
        # A reader thread wakes the executor per datagram: no polling timer while idle.
        self.wake = self.create_guard_condition(self.drain)
        try:
            self.link = UdpLink(cfg.port, self.wake.trigger)
        except OSError:
            self.log.error('cannot bind the UDP port (another gateway running?)', port=cfg.port)
            raise
        if not cfg.token:
            self.log.warning('teleop token is empty: anyone on this network can drive the robot')
        self.every(1.0 / cfg.telemetry_hz, self.tick)
        self.add_shutdown_hook(lambda: self.send_json({'t': 'bye'}))
        self.log.info('teleop UDP', port=cfg.port, source=source, limits=self.limits)

    def destroy_node(self):
        """Close the socket, so the port is free again at once."""
        self.link.close()
        return super().destroy_node()

    # ---------------------------------------------------------------- socket
    def send(self, data: bytes, addr=None) -> bool:
        """Send one datagram to the client (or addr)."""
        return self.link.send(data, addr or self.session.client)

    def send_json(self, message: dict, addr=None):
        """Send a JSON message to the client (or addr)."""
        self.link.send_json(message, addr or self.session.client)

    def send_stream(self, stream: int, payload: bytes):
        """Send a chunked payload to the client."""
        self.link.send_stream(stream, payload, self.session.client)

    def drain(self):
        """Handle the received datagrams."""
        for data, addr in self.link.take():
            message = protocol.decode_json(data)
            if message is None:
                continue
            if message.get('t') == 'discover':
                self.reply_beacon(addr)  # no token needed, nothing sensitive
            elif not self.session.authorized(message):
                if message.get('t') == 'hello' and self.session.deny_now(addr, time.monotonic()):
                    self.log.warning('teleop hello with a wrong token', ip=addr[0], once=True)
                    self.send_json({'t': 'denied', 'msg': 'wrong token'}, addr)
            else:
                try:
                    self.handle_message(message, addr)
                except (AttributeError, KeyError, TypeError, ValueError) as error:
                    self.log.warning('malformed teleop message', error=str(error), throttle=5.0)

    # ---------------------------------------------------------------- messages
    def handle_message(self, message: dict, addr):
        """Dispatch one client message."""
        kind, now = message['t'], time.monotonic()
        if kind == 'hello':
            if not self.on_hello(message, addr, now):
                return
        elif addr != self.session.client:
            return
        self.session.seen = now
        if kind == 'joy':
            self.on_joy(message)
        elif kind == 'cmd':
            self.on_command(message)
        elif kind == 'ping':
            self.send_json({'t': 'pong', 'ts': message.get('ts')})
        elif kind == 'streams':
            self.streams.update(message)
        elif kind == 'bye':
            self.end_session()
        self.streams.sync(self.session.client is not None)

    def on_hello(self, message: dict, addr, now: float) -> bool:
        """Welcome a client, or tell it another device is driving."""
        busy = self.session.busy_for(addr, now)
        if busy:
            self.send_json({'t': 'busy', 'msg': busy}, addr)
            return False
        if addr != self.session.client:
            self.stop_remote()
            self.log.info('teleop client', ip=addr[0], name=str(message.get('name', '?')))
        self.session.accept(addr, message.get('name', ''), now)
        streams = message.get('streams', {})
        self.streams.wanted = dict.fromkeys(self.streams.wanted, False)
        self.streams.update(streams if isinstance(streams, dict) else {})
        welcome = {
            't': 'welcome',
            'name': self.robot_name,
            'commands': self.calls.names(),
            'robot_commands': self.calls.robot_commands,
            'limits': list(self.limits),
        }
        if self.video:
            welcome['video'] = self.video  # WebRTC (WHEP) through MediaMTX
        self.send_json(welcome)
        self.send_route()
        self.patrol.ask()
        return True

    def on_joy(self, message: dict):
        """Publish a clamped stick command; NaN, inf and non-numbers count as 0."""
        twist = Twist()
        vx, vy, wz = self.limits
        twist.linear.x = protocol.clamp_axis(message.get('vx'), vx)
        twist.linear.y = protocol.clamp_axis(message.get('vy'), vy)
        twist.angular.z = protocol.clamp_axis(message.get('wz'), wz)
        self.remote_pub.publish(twist)

    def on_command(self, message: dict):
        """Run a named command once per (id, name) in this session and acknowledge it."""
        cid, name = commands.command_id(message.get('id')), str(message.get('name', ''))
        args = message.get('args') if isinstance(message.get('args'), dict) else {}
        if cid is None:
            return
        key = (cid, name)
        if key in self.session.acks:
            self.send_json(self.session.acks[key])
            return
        if name in commands.ESTOP_COMMANDS:  # latch first, without waiting for any service
            self.estop_pub.publish(String(data=f'handheld {name}'))
        if name in commands.LOCAL:
            self.local_command(key, name)
            return
        if name not in self.calls.table:
            self.ack(key, False, f'unknown command {name}')
            return
        client, make = self.calls.table[name]
        try:
            request = make(args)
        except (TypeError, ValueError) as error:
            self.ack(key, False, str(error))
            return
        number = self.session.number
        if call_service(client, request, lambda ok, msg, _r: self.done(key, number, ok, msg)):
            self.session.remember(key, {'t': 'ack', 'id': cid, 'ok': None, 'msg': 'pending'})
        else:
            self.ack(key, False, f'{client.srv_name} not available')

    def local_command(self, key: tuple, name: str):
        """Answer the commands the gateway knows itself."""
        if name == 'list_routes':
            self.ack(key, True, '', list_routes(self.config.sites_dir))
        elif name == 'get_route':
            self.send_route()
            self.ack(key, True, f'{len(self.telemetry.route)} points')
        else:
            self.ack(key, *self.telemetry.inspection_list(name))

    def done(self, key: tuple, number: int, ok: bool, message: str):
        """Acknowledge a finished service call, unless another session started meanwhile."""
        name = key[1]
        if ok and name in ('record_start', 'record_stop'):
            self.telemetry.state['recording'] = name == 'record_start'
        if ok and name == 'nav_speed':
            self.patrol.asked = 0.0  # show the new speed with the next state
        if number == self.session.number:
            self.ack(key, ok, message)

    def ack(self, key: tuple, ok: bool, msg: str, data=None):
        """Send and remember an acknowledgement."""
        reply = {'t': 'ack', 'id': key[0], 'ok': ok, 'msg': msg}
        if data is not None:
            reply['data'] = data
        self.session.remember(key, reply)
        self.send_json(reply)

    def stop_remote(self):
        """One zero for the stick (a new or vanished client)."""
        if self.session.client is not None:
            self.remote_pub.publish(Twist())

    def end_session(self):
        """Forget the client and stop the stick."""
        self.stop_remote()
        self.session.end()

    # ---------------------------------------------------------------- telemetry
    def tick(self):
        """Expire a silent client, then send the telemetry snapshot."""
        now = time.monotonic()
        if self.session.client and not self.session.alive(now):
            self.log.warning('teleop client timed out', ip=self.session.client[0])
            self.end_session()
            self.streams.sync(False)
        if self.session.client is None:
            return
        if now - self.patrol.asked > SPEED_POLL_SEC:
            self.patrol.ask()
        self.seq += 1
        frames = self.robot.frames
        self.send_json({
            't': 'state',
            'seq': self.seq,
            'pose': self.streams.pose(frames.map, frames.base),
            'tags': self.streams.tags(),
            'inspection': self.telemetry.inspection(),
            'people': self.telemetry.people(),
            'speed': self.patrol.speed,
            **self.telemetry.state,
        })  # fmt: skip

    def send_route(self):
        """Send the loaded route as a chunked JSON payload."""
        payload = protocol.encode_json({'t': 'route', 'points': self.telemetry.route})
        self.send_stream(protocol.STREAM_ROUTE, payload)

    def reply_beacon(self, addr):
        """Answer a LAN discovery probe."""
        state = self.telemetry.state
        beacon = {
            't': 'beacon',
            'name': self.robot_name,
            'secured': bool(self.config.token),
            'in_use_by': self.session.name if self.session.alive(time.monotonic()) else '',
            'mode': state.get('mode'),
            'battery': (state.get('battery') or {}).get('pct'),
        }
        self.link.send_json(beacon, addr)

    def speed_range(self) -> tuple[float, float]:
        """Return the patrol speeds the handheld may set (m/s)."""
        cfg = self.config.value
        return cfg.patrol_speed_min, min(cfg.patrol_speed_max, self.robot.limits.max_vx)


def main(args=None):
    """Run the teleop gateway."""
    run(TeleopUdpNode, args=args)


if __name__ == '__main__':
    main()
