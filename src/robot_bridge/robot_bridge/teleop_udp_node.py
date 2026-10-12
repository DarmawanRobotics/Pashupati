#!/usr/bin/env python3
import hmac
import json
import math
import os
import socket
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_bridge import protocol
from robot_interfaces.msg import MissionStatus, NavigationStatus, WaypointPath
from robot_interfaces.srv import LoadPath, MarkStopPoint, SetMode
from sensor_msgs.msg import BatteryState, CompressedImage, Image, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

try:
    import cv2
except ImportError:  # raw images need OpenCV, compressed ones do not
    cv2 = None

ROBOT_COMMANDS = [
    'stand_up',
    'sit_down',
    'emergency_stop',
    'move_mode',
    'balance_stand_mode',
    'lock_mode',
    'slow_speed',
    'normal_speed',
    'fast_speed',
]


def yaw_of(q) -> float:
    """Return the yaw of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def finite(value, digits: int = 2):
    """Round a float for JSON, or None when it is NaN/inf."""
    return round(float(value), digits) if math.isfinite(value) else None


def dwell_seconds(value) -> float:
    """Validate a stop-point dwell time from the client (0 < dwell <= 1 h)."""
    dwell = float(value)
    if not math.isfinite(dwell) or not 0.0 < dwell <= 3600.0:
        raise ValueError(f'dwell must be between 0 and 3600 s, got {value!r}')
    return dwell


class TeleopUdpNode(Node):
    """UDP gateway for the handheld teleop: joystick, commands, telemetry, camera and lidar."""

    def __init__(self):
        """Declare params, open the socket and wire ROS interfaces."""
        super().__init__('teleop_udp_node')
        defaults = {
            'port': protocol.PORT,
            'token': '',
            'robot_name': '',
            'client_timeout_sec': 2.0,
            'max_vx': 1.0,
            'max_vy': 0.5,
            'max_wz': 1.5,
            'telemetry_hz': 10.0,
            'robot_command_prefix': '/l1w/',
            'robot_commands': ROBOT_COMMANDS,
            'routes_dir': '/home/robot/dev/Pashupati/map',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'camera_topic': '/camera/camera/color/image_raw/compressed',
            'camera_compressed': True,
            'camera_fps': 8.0,
            'camera_jpeg_quality': 60,
            'camera_max_width': 640,
            'lidar_topic': '/cloud_registered_body',
            'lidar_hz': 2.0,
            'lidar_voxel': 0.1,
            'lidar_max_points': 3000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self._token = p('token')
        self._robot_name = p('robot_name') or socket.gethostname()
        self._seq = 0
        self._client_name = ''
        self._client_timeout = float(p('client_timeout_sec'))
        self._limits = (float(p('max_vx')), float(p('max_vy')), float(p('max_wz')))
        self._routes_dir = p('routes_dir')
        self._map_frame, self._base_frame = p('map_frame'), p('base_frame')
        self._client = None
        self._client_seen = 0.0
        self._streams = {'camera': False, 'lidar': False}
        self._acks: dict = {}
        self._frame_ids = {
            protocol.STREAM_CAMERA: 0,
            protocol.STREAM_LIDAR: 0,
            protocol.STREAM_ROUTE: 0,
        }
        self._state = {
            'mode': 'auto',
            'nav': 'NAV_INACTIVE',
            'nav_msg': '',
            'mission': '',
            'localization': 'none',
            'recording': False,
            'battery': None,
            'driver': None,
            'cmd': [0.0, 0.0, 0.0],
            'health': None,
        }
        self._route: list = []
        self._extrinsics: dict = {}
        self._last_camera = self._last_lidar = 0.0
        self._camera_period = 1.0 / max(float(p('camera_fps')), 0.1)
        self._lidar_period = 1.0 / max(float(p('lidar_hz')), 0.1)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._sock.bind(('0.0.0.0', int(p('port'))))
        except OSError as error:
            self.get_logger().fatal(
                f'cannot bind UDP port {p("port")}: {error} (another teleop gateway running?)'
            )
            raise
        self._sock.setblocking(False)
        if not self._token:
            self.get_logger().warn(
                'teleop token is empty: anyone on this network can drive the robot, '
                'set teleop_udp_node.token'
            )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._remote_pub = self.create_publisher(Twist, 'bridge/remote_cmd_vel', 10)
        self.make_clients(p('robot_command_prefix'), p('robot_commands'))
        self.make_subscriptions(p('camera_topic'), bool(p('camera_compressed')), p('lidar_topic'))

        self.create_timer(0.01, self.poll)
        self.create_timer(1.0 / float(p('telemetry_hz')), self.send_state)
        self.get_logger().info(f'teleop UDP on port {p("port")}')

    # ------------------------------------------------------------------ setup

    def make_clients(self, prefix: str, robot_commands: list):
        """Map command names to (service type, client, request factory)."""

        def trigger(name):
            return Trigger, self.create_client(Trigger, name), lambda a: Trigger.Request()

        def set_bool(name, value):
            return (
                SetBool,
                self.create_client(SetBool, name),
                lambda a, v=value: SetBool.Request(data=v),
            )

        start_nav = self.create_client(SetBool, 'navigation/start_nav')
        pause = self.create_client(SetBool, 'navigation/pause')
        record = self.create_client(SetBool, 'route/record')
        self._commands = {
            'set_mode': (
                SetMode,
                self.create_client(SetMode, 'bridge/set_mode'),
                lambda a: SetMode.Request(mode=str(a.get('mode', 'auto'))),
            ),
            'nav_start': (SetBool, start_nav, lambda a: SetBool.Request(data=True)),
            'nav_stop': (SetBool, start_nav, lambda a: SetBool.Request(data=False)),
            'nav_pause': (SetBool, pause, lambda a: SetBool.Request(data=True)),
            'nav_resume': (SetBool, pause, lambda a: SetBool.Request(data=False)),
            'record_start': (SetBool, record, lambda a: SetBool.Request(data=True)),
            'record_stop': (SetBool, record, lambda a: SetBool.Request(data=False)),
            'mark_stop': (
                MarkStopPoint,
                self.create_client(MarkStopPoint, 'route/mark_stop_point'),
                lambda a: MarkStopPoint.Request(dwell_sec=dwell_seconds(a.get('dwell', 10.0))),
            ),
            'load_route': (
                LoadPath,
                self.create_client(LoadPath, 'navigation/load_path'),
                lambda a: LoadPath.Request(waypoints_file=self.route_path(a.get('file', ''))),
            ),
            'localize': trigger('localization/start'),
            'record_tags': trigger('localization/record_tags'),
        }
        for name in robot_commands:
            if name:
                self._commands[name] = trigger(prefix + name)

    def make_subscriptions(self, camera_topic: str, compressed: bool, lidar_topic: str):
        """Subscribe to everything the telemetry, camera and lidar streams need."""
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(NavigationStatus, 'navigation/status', self.on_nav_status, 10)
        self.create_subscription(MissionStatus, 'navigation/mission_status', self.on_mission, 10)
        self.create_subscription(String, 'localization/status', self.on_localization, latched)
        self.create_subscription(String, 'bridge/mode', self.on_mode, latched)
        self.create_subscription(String, 'health/status', self.on_health, latched)
        self.create_subscription(WaypointPath, 'navigation/waypoints', self.on_route, latched)
        self.create_subscription(BatteryState, 'battery', self.on_battery, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self.on_diagnostics, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        if camera_topic:
            kind = CompressedImage if compressed else Image
            self.create_subscription(kind, camera_topic, self.on_camera, qos_profile_sensor_data)
        if lidar_topic:
            self.create_subscription(
                PointCloud2, lidar_topic, self.on_lidar, qos_profile_sensor_data
            )

    # ------------------------------------------------------------------ socket

    def send(self, data: bytes, addr=None):
        """Send one datagram to the client (or addr), dropping it when the socket is busy."""
        addr = addr or self._client
        if addr is None:
            return
        try:
            self._sock.sendto(data, addr)
        except (BlockingIOError, OSError):
            pass

    def send_json(self, message: dict, addr=None):
        """Send a JSON message to the client (or addr)."""
        self.send(protocol.encode_json(message), addr)

    def send_stream(self, stream: int, payload: bytes):
        """Send a chunked binary payload to the client."""
        self._frame_ids[stream] += 1
        for datagram in protocol.chunk(stream, self._frame_ids[stream], payload):
            self.send(datagram)

    def poll(self):
        """Drain the socket (bounded, so a flood cannot starve ROS) and expire a silent client."""
        for _ in range(256):
            try:
                data, addr = self._sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as error:
                self.get_logger().warn(f'teleop socket: {error}', throttle_duration_sec=5.0)
                break
            message = protocol.decode_json(data)
            if message is not None and message.get('t') == 'discover':
                self.reply_beacon(addr)  # no token needed, nothing sensitive
                continue
            if message is None or not self.authorized(message):
                continue
            try:
                self.handle_message(message, addr)
            except (AttributeError, KeyError, TypeError, ValueError) as error:
                self.get_logger().warn(
                    f'ignored malformed teleop message from {addr}: {error}',
                    throttle_duration_sec=5.0,
                )
        if self._client and time.monotonic() - self._client_seen > self._client_timeout:
            self.get_logger().warn(f'teleop client {self._client} timed out')
            self._client = None
            self._remote_pub.publish(Twist())

    def authorized(self, message: dict) -> bool:
        """Check the shared token (constant time) when one is configured."""
        if not self._token:
            return True
        return hmac.compare_digest(str(message.get('k', '')), self._token)

    def accept_client(self, addr, message: dict) -> bool:
        """
        Register addr as the client; another device cannot take over a live session.

        The same device may come back from a new port (app restart). A different device gets
        `busy` until the current client has been silent for client_timeout_sec.
        """
        current = self._client
        if current is not None and addr != current and addr[0] != current[0]:
            if time.monotonic() - self._client_seen <= self._client_timeout:
                self.send_json({'t': 'busy', 'msg': f'robot is driven from {current[0]}'}, addr)
                return False
        if addr != current:
            self.get_logger().info(f'teleop client {addr} ({message.get("name", "?")})')
            if current is not None:
                self._remote_pub.publish(Twist())
        self._client = addr
        return True

    def handle_message(self, message: dict, addr):
        """Dispatch one client message."""
        kind = message['t']
        if kind == 'hello':
            if not self.accept_client(addr, message):
                return
            self._client_name = str(message.get('name', ''))[:32]
            streams = message.get('streams', {})
            if not isinstance(streams, dict):
                streams = {}
            self.set_camera_fps(streams.get('camera_fps'))
            self._streams = {k: bool(streams.get(k, False)) for k in self._streams}
            self.send_json(
                {
                    't': 'welcome',
                    'commands': sorted(self._commands) + ['list_routes'],
                    'limits': self._limits,
                }
            )
            self.send_route()
        elif addr != self._client:
            return
        self._client_seen = time.monotonic()
        if kind == 'joy':
            self.on_joy(message)
        elif kind == 'cmd':
            self.on_command(message)
        elif kind == 'ping':
            self.send_json({'t': 'pong', 'ts': message.get('ts')})
        elif kind == 'streams':
            self._streams.update({k: bool(v) for k, v in message.items() if k in self._streams})
            self.set_camera_fps(message.get('camera_fps'))
        elif kind == 'bye':
            self._client = None
            self._remote_pub.publish(Twist())

    def on_joy(self, message: dict):
        """Publish a clamped teleop velocity; NaN, inf and non-numbers count as 0."""
        vx_max, vy_max, wz_max = self._limits
        twist = Twist()
        twist.linear.x = protocol.clamp_axis(message.get('vx'), vx_max)
        twist.linear.y = protocol.clamp_axis(message.get('vy'), vy_max)
        twist.angular.z = protocol.clamp_axis(message.get('wz'), wz_max)
        self._remote_pub.publish(twist)

    def on_command(self, message: dict):
        """Run a named command once per id and acknowledge it."""
        cid, name, args = message.get('id'), message.get('name', ''), message.get('args', {})
        if not isinstance(cid, (int, str)) or isinstance(cid, bool):
            return
        if not isinstance(args, dict):
            args = {}
        if cid in self._acks:
            self.send_json(self._acks[cid])
            return
        if name == 'list_routes':
            self.ack(cid, True, '', self.list_routes())
            return
        if name == 'get_route':
            self.send_route()
            self.ack(cid, True, f'{len(self._route)} points')
            return
        if name not in self._commands:
            self.ack(cid, False, f'unknown command {name}')
            return
        _, client, make_request = self._commands[name]
        if not client.service_is_ready():
            self.ack(cid, False, f'{client.srv_name} not available')
            return
        try:
            request = make_request(args)
        except (TypeError, ValueError) as error:
            self.ack(cid, False, str(error))
            return
        future = client.call_async(request)
        future.add_done_callback(lambda f, c=cid, n=name: self.on_command_done(c, n, f))
        self._acks[cid] = {'t': 'ack', 'id': cid, 'ok': None, 'msg': 'pending'}

    def on_command_done(self, cid, name: str, future):
        """Acknowledge a finished service call."""
        try:
            result = future.result()
        except Exception as error:  # noqa: B902  the service raised, report it to the client
            self.ack(cid, False, f'{name} failed: {error}')
            return
        ok = bool(result and getattr(result, 'success', True))
        if ok and name in ('record_start', 'record_stop'):
            self._state['recording'] = name == 'record_start'
        self.ack(cid, ok, getattr(result, 'message', '') if result else 'no response')

    def ack(self, cid, ok: bool, msg: str, data=None):
        """Send and cache an acknowledgement."""
        reply = {'t': 'ack', 'id': cid, 'ok': ok, 'msg': msg}
        if data is not None:
            reply['data'] = data
        self._acks[cid] = reply
        if len(self._acks) > 200:
            self._acks.pop(next(iter(self._acks)))
        self.send_json(reply)

    def list_routes(self) -> list:
        """Return recorded route files under routes_dir, newest first."""
        found = []
        for root, _, files in os.walk(self._routes_dir):
            for name in files:
                if name.endswith('.csv'):
                    path = os.path.join(root, name)
                    try:
                        found.append((os.path.getmtime(path), path))
                    except OSError:  # deleted while listing
                        continue
        return [path for _, path in sorted(found, reverse=True)[:50]]

    def route_path(self, file) -> str:
        """Resolve a route file from the client; only files under routes_dir may be loaded."""
        root = os.path.realpath(self._routes_dir)
        path = os.path.realpath(os.path.join(root, str(file)))
        if not str(file) or os.path.commonpath([root, path]) != root:
            raise ValueError(f'route must be a file under {self._routes_dir}')
        return path

    # ------------------------------------------------------------------ telemetry

    def on_nav_status(self, msg: NavigationStatus):
        """Cache the navigation state."""
        self._state['nav'], self._state['nav_msg'] = msg.state, msg.message

    def on_mission(self, msg: MissionStatus):
        """Cache the mission summary."""
        self._state['mission'] = msg.active_behavior

    def on_localization(self, msg: String):
        """Cache the localization source."""
        self._state['localization'] = msg.data

    def on_mode(self, msg: String):
        """Cache the control mode."""
        self._state['mode'] = msg.data

    def on_health(self, msg: String):
        """Cache ok + issues of the health summary."""
        try:
            health = json.loads(msg.data)
        except ValueError:
            return
        self._state['health'] = {
            'ok': bool(health.get('ok')),
            'issues': health.get('issues', [])[:5],
        }

    def on_battery(self, msg: BatteryState):
        """Cache battery readings."""
        self._state['battery'] = {
            'pct': finite(msg.percentage * 100.0, 0),
            'v': finite(msg.voltage, 1),
            'a': finite(msg.current, 1),
            'temp': finite(msg.temperature, 0),
        }

    def on_diagnostics(self, msg: DiagnosticArray):
        """Cache the driver status line."""
        if msg.status:
            status = msg.status[0]
            self._state['driver'] = {'level': int(status.level), 'msg': status.message}

    def on_cmd_vel(self, msg: Twist):
        """Cache the command actually sent to the robot."""
        self._state['cmd'] = [
            round(msg.linear.x, 2),
            round(msg.linear.y, 2),
            round(msg.angular.z, 2),
        ]

    def on_route(self, msg: WaypointPath):
        """Cache the loaded route and push it to the client."""
        self._route = protocol.simplify_route(
            [[round(w.x, 2), round(w.y, 2), round(w.dwell_sec, 1)] for w in msg.waypoints], 600
        )
        self.send_route()

    def send_route(self):
        """Send the loaded route as a chunked JSON payload."""
        if self._client is not None:
            payload = protocol.encode_json({'t': 'route', 'points': self._route})
            self.send_stream(protocol.STREAM_ROUTE, payload)

    def pose(self):
        """Return the robot [x, y, yaw] in the map frame, or None."""
        try:
            t = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None
        p = t.transform.translation
        return [round(p.x, 3), round(p.y, 3), round(yaw_of(t.transform.rotation), 3)]

    def send_state(self):
        """Send the telemetry snapshot; seq lets the client measure packet loss."""
        if self._client is not None:
            self._seq += 1
            self.send_json({'t': 'state', 'seq': self._seq, 'pose': self.pose(), **self._state})

    def reply_beacon(self, addr):
        """Answer a LAN discovery probe (no token needed, nothing sensitive)."""
        alive = (
            self._client is not None
            and time.monotonic() - self._client_seen < self._client_timeout
        )
        beacon = {
            't': 'beacon',
            'name': self._robot_name,
            'secured': bool(self._token),
            'in_use_by': self._client_name if alive else '',
            'mode': self._state.get('mode'),
            'battery': (self._state.get('battery') or {}).get('pct'),
        }
        try:
            self._sock.sendto(protocol.encode_json(beacon), addr)
        except OSError:
            pass

    def set_camera_fps(self, fps):
        """Let the handheld lower or restore the camera rate, capped by camera_fps."""
        if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not math.isfinite(fps):
            return  # absent, text or NaN (json accepts NaN): keep the current rate
        limit = max(float(self.get_parameter('camera_fps').value), 0.1)
        self._camera_period = 1.0 / min(max(float(fps), 0.5), limit)

    # ------------------------------------------------------------------ streams

    def on_camera(self, msg):
        """Forward a throttled JPEG frame."""
        now = time.monotonic()
        if not self._streams['camera'] or self._client is None:
            return
        if now - self._last_camera < self._camera_period:
            return
        self._last_camera = now
        jpeg = bytes(msg.data) if isinstance(msg, CompressedImage) else self.encode(msg)
        if jpeg:
            self.send_stream(protocol.STREAM_CAMERA, jpeg)

    def encode(self, msg: Image) -> bytes:
        """JPEG-encode a raw rgb8/bgr8 image, downscaled to camera_max_width."""
        if cv2 is None or msg.encoding not in ('rgb8', 'bgr8'):
            return b''
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
        img = img[:, : msg.width]
        if msg.encoding == 'rgb8':
            img = img[:, :, ::-1]
        max_width = int(self.get_parameter('camera_max_width').value)
        if msg.width > max_width:
            img = cv2.resize(img, (max_width, int(msg.height * max_width / msg.width)))
        quality = int(self.get_parameter('camera_jpeg_quality').value)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else b''

    def on_lidar(self, msg: PointCloud2):
        """Forward a throttled, voxel-downsampled cloud in the robot frame."""
        now = time.monotonic()
        if not self._streams['lidar'] or self._client is None:
            return
        if now - self._last_lidar < self._lidar_period:
            return
        self._last_lidar = now
        extrinsic = self.extrinsic(msg.header.frame_id)
        if extrinsic is None:
            return
        offsets = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= offsets.keys():
            return
        dtype = np.dtype(
            {
                'names': ['x', 'y', 'z'],
                'formats': ['<f4'] * 3,
                'offsets': [offsets['x'], offsets['y'], offsets['z']],
                'itemsize': msg.point_step,
            }
        )
        cloud = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        xyz = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        rotation, translation = extrinsic
        xyz = xyz @ rotation.T + translation
        xyz = protocol.voxel_downsample(
            xyz,
            float(self.get_parameter('lidar_voxel').value),
            int(self.get_parameter('lidar_max_points').value),
        )
        self.send_stream(protocol.STREAM_LIDAR, protocol.pack_points(xyz))

    def extrinsic(self, frame_id: str):
        """Return the cached (rotation, translation) from frame_id to base_frame, or None."""
        if frame_id not in self._extrinsics:
            try:
                t = self._tf_buffer.lookup_transform(self._base_frame, frame_id, Time())
            except TransformException:
                return None
            q, v = t.transform.rotation, t.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ],
                dtype=np.float32,
            )
            self._extrinsics[frame_id] = (rotation, np.array([v.x, v.y, v.z], np.float32))
        return self._extrinsics[frame_id]


def main(args=None):
    """Spin the teleop UDP node."""
    rclpy.init(args=args)
    node = TeleopUdpNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.send_json({'t': 'bye'})
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
