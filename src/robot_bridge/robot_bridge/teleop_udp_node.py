#!/usr/bin/env python3
import json
import math
import os
import socket
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
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


class TeleopUdpNode(Node):
    """UDP gateway for the handheld teleop: joystick, commands, telemetry, camera and lidar."""

    def __init__(self):
        """Declare params, open the socket and wire ROS interfaces."""
        super().__init__('teleop_udp_node')
        defaults = {
            'port': protocol.PORT,
            'token': '',
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
        self._sock.bind(('0.0.0.0', int(p('port'))))
        self._sock.setblocking(False)

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
        record = self.create_client(SetBool, 'mapping/path_record')
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
                self.create_client(MarkStopPoint, 'mapping/mark_stop_point'),
                lambda a: MarkStopPoint.Request(dwell_sec=float(a.get('dwell', 10.0))),
            ),
            'load_route': (
                LoadPath,
                self.create_client(LoadPath, 'navigation/load_path'),
                lambda a: LoadPath.Request(waypoints_file=str(a.get('file', ''))),
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

    def send(self, data: bytes):
        """Send one datagram to the client, dropping it when the socket is busy."""
        if self._client is None:
            return
        try:
            self._sock.sendto(data, self._client)
        except (BlockingIOError, OSError):
            pass

    def send_json(self, message: dict):
        """Send a JSON message to the client."""
        self.send(protocol.encode_json(message))

    def send_stream(self, stream: int, payload: bytes):
        """Send a chunked binary payload to the client."""
        self._frame_ids[stream] += 1
        for datagram in protocol.chunk(stream, self._frame_ids[stream], payload):
            self.send(datagram)

    def poll(self):
        """Drain the socket and expire a silent client."""
        while True:
            try:
                data, addr = self._sock.recvfrom(65535)
            except BlockingIOError:
                break
            message = protocol.decode_json(data)
            if message is None or (self._token and message.get('k') != self._token):
                continue
            self.handle(message, addr)
        if self._client and time.monotonic() - self._client_seen > self._client_timeout:
            self.get_logger().warn(f'teleop client {self._client} timed out')
            self._client = None
            self._remote_pub.publish(Twist())

    def handle(self, message: dict, addr):
        """Dispatch one client message."""
        kind = message['t']
        if kind == 'hello':
            if addr != self._client:
                self.get_logger().info(f'teleop client {addr} ({message.get("name", "?")})')
            self._client = addr
            streams = message.get('streams', {})
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
        elif kind == 'bye':
            self._client = None
            self._remote_pub.publish(Twist())

    def on_joy(self, message: dict):
        """Publish a clamped teleop velocity."""
        vx_max, vy_max, wz_max = self._limits
        twist = Twist()
        twist.linear.x = max(-vx_max, min(vx_max, float(message.get('vx', 0.0))))
        twist.linear.y = max(-vy_max, min(vy_max, float(message.get('vy', 0.0))))
        twist.angular.z = max(-wz_max, min(wz_max, float(message.get('wz', 0.0))))
        self._remote_pub.publish(twist)

    def on_command(self, message: dict):
        """Run a named command once per id and acknowledge it."""
        cid, name, args = message.get('id'), message.get('name', ''), message.get('args', {})
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
        future = client.call_async(make_request(args if isinstance(args, dict) else {}))
        future.add_done_callback(lambda f, c=cid, n=name: self.on_command_done(c, n, f))
        self._acks[cid] = {'t': 'ack', 'id': cid, 'ok': None, 'msg': 'pending'}

    def on_command_done(self, cid, name: str, future):
        """Acknowledge a finished service call."""
        result = future.result()
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
            found += [os.path.join(root, f) for f in files if f.endswith('.csv')]
        return sorted(found, key=os.path.getmtime, reverse=True)[:50]

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
        """Send the telemetry snapshot."""
        if self._client is not None:
            self.send_json({'t': 'state', 'pose': self.pose(), **self._state})

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
    except KeyboardInterrupt:
        pass
    finally:
        node.send_json({'t': 'bye'})
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
