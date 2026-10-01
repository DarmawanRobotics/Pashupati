#!/usr/bin/env python3
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
import queue
import re
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_fleet.anomaly import parse_result
from robot_fleet.crowd import CrowdAggregator
from robot_fleet.routes import summarize_csv
from robot_fleet.spool import Spool
from robot_interfaces.msg import MissionStatus, NavigationStatus, StopPointEvent
from robot_interfaces.srv import LoadPath
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

ALERT_STATES = {'BLOCKED', 'OFF_PATH', 'TF_UNAVAILABLE', 'AVOIDANCE_STALE'}
LAP_RE = re.compile(r'lap (\d+)')


def finite(value, digits: int = 2):
    """Round a float for JSON, or None when it is NaN/inf."""
    return round(float(value), digits) if math.isfinite(value) else None


class FleetUplinkNode(Node):
    """Connects the robot to the command center: telemetry, events, crowd, anomalies, routes."""

    def __init__(self):
        """Declare params, start the WebSocket and upload workers, and wire ROS topics."""
        super().__init__('fleet_uplink_node')
        defaults = {
            'server_url': '',
            'token': '',
            'robot_id': 'robot-1',
            'verify_tls': True,
            'telemetry_hz': 1.0,
            'upload_period_sec': 5.0,
            'spool_dir': '~/.pashupati/spool',
            'routes_dir': '/home/robot/dev/Pashupati/map',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'crowd_cell_size': 1.0,
            'crowd_window_sec': 60.0,
            'anomaly_topic': '/anomaly_detector/result',
            'anomaly_image_topic': '/camera/camera/color/image_raw/compressed',
            'anomaly_min_confidence': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self._base_url = p('server_url').rstrip('/')
        self._token, self._robot_id = p('token'), p('robot_id')
        self._verify = bool(p('verify_tls'))
        self._routes_dir = p('routes_dir')
        self._map_frame, self._base_frame = p('map_frame'), p('base_frame')
        self._min_confidence = float(p('anomaly_min_confidence'))
        self._spool = Spool(p('spool_dir'))
        self._crowd = CrowdAggregator(float(p('crowd_cell_size')), float(p('crowd_window_sec')))

        self._state = {
            'mode': 'auto',
            'nav': {'state': 'NAV_INACTIVE', 'message': ''},
            'mission': '',
            'localization': 'none',
            'battery': None,
            'driver': None,
            'people': 0,
            'health': None,
        }
        self._patrol = None
        self._lap = 0
        self._stop_index = None
        self._last_image = (0.0, b'')

        self._incoming: queue.Queue = queue.Queue()
        self._outbox: deque = deque(maxlen=500)
        self._ws = None
        self._ws_lock = threading.Lock()
        self._running = True
        self._io = ThreadPoolExecutor(max_workers=1)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.make_clients()
        self.make_subscriptions(p('anomaly_topic'), p('anomaly_image_topic'))
        self.create_timer(1.0 / float(p('telemetry_hz')), self.send_telemetry)
        self.create_timer(0.1, self.drain_incoming)
        self.create_timer(5.0, self.flush_crowd)

        if self._base_url:
            threading.Thread(target=self.ws_loop, daemon=True).start()
            threading.Thread(
                target=self.upload_loop, args=(float(p('upload_period_sec')),), daemon=True
            ).start()
            self.get_logger().info(f'command center {self._base_url} as {self._robot_id}')
        else:
            self.get_logger().warn('server_url empty: spooling uploads, not connecting')

    # ------------------------------------------------------------------ ROS wiring

    def make_clients(self):
        """Create the service clients used by command-center commands."""
        self._load_path = self.create_client(LoadPath, 'navigation/load_path')
        self._start_nav = self.create_client(SetBool, 'navigation/start_nav')
        self._pause = self.create_client(SetBool, 'navigation/pause')
        self._localize = self.create_client(Trigger, 'localization/start')

    def make_subscriptions(self, anomaly_topic: str, image_topic: str):
        """Subscribe to state, events, people, routes and anomalies."""
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(NavigationStatus, 'navigation/status', self.on_nav_status, 10)
        self.create_subscription(MissionStatus, 'navigation/mission_status', self.on_mission, 10)
        self.create_subscription(StopPointEvent, 'navigation/stop_point_event', self.on_stop, 10)
        self.create_subscription(String, 'localization/status', self.on_localization, latched)
        self.create_subscription(String, 'bridge/mode', self.on_mode, latched)
        self.create_subscription(String, 'health/status', self.on_health, latched)
        self.create_subscription(BatteryState, 'battery', self.on_battery, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self.on_diagnostics, 10)
        self.create_subscription(PoseArray, 'perception/people', self.on_people, 10)
        self.create_subscription(String, 'mapping/route_saved', self.on_route_saved, 10)
        if anomaly_topic:
            self.create_subscription(String, anomaly_topic, self.on_anomaly, 10)
        if image_topic:
            self.create_subscription(
                CompressedImage, image_topic, self.on_image, qos_profile_sensor_data
            )

    # ------------------------------------------------------------------ connection

    def ws_url(self) -> str:
        """Return the WebSocket URL derived from server_url."""
        return re.sub(r'^http', 'ws', self._base_url) + '/api/v1/robot/ws'

    def headers(self) -> dict:
        """Return the auth headers for HTTP and WebSocket."""
        return {'Authorization': f'Bearer {self._token}', 'X-Robot-Id': self._robot_id}

    def ws_loop(self):
        """Keep a WebSocket to the command center open, reconnecting with backoff."""
        import ssl

        import websocket

        backoff = 1.0
        sslopt = None if self._verify else {'cert_reqs': ssl.CERT_NONE}
        while self._running:
            try:
                ws = websocket.create_connection(
                    self.ws_url(),
                    header=[f'{k}: {v}' for k, v in self.headers().items()],
                    timeout=10,
                    sslopt=sslopt,
                )
            except Exception as error:  # noqa: B902
                self.get_logger().warn(
                    f'command center unreachable: {error}', throttle_duration_sec=30.0
                )
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 30.0)
                continue
            backoff = 1.0
            ws.settimeout(30)
            with self._ws_lock:
                self._ws = ws
            self.get_logger().info('command center connected')
            self.send({'type': 'hello', 'robot_id': self._robot_id, 'version': 1})
            while self._outbox:
                self.send(self._outbox.popleft())
            self.ws_receive(ws, websocket.WebSocketTimeoutException)
            with self._ws_lock:
                self._ws = None
            try:
                ws.close()
            except Exception:  # noqa: B902
                pass
            self.get_logger().warn('command center disconnected')

    def ws_receive(self, ws, timeout_error):
        """Read server messages into the incoming queue until the socket closes."""
        while self._running:
            try:
                text = ws.recv()
            except timeout_error:
                continue
            except Exception:  # noqa: B902
                return
            if not text:
                return
            try:
                message = json.loads(text)
            except ValueError:
                continue
            if isinstance(message, dict) and message.get('type') == 'command':
                self._incoming.put(message)

    def send(self, message: dict, durable: bool = False) -> bool:
        """Send a message now; durable messages wait in the outbox while offline."""
        with self._ws_lock:
            ws = self._ws
            if ws is not None:
                try:
                    ws.send(json.dumps(message, separators=(',', ':')))
                    return True
                except Exception:  # noqa: B902
                    self._ws = None
        if durable:
            self._outbox.append(message)
        return False

    def upload_loop(self, period: float):
        """Upload spooled anomalies and routes; keep them on disk until the server accepts."""
        import requests

        urls = {'anomaly': '/api/v1/robot/anomalies', 'route': '/api/v1/robot/routes'}
        types = {'.jpg': 'image/jpeg', '.csv': 'text/csv'}
        while self._running:
            for item in self._spool.items():
                kind, meta, files = Spool.load(item)
                multipart = {
                    name: (
                        name,
                        data,
                        types.get(os.path.splitext(name)[1], 'application/octet-stream'),
                    )
                    for name, data in files.items()
                }
                try:
                    reply = requests.post(
                        self._base_url + urls[kind],
                        headers=self.headers(),
                        data={'meta': json.dumps(meta)},
                        files=multipart,
                        timeout=20,
                        verify=self._verify,
                    )
                except Exception as error:  # noqa: B902
                    self.get_logger().warn(f'upload failed: {error}', throttle_duration_sec=30.0)
                    break
                if reply.status_code < 300 or reply.status_code == 409:
                    Spool.remove(item)
                elif reply.status_code in (400, 413, 422):
                    self.get_logger().error(f'{kind} {meta.get("uid")} rejected: {reply.text}')
                    Spool.remove(item)
                else:
                    break
            time.sleep(period)

    # ------------------------------------------------------------------ state

    def context(self) -> dict:
        """Return the patrol context attached to events."""
        return {
            'patrol_id': (self._patrol or {}).get('patrol_id'),
            'route_id': (self._patrol or {}).get('route_id'),
            'ts': time.time(),
        }

    def pose(self):
        """Return {x, y, yaw} in the map frame, or None."""
        try:
            t = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None
        q, v = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {'x': round(v.x, 3), 'y': round(v.y, 3), 'yaw': round(yaw, 3)}

    def send_telemetry(self):
        """Send the 1 Hz robot snapshot."""
        self.send(
            {
                'type': 'telemetry',
                'pose': self.pose(),
                'patrol': self._patrol,
                **self.context(),
                **self._state,
            }
        )

    def on_nav_status(self, msg: NavigationStatus):
        """Track navigation state, raising alerts and finishing patrols."""
        previous = self._state['nav']['state']
        self._state['nav'] = {'state': msg.state, 'message': msg.message}
        if msg.state == previous:
            return
        if msg.state in ALERT_STATES:
            self.send(
                {
                    'type': 'alert',
                    'state': msg.state,
                    'message': msg.message,
                    'pose': self.pose(),
                    **self.context(),
                },
                durable=True,
            )
        if self._patrol and msg.state in ('GOAL_REACHED', 'NAV_INACTIVE'):
            self.patrol_event('finished', reason=msg.state.lower())
            self._patrol = None

    def on_mission(self, msg: MissionStatus):
        """Track the mission summary and report completed laps."""
        self._state['mission'] = msg.active_behavior
        match = LAP_RE.search(msg.active_behavior)
        lap = int(match.group(1)) if match else 0
        if self._patrol and lap > self._lap:
            self.patrol_event('lap', lap=lap)
        self._lap = lap

    def on_stop(self, msg: StopPointEvent):
        """Forward stop-point events and remember the current stop."""
        self._stop_index = int(msg.index) if msg.event in ('arrived', 'inspected') else None
        self.send(
            {
                'type': 'stop_event',
                'index': int(msg.index),
                'event': msg.event,
                'message': msg.message,
                'x': msg.x,
                'y': msg.y,
                'yaw_deg': msg.yaw_deg,
                **self.context(),
            },
            durable=True,
        )

    def on_localization(self, msg: String):
        """Cache the localization source."""
        self._state['localization'] = msg.data

    def on_mode(self, msg: String):
        """Cache the control mode."""
        self._state['mode'] = msg.data

    def on_health(self, msg: String):
        """Cache the health summary (ok, issues, system figures)."""
        try:
            self._state['health'] = json.loads(msg.data)
        except ValueError:
            pass

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
            self._state['driver'] = {
                'level': int(msg.status[0].level),
                'msg': msg.status[0].message,
            }

    def on_people(self, msg: PoseArray):
        """Feed people positions into the crowd aggregator."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 or time.time()
        self._crowd.add(stamp, [(p.position.x, p.position.y) for p in msg.poses])
        self._state['people'] = len(msg.poses)

    def flush_crowd(self):
        """Send a finished crowd window."""
        snapshot = self._crowd.flush(time.time())
        if snapshot:
            self.send({'type': 'crowd', **snapshot, **self.context()}, durable=True)

    def on_image(self, msg: CompressedImage):
        """Keep the latest camera frame for anomaly snapshots."""
        self._last_image = (time.time(), bytes(msg.data))

    def on_anomaly(self, msg: String):
        """Spool an anomaly with the latest frame and the robot pose."""
        result = parse_result(msg.data)
        if result is None or result['confidence'] < self._min_confidence:
            return
        stamp, jpeg = self._last_image
        if not jpeg or time.time() - stamp > 3.0:
            self.get_logger().warn('anomaly without a recent camera frame, skipped')
            return
        meta = {
            **result,
            **self.context(),
            'pose': self.pose(),
            'stop_index': self._stop_index,
            'detector': 'robot',
        }
        uid = self._spool.put('anomaly', meta, {'image.jpg': jpeg})
        self.get_logger().info(f'anomaly {result["category"]} spooled ({uid})')

    def on_route_saved(self, msg: String):
        """Spool a freshly recorded route for the route database."""
        try:
            with open(msg.data, 'rb') as f:
                data = f.read()
        except OSError as error:
            self.get_logger().error(f'cannot read route {msg.data}: {error}')
            return
        meta = {
            'name': os.path.splitext(os.path.basename(msg.data))[0],
            'recorded_at': os.path.getmtime(msg.data),
            'source_path': msg.data,
            **summarize_csv(data.decode(errors='ignore')),
        }
        self._spool.put('route', meta, {'route.csv': data})

    # ------------------------------------------------------------------ commands

    def patrol_event(self, event: str, **extra):
        """Report a patrol lifecycle event."""
        self.send(
            {'type': 'patrol_event', 'event': event, **extra, **self.context()}, durable=True
        )

    def ack(self, cid, ok: bool, msg: str):
        """Acknowledge a command."""
        self.send({'type': 'ack', 'id': cid, 'ok': ok, 'msg': msg}, durable=True)

    def drain_incoming(self):
        """Run queued command-center commands on the ROS thread."""
        while not self._incoming.empty():
            self.run_command(self._incoming.get())

    def run_command(self, message: dict):
        """Dispatch one command."""
        cid, name, args = message.get('id'), message.get('name'), message.get('args') or {}
        if name == 'start_patrol':
            self._io.submit(self.download_route, cid, args)
        elif name == '_route_ready':
            self.load_and_start(cid, args)
        elif name == 'stop_patrol':
            self.call(self._start_nav, SetBool.Request(data=False), cid)
        elif name in ('pause', 'resume'):
            self.call(self._pause, SetBool.Request(data=name == 'pause'), cid)
        elif name == 'localize':
            self.call(self._localize, Trigger.Request(), cid)
        else:
            self.ack(cid, False, f'unknown command {name}')

    def call(self, client, request, cid, then=None):
        """Call a service asynchronously; ack failures, hand successes to then or ack them."""
        if not client.service_is_ready():
            self.ack(cid, False, f'{client.srv_name} not available')
            return

        def done(future):
            result = future.result()
            ok = bool(result and getattr(result, 'success', True))
            text = getattr(result, 'message', '') if result else 'no response'
            if ok and then:
                then()
            else:
                self.ack(cid, ok, text)

        client.call_async(request).add_done_callback(done)

    def download_route(self, cid, args: dict):
        """Fetch the patrol route CSV (IO thread), then continue on the ROS thread."""
        import requests

        route_id = args.get('route_id')
        path = os.path.join(self._routes_dir, 'command_center', f'{route_id}.csv')
        try:
            reply = requests.get(
                f'{self._base_url}/api/v1/robot/routes/{route_id}/file',
                headers=self.headers(),
                timeout=20,
                verify=self._verify,
            )
            reply.raise_for_status()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'wb') as f:
                f.write(reply.content)
        except Exception as error:  # noqa: B902
            self.ack(cid, False, f'route download failed: {error}')
            return
        self._incoming.put(
            {'type': 'command', 'id': cid, 'name': '_route_ready', 'args': {**args, 'path': path}}
        )

    def load_and_start(self, cid, args: dict):
        """Load the downloaded route, start navigation and open the patrol."""

        def started():
            self._patrol = {'patrol_id': args.get('patrol_id'), 'route_id': args.get('route_id')}
            self._lap = 0
            self.patrol_event('started')
            self.ack(cid, True, 'patrol started')

        def loaded():
            self.call(self._start_nav, SetBool.Request(data=True), cid, then=started)

        self.call(self._load_path, LoadPath.Request(waypoints_file=args['path']), cid, then=loaded)

    def shutdown(self):
        """Stop the worker threads."""
        self._running = False
        self._io.shutdown(wait=False)


def main(args=None):
    """Spin the fleet uplink node."""
    rclpy.init(args=args)
    node = FleetUplinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
