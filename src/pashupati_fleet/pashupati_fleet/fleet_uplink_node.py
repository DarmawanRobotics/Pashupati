"""
fleet_uplink_node: the robot side of the NETRA command center (API.md).

Telemetry, patrol, stop-point and alert events over one WebSocket; anomalies and taught routes
through an on-disk spool uploaded over HTTP; NETRA's commands become the stack's services.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import faulthandler
import os
import signal
import time

from geometry_msgs.msg import PoseArray
from pashupati_utils.utils.calls import call_service
from pashupati_utils.utils.params import set_parameters_request
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dharma_interfaces.srv import LoadPath
from dharma_utils.io import expand, valid_name
from dharma_utils.math import yaw_of
from dharma_utils.ros import EVENTS, SENSOR, DharmaNode, run
from pashupati_fleet.core.anomalies import Anomalies, stamp_key
from pashupati_fleet.core.config import FleetConfig
from pashupati_fleet.core.crowd import CrowdAggregator
from pashupati_fleet.core.patrol import Patrol
from pashupati_fleet.core.poses import PoseHistory
from pashupati_fleet.core.routes import safe_route_id, summarize_csv
from pashupati_fleet.core.spool import Spool
from pashupati_fleet.core.switches import SWITCHES, live_parameters
from pashupati_fleet.utils.connection import NetraLink
from pashupati_fleet.utils.telemetry import FleetTelemetry

# NETRA command -> std_srvs/Trigger service of the stack
TRIGGERS = {
    'stop_patrol': 'navigation/stop',
    'pause': 'navigation/pause',
    'resume': 'navigation/resume',
    'localize': 'localization/start',
    'estop': 'control/estop/engage',
    'estop_release': 'control/estop/release',
    'dock': 'docking/dock',
    'undock': 'docking/undock',
}
ROUTES_SUBDIR = 'netra'  # <sites_dir>/<site>/netra/<route_id>.csv


class FleetUplinkNode(DharmaNode):
    """Connects the robot to NETRA: telemetry, events, crowd, anomalies, routes, commands."""

    def __init__(self, **kwargs):
        super().__init__('fleet_uplink_node', **kwargs)
        self.config = self.declare_config(FleetConfig)
        self.robot = self.declare_robot()
        cfg = self.config.value
        self.spool = Spool(cfg.spool_dir)
        self.crowd = CrowdAggregator(cfg.crowd_cell_size, cfg.crowd_window_sec)
        self.anomalies = Anomalies(cfg.anomaly_min_confidence)
        self.poses = PoseHistory()
        self.patrol = Patrol()
        self.link = NetraLink(cfg.server_url, cfg.token, cfg.robot_id, cfg.verify_tls,
                              self.spool, self.log)  # fmt: skip
        self.io = ThreadPoolExecutor(max_workers=1)
        self.estop_pub = self.create_publisher(String, 'control/estop_request', EVENTS)
        self.triggers = {name: self.create_client(Trigger, s) for name, s in TRIGGERS.items()}
        self.load_client = self.create_client(LoadPath, 'navigation/load_path')
        self.start_client = self.create_client(Trigger, 'navigation/start')
        self.switches = {c: self.create_client(SetParameters, f'{n}/set_parameters')
                         for c, n in SWITCHES.items()}  # fmt: skip
        self.telemetry = FleetTelemetry(self, cfg.health_diagnostics_topic)
        sub = self.create_subscription
        sub(PoseArray, 'perception/people', self.on_people, 10)
        sub(String, 'route/saved', self.on_route_saved, EVENTS)
        if cfg.anomaly_topic:
            sub(String, cfg.anomaly_topic, self.on_anomaly, EVENTS)
        if cfg.anomaly_image_topic:
            sub(CompressedImage, cfg.anomaly_image_topic, self.on_image, SENSOR)
        if cfg.anomaly_evidence_topic:
            sub(CompressedImage, cfg.anomaly_evidence_topic, self.on_evidence, EVENTS)
        self.every(1.0 / cfg.telemetry_hz, self.send_telemetry)
        self.every(0.1, self.drain_commands)
        self.every(5.0, self.flush_crowd)
        self.every(0.25, self.flush_anomalies)
        if cfg.server_url:
            self.link.start(cfg.upload_period_sec)
            self.log.info('command center', url=cfg.server_url, robot_id=cfg.robot_id)
        else:
            self.log.warning('server_url empty: spooling uploads, not connecting')

    def destroy_node(self):
        """Stop the worker threads."""
        self.link.stop()
        self.io.shutdown(wait=False)
        return super().destroy_node()

    # ---------------------------------------------------------------- state
    def send(self, message: dict, durable: bool = False) -> bool:
        """Send to NETRA (durable messages wait while offline)."""
        return self.link.send(message, durable)

    def pose(self):
        """Return {x, y, yaw} in the map frame, or None."""
        frames = self.robot.frames
        found = self.tf.lookup(frames.map, frames.base)
        if found is None:
            return None
        m = found.matrix
        return {'x': round(float(m[0, 3]), 3), 'y': round(float(m[1, 3]), 3),
                'yaw': round(yaw_of(m), 3)}  # fmt: skip

    def send_telemetry(self):
        """Send the snapshot (telemetry_hz) and remember where the robot was."""
        pose, now = self.pose(), time.time()
        self.poses.add(now, pose)
        self.send({'type': 'telemetry', 'pose': pose, 'patrol': self.patrol.current,
                   **self.patrol.context(), **self.telemetry.state})  # fmt: skip

    def on_people(self, msg: PoseArray):
        """Feed people positions into the crowd heatmap."""
        stamp = self.telemetry.people(msg)
        self.crowd.add(stamp, [(p.position.x, p.position.y) for p in msg.poses])

    def flush_crowd(self):
        """Send a finished crowd window."""
        snapshot = self.crowd.flush(time.time())
        if snapshot:
            self.send({'type': 'crowd', **snapshot, **self.patrol.context()}, durable=True)

    # ---------------------------------------------------------------- uploads
    def on_image(self, msg: CompressedImage):
        """Keep the latest camera frame, the fallback evidence."""
        self.anomalies.latest = (time.time(), bytes(msg.data))

    def on_evidence(self, msg: CompressedImage):
        """Keep the frames the detector analysed, by their camera stamp."""
        key = stamp_key(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.anomalies.evidence.append((key, bytes(msg.data)))
        self.flush_anomalies()

    def on_anomaly(self, msg: String):
        """Queue a finding until its frame arrived (or a second passed)."""
        context, pose = self.patrol.context(), self.pose()
        stop = self.patrol.stop_index
        if self.anomalies.finding(msg.data, context, pose, stop, self.poses, time.time()):
            self.flush_anomalies()

    def flush_anomalies(self):
        """Spool findings with their evidence frame."""
        ready, dropped = self.anomalies.ready(time.time())
        for meta, jpeg in ready:
            uid = self.spool.put('anomaly', meta, {'image.jpg': jpeg})
            self.log.info('anomaly spooled', category=meta['category'], uid=uid)
        if dropped:
            self.log.warning('anomaly without a recent camera frame, skipped', count=dropped)

    def on_route_saved(self, msg: String):
        """Spool a freshly taught route for NETRA's route database."""
        try:
            with open(msg.data, 'rb') as f:
                data = f.read()
            recorded_at = os.path.getmtime(msg.data)
        except OSError as error:
            self.log.error('cannot read the taught route', path=msg.data, error=str(error))
            return
        meta = {
            'name': os.path.splitext(os.path.basename(msg.data))[0],
            'recorded_at': recorded_at,
            'source_path': msg.data,
            'site': self.telemetry.site,
            **summarize_csv(data.decode(errors='ignore')),
        }
        self.spool.put('route', meta, {'route.csv': data})

    # ---------------------------------------------------------------- commands
    def ack(self, cid, ok: bool, msg: str):
        """Acknowledge a command."""
        self.send({'type': 'ack', 'id': cid, 'ok': ok, 'msg': msg}, durable=True)

    def drain_commands(self):
        """Run NETRA's commands on the ROS thread."""
        while not self.link.incoming.empty():
            self.run_command(self.link.incoming.get())

    def run_command(self, message: dict):
        """Dispatch one command."""
        cid, name = message.get('id'), message.get('name')
        args = message.get('args') if isinstance(message.get('args'), dict) else {}
        if name == 'start_patrol':
            self.start_patrol(cid, args)
        elif name == '_route_ready':
            self.call(self.load_client, LoadPath.Request(path=args['path']), cid,
                      then=lambda: self.start_navigation(cid, args))  # fmt: skip
        elif name in TRIGGERS:
            if name == 'estop':  # latch first, without waiting for the service
                self.estop_pub.publish(String(data='command center'))
            self.call(self.triggers[name], Trigger.Request(), cid)
        elif name in self.switches:
            try:
                request = set_parameters_request(live_parameters(name, args))
            except ValueError as error:
                self.ack(cid, False, str(error))
                return
            self.call(self.switches[name], request, cid)
        else:
            self.ack(cid, False, f'unknown command {name}')

    def call(self, client, request, cid, then=None):
        """Call a service; ack a failure, hand a success to then or ack it."""

        def done(ok, message, _result):
            if ok and then is not None:
                then()
            else:
                self.ack(cid, ok, message)

        if not call_service(client, request, done):
            self.ack(cid, False, f'{client.srv_name} not available')

    def start_patrol(self, cid, args: dict):
        """Download the route into the active site (IO thread), then load and start it."""
        route_id, site = args.get('route_id'), self.telemetry.site
        if not safe_route_id(route_id):
            self.ack(cid, False, f'invalid route_id {route_id!r}')
            return
        if not valid_name(site):
            self.ack(cid, False, 'no active site: load one first (site/load)')
            return
        relative = os.path.join(site, ROUTES_SUBDIR, f'{route_id}.csv')
        path = os.path.join(expand(self.config.sites_dir), relative)
        self.io.submit(self.download, cid, args, path, relative)

    def download(self, cid, args: dict, path: str, relative: str):
        """Fetch the route CSV (IO thread), then continue on the ROS thread."""
        try:
            self.link.download_route(str(args['route_id']), path)
        except Exception as error:
            self.ack(cid, False, f'route download failed: {error}')
            return
        ready = {**args, 'path': relative}
        self.link.incoming.put({'type': 'command', 'id': cid, 'name': '_route_ready',
                                'args': ready})  # fmt: skip

    def start_navigation(self, cid, args: dict):
        """Start the loaded route and open the patrol."""

        def started():
            for event in self.patrol.start(args.get('patrol_id'), args.get('route_id')):
                self.send(event, durable=True)
            self.ack(cid, True, 'patrol started')

        self.call(self.start_client, Trigger.Request(), cid, then=started)


def main(args=None):
    """Run the NETRA uplink (SIGUSR1 prints the stack of every thread to stderr)."""
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    run(FleetUplinkNode, args=args)


if __name__ == '__main__':
    main()
