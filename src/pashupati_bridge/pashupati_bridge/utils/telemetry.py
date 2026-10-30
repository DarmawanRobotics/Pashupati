"""What the handheld's `state` message carries, kept up to date from the stack's topics."""

from __future__ import annotations

import json
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from pashupati_utils.core.numbers import finite
from pashupati_utils.utils import status
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from dharma_interfaces.msg import (
    ControlStatus,
    HealthStatus,
    LocalizationStatus,
    NavigationStatus,
    WaypointPath,
)
from dharma_utils.ros import COMMAND, LATCHED
from pashupati_bridge.core import protocol
from pashupati_bridge.core.summaries import (
    handheld_health,
    inspection_summary,
    mission_text,
    people_summary,
)

DETECTOR_STALE_SEC = 6.0  # anomaly_detector/status and people_status come every 2 s
ROUTE_POINTS = 600  # the route sent to the handheld, at most


def level_of(diagnostic) -> int:
    """Return a DiagnosticStatus level as int (rclpy gives the byte field as bytes)."""
    level = diagnostic.level
    return level[0] if isinstance(level, bytes) else int(level)


class Telemetry:
    """Subscribes to the status topics and keeps the handheld's view of the robot."""

    def __init__(self, node, health_diagnostics: str, on_route):
        self.on_route = on_route  # called with the new route points
        self.state = {
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
            'estop': False,
            'field': None,
        }
        self.route: list = []
        self.findings: list = []
        self.detectors = {'inspection': (None, 0.0), 'people': (None, 0.0)}
        sub = node.create_subscription
        sub(ControlStatus, 'control/status', self.on_control, LATCHED)
        sub(NavigationStatus, 'navigation/status', self.on_nav, 10)
        sub(LocalizationStatus, 'localization/status', self.on_localization, LATCHED)
        sub(HealthStatus, 'health/status', self.on_health, LATCHED)
        sub(DiagnosticArray, health_diagnostics, self.on_findings, 10)
        sub(
            String, 'anomaly_detector/status', lambda m: self.on_detector('inspection', m), LATCHED
        )
        sub(String, 'perception/people_status', lambda m: self.on_detector('people', m), LATCHED)
        sub(WaypointPath, 'navigation/waypoints', self.on_waypoints, LATCHED)
        sub(BatteryState, 'battery', self.on_battery, 10)
        sub(DiagnosticArray, 'diagnostics', self.on_driver, 10)
        sub(Twist, 'cmd_vel', self.on_cmd_vel, COMMAND)

    def on_control(self, msg: ControlStatus):
        """Mode, e-stop and protective field of the control mux."""
        self.state.update(
            mode=status.mode_name(msg), estop=bool(msg.estop), field=status.field_name(msg)
        )

    def on_nav(self, msg: NavigationStatus):
        """Keep the follower's state, message and a line about the route."""
        name = status.nav_state_name(msg)
        reasons = list(msg.pause_reasons)
        self.state.update(
            nav=name,
            nav_msg=msg.message,
            mission=mission_text(name, reasons, int(msg.index), int(msg.count)),
        )

    def on_localization(self, msg: LocalizationStatus):
        """'none' or the source of the last correction."""
        self.state['localization'] = status.localization_text(msg)

    def on_findings(self, msg: DiagnosticArray):
        """Keep the health monitor's findings (the computer's figures)."""
        self.findings = status.findings(msg)

    def on_health(self, msg: HealthStatus):
        """ok, issues and the computer's load."""
        self.state['health'] = handheld_health(status.health_summary(msg, self.findings))

    def on_detector(self, key: str, msg: String):
        """Keep the status JSON of the anomaly or person detector."""
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict):
            self.detectors[key] = (data, time.monotonic())

    def detector(self, key: str, summarize):
        """Return the summary of a detector, None when none ever ran."""
        data, seen = self.detectors[key]
        if data is None:
            return None
        return summarize(data, time.monotonic() - seen > DETECTOR_STALE_SEC, time.time())

    def inspection(self):
        """Return the inspection part of the state, None without a detector."""
        return self.detector('inspection', inspection_summary)

    def people(self):
        """Return the people part of the state, None without a detector."""
        return self.detector('people', people_summary)

    def inspection_list(self, name: str):
        """Return (ok, message, data) for inspection_models / inspection_categories."""
        data, seen = self.detectors['inspection']
        if data is None or time.monotonic() - seen > DETECTOR_STALE_SEC:
            return False, 'inspection not running (INSPECTION=true in pashupati.env)', None
        listed = data.get(name.split('_')[1])
        if not isinstance(listed, list):
            return False, 'update the robot: its inspection has no own types yet', None
        return True, '', [m for m in listed if isinstance(m, dict)][:30]

    def on_waypoints(self, msg: WaypointPath):
        """Keep the loaded route and push it to the handheld."""
        points = [[round(w.x, 2), round(w.y, 2), round(w.dwell_sec, 1)] for w in msg.waypoints]
        self.route = protocol.simplify_route(points, ROUTE_POINTS)
        self.on_route()

    def on_battery(self, msg: BatteryState):
        """Battery readings."""
        self.state['battery'] = {
            'pct': finite(msg.percentage * 100.0, 0),
            'v': finite(msg.voltage, 1),
            'a': finite(msg.current, 1),
            'temp': finite(msg.temperature, 0),
        }

    def on_driver(self, msg: DiagnosticArray):
        """Keep the robot driver's status line."""
        if msg.status:
            first = msg.status[0]
            self.state['driver'] = {'level': level_of(first), 'msg': first.message}

    def on_cmd_vel(self, msg: Twist):
        """Keep the command the control mux sends to the robot."""
        self.state['cmd'] = [round(msg.linear.x, 2), round(msg.linear.y, 2),
                             round(msg.angular.z, 2)]  # fmt: skip
