"""The robot as NETRA sees it (API.md telemetry), kept up to date from the stack's topics."""

from __future__ import annotations

import contextlib
import json
import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseArray
from pashupati_utils.core.numbers import finite
from pashupati_utils.utils import status
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from dharma_interfaces.msg import (
    ControlStatus,
    HealthStatus,
    LocalizationStatus,
    NavigationStatus,
    SiteStatus,
    StopPointEvent,
)
from dharma_utils.ros import EVENTS, LATCHED
from pashupati_fleet.core.switches import status_summary


def level_of(diagnostic) -> int:
    """Return a DiagnosticStatus level as int (rclpy gives the byte field as bytes)."""
    level = diagnostic.level
    return level[0] if isinstance(level, bytes) else int(level)


class FleetTelemetry:
    """Status subscriptions; events for NETRA go to send(message, durable)."""

    def __init__(self, node, health_diagnostics: str):
        self.node = node
        self.state = {
            'mode': 'auto',
            'nav': {'state': 'NAV_INACTIVE', 'message': ''},
            'mission': '',
            'localization': 'none',
            'battery': None,
            'driver': None,
            'people': 0,
            'inspection': None,
            'people_detection': None,
            'health': None,
            'estop': False,
            'field': None,
        }
        self.site = ''
        self.findings: list = []
        sub = node.create_subscription
        sub(ControlStatus, 'control/status', self.on_control, LATCHED)
        sub(NavigationStatus, 'navigation/status', self.on_nav, 10)
        sub(StopPointEvent, 'navigation/stop_point', self.on_stop, EVENTS)
        sub(LocalizationStatus, 'localization/status', self.on_localization, LATCHED)
        sub(SiteStatus, 'site/status', self.on_site, LATCHED)
        sub(HealthStatus, 'health/status', self.on_health, LATCHED)
        sub(DiagnosticArray, health_diagnostics, self.on_findings, 10)
        for topic, key in (
            ('anomaly_detector/status', 'inspection'),
            ('perception/people_status', 'people_detection'),
        ):
            sub(String, topic, lambda m, k=key: self.on_detector(k, m), LATCHED)
        sub(BatteryState, 'battery', self.on_battery, 10)
        sub(DiagnosticArray, 'diagnostics', self.on_driver, 10)

    def on_control(self, msg: ControlStatus):
        """Mode, field and the e-stop (an alert when it engages)."""
        engaged = bool(msg.estop)
        if engaged and not self.state['estop']:
            self.node.send(self.node.patrol.estop(list(msg.estop_reasons), self.node.pose()),
                           durable=True)  # fmt: skip
        self.state.update(mode=status.mode_name(msg), estop=engaged, field=status.field_name(msg))

    def on_nav(self, msg: NavigationStatus):
        """Follower state; alerts, laps and the end of a patrol."""
        name = status.nav_state_name(msg)
        self.state['nav'] = {'state': name, 'message': msg.message}
        events = self.node.patrol.navigation(
            name, msg.message, int(msg.index), int(msg.count), self.node.pose()
        )
        for event in events:
            self.node.send(event, durable=True)
        self.state['mission'] = self.node.patrol.mission()

    def on_stop(self, msg: StopPointEvent):
        """Forward a stop-point event."""
        event = self.node.patrol.stop_point(
            status.stop_event_name(msg), int(msg.index), msg.message, float(msg.x),
            float(msg.y), round(math.degrees(msg.yaw), 1),
        )  # fmt: skip
        self.node.send(event, durable=True)

    def on_localization(self, msg: LocalizationStatus):
        """'none' or the source of the last correction."""
        self.state['localization'] = status.localization_text(msg)

    def on_site(self, msg: SiteStatus):
        """Keep the active site (patrol routes from NETRA go into it)."""
        self.site = msg.site

    def on_findings(self, msg: DiagnosticArray):
        """Keep the health monitor's findings (the computer's figures, topic rates)."""
        self.findings = status.findings(msg)

    def on_health(self, msg: HealthStatus):
        """ok, issues, topic rates and the computer."""
        self.state['health'] = status.health_summary(msg, self.findings)

    def on_detector(self, key: str, msg: String):
        """On / off and state of a detector."""
        with contextlib.suppress(ValueError):
            self.state[key] = status_summary(json.loads(msg.data))

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
            self.state['driver'] = {'level': level_of(msg.status[0]), 'msg': msg.status[0].message}

    def people(self, msg: PoseArray) -> float:
        """Count people in view; return the stamp of the detection."""
        self.state['people'] = len(msg.poses)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return stamp or time.time()
