#!/usr/bin/env python3
import json
import os
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile
from robot_health.checks import (
    cpu_temperature, disk_free_gb, memory_percent, parse_topic_specs, RateTracker,
)
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


class HealthMonitorNode(Node):
    """Watches sensor topic rates and the computer, publishes diagnostics and a health summary."""

    def __init__(self):
        """Declare params and start the checks."""
        super().__init__('health_monitor_node')
        defaults = {
            'topics': [
                '/livox/lidar:5.0',
                '/perception/sector_scan:5.0',
                '/camera/camera/color/image_raw/compressed:5.0',
                '/l1w/odom:10.0',
            ],
            'max_cpu_temp_c': 85.0,
            'max_memory_percent': 92.0,
            'min_disk_free_gb': 5.0,
            'disk_path': '~',
            'publish_hz': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._expected = parse_topic_specs(p('topics'))
        self._trackers = {topic: RateTracker() for topic in self._expected}
        self._subscribed: set[str] = set()
        self._limits = (
            float(p('max_cpu_temp_c')),
            float(p('max_memory_percent')),
            float(p('min_disk_free_gb')),
        )
        self._disk_path = p('disk_path')

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._diag_pub = self.create_publisher(DiagnosticArray, 'pashupati/diagnostics', 10)
        self._status_pub = self.create_publisher(String, 'health/status', latched)
        self.create_timer(5.0, self.discover)
        self.create_timer(1.0 / float(p('publish_hz')), self.publish)
        self.discover()

    def discover(self):
        """Subscribe (raw, no deserialisation) to watched topics as they appear."""
        types = dict(self.get_topic_names_and_types())
        for topic in self._expected:
            if topic in self._subscribed or topic not in types:
                continue
            try:
                msg_type = get_message(types[topic][0])
            except (AttributeError, ModuleNotFoundError, ValueError):
                continue
            tracker = self._trackers[topic]
            self.create_subscription(
                msg_type,
                topic,
                lambda _m, t=tracker: t.tick(time.monotonic()),
                qos_profile_sensor_data,
                raw=True,
            )
            self._subscribed.add(topic)

    def publish(self):
        """Evaluate every check and publish diagnostics plus a JSON summary."""
        now = time.monotonic()
        issues, statuses, rates = [], [], {}
        for topic, min_hz in self._expected.items():
            hz = self._trackers[topic].rate(now)
            rates[topic] = round(hz, 1)
            ok = hz >= min_hz * 0.8
            if not ok:
                issues.append(f'{topic} {hz:.1f}/{min_hz:.0f} Hz')
            statuses.append(self.status(f'topic {topic}', ok, f'{hz:.1f} Hz', {'min_hz': min_hz}))

        max_temp, max_mem, min_disk = self._limits
        temp, mem, disk = cpu_temperature(), memory_percent(), disk_free_gb(self._disk_path)
        load = os.getloadavg()[0]
        system = {'cpu_temp_c': temp, 'memory_percent': mem, 'disk_free_gb': disk, 'load': load}
        for label, value, bad in (
            ('cpu temperature', temp, temp is not None and temp > max_temp),
            ('memory', mem, mem is not None and mem > max_mem),
            ('disk', disk, disk is not None and disk < min_disk),
        ):
            if bad:
                issues.append(f'{label} {value:.0f}')
            statuses.append(self.status(f'system {label}', not bad, f'{value}', {}))

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses
        self._diag_pub.publish(array)
        summary = {
            'ok': not issues,
            'issues': issues,
            'topics': rates,
            **{k: (round(v, 1) if v is not None else None) for k, v in system.items()},
        }
        self._status_pub.publish(String(data=json.dumps(summary)))

    @staticmethod
    def status(name: str, ok: bool, message: str, values: dict) -> DiagnosticStatus:
        """Return a DiagnosticStatus."""
        s = DiagnosticStatus()
        s.name = f'pashupati: {name}'
        s.level = DiagnosticStatus.OK if ok else DiagnosticStatus.WARN
        s.message = message
        s.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        return s


def main(args=None):
    """Spin the health monitor."""
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
