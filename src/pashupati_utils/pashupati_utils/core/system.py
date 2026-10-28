"""
The computer's figures and the sensor rates the handheld and NETRA show, from health findings.

health_monitor_node publishes one diagnostic status per finding, named '<prefix>: <finding>'
(`system:cpu`, `system:memory`, `sensor:/livox/lidar`, ...) with its numbers as key / value
pairs. These helpers turn them back into the fields of the Pashupati protocols.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import os

from pashupati_utils.core.numbers import text_number

SENSOR_PREFIX = 'sensor:'
# finding -> (protocol field, key of the finding's values, digits)
SYSTEM_FIELDS = (
    ('system:temperature', 'cpu_temp_c', 'value', 1),
    ('system:memory', 'memory_percent', 'value', 1),
    ('system:memory', 'memory_used_gb', 'used_gb', 1),
    ('system:memory', 'memory_total_gb', 'total_gb', 1),
    ('system:cpu', 'cpu_percent', 'value', 1),
    ('system:gpu', 'gpu_percent', 'value', 1),
    ('system:disk', 'disk_free_gb', 'value', 1),
)
FIGURES = (*dict.fromkeys(name for _, name, _, _ in SYSTEM_FIELDS), 'load')


@dataclass(frozen=True)
class Finding:
    """One health finding: its name without the diagnostics prefix and its values as text."""

    name: str
    values: dict = field(default_factory=dict)

    @classmethod
    def of(cls, status_name: str, values: dict) -> Finding:
        """Return the finding of a diagnostic status ('health: system:cpu' -> 'system:cpu')."""
        _, sep, rest = status_name.partition(': ')
        return cls(rest if sep else status_name, dict(values))

    def value(self, key: str) -> float | None:
        """Return one value as a number, None when missing or not a number."""
        return text_number(self.values.get(key, ''))


def system_figures(findings: list[Finding], cpus: int | None = None) -> dict:
    """Return the computer's figures (None when health did not report them)."""
    by_name = {f.name: f for f in findings}
    out = dict.fromkeys(FIGURES)
    for name, key, value_key, digits in SYSTEM_FIELDS:
        finding = by_name.get(name)
        value = finding.value(value_key) if finding is not None else None
        out[key] = None if value is None else round(value, digits)
    load = by_name.get('system:load')
    per_cpu = load.value('value') if load is not None else None
    if per_cpu is not None:  # the protocols carry the 1 min load average, not per CPU
        out['load'] = round(per_cpu * (cpus or os.cpu_count() or 1), 2)
    return out


def topic_rates(findings: list[Finding]) -> dict:
    """Return {topic: Hz} of the watched sensor topics (0 when not published)."""
    rates = {}
    for finding in findings:
        if finding.name.startswith(SENSOR_PREFIX):
            hz = finding.value('hz')
            rates[finding.name[len(SENSOR_PREFIX) :]] = round(hz, 1) if hz is not None else 0.0
    return rates
