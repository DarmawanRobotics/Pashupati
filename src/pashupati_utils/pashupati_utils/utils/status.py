"""The Dharma status messages as the values the handheld and NETRA protocols carry."""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray

from dharma_interfaces.msg import (
    ControlStatus,
    HealthStatus,
    LocalizationStatus,
    NavigationStatus,
    StopPointEvent,
)
from pashupati_utils.core.system import Finding, system_figures, topic_rates

# Protocol v1 names; a blind scan is as unusable as a stale one, so it stays 'stale' there.
FIELD = {
    ControlStatus.FIELD_OFF: 'off',
    ControlStatus.FIELD_CLEAR: 'clear',
    ControlStatus.FIELD_LIMITED: 'limited',
    ControlStatus.FIELD_STOP: 'stop',
    ControlStatus.FIELD_STALE: 'stale',
    ControlStatus.FIELD_BLIND: 'stale',
}
NAV_NAMES = (
    'INACTIVE',
    'JOINING',
    'FOLLOWING',
    'APPROACHING',
    'DWELLING',
    'PAUSED',
    'BLOCKED',
    'OFF_PATH',
    'GOAL_REACHED',
    'FAILED',
)
NAV_STATES = {getattr(NavigationStatus, n): n for n in NAV_NAMES}
NAV_WIRE = {'INACTIVE': 'NAV_INACTIVE'}  # the name protocol v1 has for it
STOP_EVENTS = {
    StopPointEvent.ARRIVED: 'arrived',
    StopPointEvent.INSPECTED: 'inspected',
    StopPointEvent.DEPARTED: 'departed',
    StopPointEvent.SKIPPED: 'skipped',
}
HEALTH_LEVELS = {HealthStatus.OK: 'ok', HealthStatus.WARN: 'warn', HealthStatus.ERROR: 'error'}


def mode_name(msg: ControlStatus) -> str:
    """Return 'auto' or 'remote'."""
    return 'remote' if msg.mode == ControlStatus.MODE_REMOTE else 'auto'


def field_name(msg: ControlStatus) -> str:
    """Return the protective field state: clear, limited, stop, stale or off."""
    return FIELD.get(msg.field, 'stale')


def nav_state_name(msg: NavigationStatus) -> str:
    """Return the follower state as protocol v1 names it (NAV_INACTIVE, FOLLOWING, ...)."""
    name = NAV_STATES.get(msg.state, 'FAILED')
    return NAV_WIRE.get(name, name)


def localization_text(msg: LocalizationStatus) -> str:
    """Return 'none', or the source of the last correction (tag:<frame>, pose_estimate, ...)."""
    if msg.state == LocalizationStatus.NONE:
        return 'none'
    return msg.source or 'localized'


def stop_event_name(msg: StopPointEvent) -> str:
    """Return arrived, inspected, departed or skipped."""
    return STOP_EVENTS.get(msg.event, 'skipped')


def findings(msg: DiagnosticArray) -> list[Finding]:
    """Return the health findings of a diagnostics message."""
    return [Finding.of(s.name, {kv.key: kv.value for kv in s.values}) for s in msg.status]


def health_summary(msg: HealthStatus, report: list[Finding]) -> dict:
    """Return the health of protocol v1: ok, level, issues, topic rates, the computer."""
    return {
        'ok': msg.level == HealthStatus.OK,
        'level': HEALTH_LEVELS.get(msg.level, 'error'),
        'issues': list(msg.issues),
        'speed_scale': round(float(msg.speed_scale), 2),
        'topics': topic_rates(report),
        **system_figures(report),
    }
