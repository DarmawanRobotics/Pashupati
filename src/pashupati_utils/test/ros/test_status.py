from __future__ import annotations

from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pashupati_utils.utils import status
from pashupati_utils.utils.params import outcome, set_parameters_request

from dharma_interfaces.msg import (
    ControlStatus,
    HealthStatus,
    LocalizationStatus,
    NavigationStatus,
    StopPointEvent,
)


def test_control_status_names():
    msg = ControlStatus(mode=ControlStatus.MODE_REMOTE, field=ControlStatus.FIELD_LIMITED)
    assert (status.mode_name(msg), status.field_name(msg)) == ('remote', 'limited')
    assert status.field_name(ControlStatus(field=ControlStatus.FIELD_BLIND)) == 'stale'
    assert status.mode_name(ControlStatus()) == 'auto'


def test_navigation_state_names_of_protocol_v1():
    assert status.nav_state_name(NavigationStatus(state=NavigationStatus.INACTIVE)) == (
        'NAV_INACTIVE'
    )
    for name in ('FOLLOWING', 'BLOCKED', 'OFF_PATH', 'GOAL_REACHED', 'DWELLING', 'FAILED'):
        msg = NavigationStatus(state=getattr(NavigationStatus, name))
        assert status.nav_state_name(msg) == name


def test_localization_text():
    assert status.localization_text(LocalizationStatus()) == 'none'
    msg = LocalizationStatus(state=LocalizationStatus.LOCALIZED, source='tag:base_map')
    assert status.localization_text(msg) == 'tag:base_map'


def test_stop_events():
    assert status.stop_event_name(StopPointEvent(event=StopPointEvent.INSPECTED)) == 'inspected'


def test_health_summary_from_status_and_diagnostics():
    array = DiagnosticArray(
        status=[
            DiagnosticStatus(
                name='health: system:cpu', values=[KeyValue(key='value', value='12')]
            ),
            DiagnosticStatus(
                name='health: sensor:/Odometry', values=[KeyValue(key='hz', value='9.9')]
            ),
        ]
    )
    msg = HealthStatus(level=HealthStatus.WARN, issues=['odometry late'], speed_scale=0.5)
    summary = status.health_summary(msg, status.findings(array))
    assert summary['ok'] is False
    assert summary['level'] == 'warn'
    assert summary['issues'] == ['odometry late']
    assert summary['cpu_percent'] == 12.0
    assert summary['topics'] == {'/Odometry': 9.9}
    assert summary['speed_scale'] == 0.5


def test_parameter_requests_and_outcomes():
    request = set_parameters_request([('enabled', True), ('categories', [])])
    assert [p.name for p in request.parameters] == ['enabled', 'categories']
    assert list(request.parameters[1].value.string_array_value) == ['']
    refused = SimpleNamespace(results=[SimpleNamespace(successful=False, reason='no')])
    assert outcome(refused) == (False, 'no')
    assert outcome(SimpleNamespace(success=True, message='done')) == (True, 'done')
    assert outcome(None) == (False, 'no response')
