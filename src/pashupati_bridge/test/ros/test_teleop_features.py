"""In-process tests of the handheld's inspection, people, patrol speed and health views."""

from __future__ import annotations

import json
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest
from std_msgs.msg import String
from teleop_rig import acks_of, fake_node, wait_until

from dharma_interfaces.msg import HealthStatus
from dharma_utils.ros import LATCHED


def test_inspection_arguments_are_checked():
    """inspection_set maps to the detector's live parameters; junk is refused with the reason."""
    from pashupati_bridge.core.commands import inspection_parameters
    from pashupati_utils.utils.params import set_parameters_request

    assert inspection_parameters({'enabled': False, 'model': ' moondream ', 'interval': 0}) == [
        ('enabled', False),
        ('model', 'moondream'),
        ('interval_sec', 0.0),
    ]
    assert inspection_parameters({'interval': -5}) == [('interval_sec', -1.0)]
    for bad in ({}, {'enabled': 'yes'}, {'model': ''}, {'interval': 'x'}, {'interval': 1e9}):
        with pytest.raises(ValueError):
            inspection_parameters(bad)
    request = set_parameters_request([('enabled', True), ('interval_sec', 2.0), ('model', 'm')])
    assert [p.value.type for p in request.parameters] == [1, 3, 4]  # bool, double, string


def test_own_finding_types_map_to_the_category_parameters():
    """The handheld's types become names, reported names and 'name: description' entries."""
    from pashupati_bridge.core.commands import inspection_parameters
    from pashupati_utils.utils.params import set_parameters_request

    types = [
        {'name': 'trash', 'desc': '', 'report': True},
        {'name': 'bag', 'desc': ' a bag  left alone ', 'report': True},
        {'name': 'stain', 'report': False},
    ]
    params = inspection_parameters({'categories': types})
    assert params == [
        ('categories', ['trash', 'bag', 'stain']),
        ('alarm_categories', ['trash', 'bag']),
        ('category_descriptions', ['bag: a bag left alone']),
    ]
    request = set_parameters_request([*params, ('alarm_categories', [])])
    assert {p.value.type for p in request.parameters} == {9}  # string arrays, also when empty
    assert list(request.parameters[-1].value.string_array_value) == ['']
    for bad in (
        [],
        [{'name': 'Bag'}],
        [{'name': 'none'}],
        [{'name': 'bag'}, {'name': 'bag'}],
        [{'name': 'bag', 'desc': 'x' * 201}],
        [{'name': f'c{i}'} for i in range(13)],
        'trash',
    ):
        with pytest.raises(ValueError):
            inspection_parameters({'categories': bad})


def test_inspection_is_set_on_the_detector_and_shown(rig):
    """The handheld turns inspection off and picks a model; the detector status reaches it."""
    detector = fake_node(
        rig,
        'anomaly_detector_node',
        {
            'enabled': True,
            'model': 'qwen3-vl:2b-instruct',
            'interval_sec': 0.0,
            'categories': ['trash', 'spill', 'stain'],
            'alarm_categories': ['trash', 'spill'],
            'category_descriptions': [''],
        },
    )
    status_pub = detector.create_publisher(String, 'anomaly_detector/status', LATCHED)
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    welcome = next(m for m in rig.received(a) if m['t'] == 'welcome')
    assert {'inspection_set', 'inspection_models', 'nav_speed'} <= set(welcome['commands'])
    assert wait_until(rig, rig.node.calls.table['inspection_set'][0].service_is_ready)

    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'inspection_set', 'args': {'enabled': False}})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'inspection_set', 'args': {'model': 'moondream'}})
    rig.send(a, {'t': 'cmd', 'id': 3, 'name': 'inspection_set', 'args': {'interval': 'soon'}})
    types = [{'name': 'trash', 'report': False}, {'name': 'bag', 'desc': 'a bag', 'report': True}]
    rig.send(a, {'t': 'cmd', 'id': 5, 'name': 'inspection_set', 'args': {'categories': types}})
    acks = {}
    wait_until(rig, lambda: acks.update(acks_of(rig, a)) or len(acks) >= 4)
    assert acks[1]['ok'] and acks[2]['ok'] and not acks[3]['ok'] and acks[5]['ok']
    assert detector.get_parameter('enabled').value is False
    assert detector.get_parameter('model').value == 'moondream'
    assert list(detector.get_parameter('categories').value) == ['trash', 'bag']
    assert list(detector.get_parameter('alarm_categories').value) == ['bag']
    assert list(detector.get_parameter('category_descriptions').value) == ['bag: a bag']

    status = {
        'enabled': False,
        'model': 'moondream',
        'state': 'off',
        'interval_sec': 0.0,
        'models': [{'name': 'moondream', 'installed': True, 'active': True}],
        'categories': [{'name': 'bag', 'desc': 'a bag', 'report': True}],
        'last': {
            'category': 'trash',
            'reported': True,
            'latency_ms': 900,
            'at': time.time(),
            'waited_sec': 12.4,
        },
        'inspections': 7,
        'reports': 1,
        'queue': 4,
        'queue_max': 50,
    }
    status_pub.publish(String(data=json.dumps(status)))
    states = []
    wait_until(
        rig,
        lambda: (
            states.extend(m for m in rig.received(a) if m['t'] == 'state')
            or (states and states[-1]['inspection'] is not None)
        ),
    )
    inspection = states[-1]['inspection']
    assert (inspection['on'], inspection['state'], inspection['model']) == (
        False,
        'off',
        'moondream',
    )
    assert inspection['last']['cat'] == 'trash' and inspection['reports'] == 1
    assert (inspection['queue'], inspection['queue_max'], inspection['last']['waited']) == (
        4,
        50,
        12,
    )
    rig.send(a, {'t': 'cmd', 'id': 4, 'name': 'inspection_models'})
    assert acks_of(rig, a)[4]['data'] == status['models']
    rig.send(a, {'t': 'cmd', 'id': 6, 'name': 'inspection_categories'})
    assert acks_of(rig, a)[6]['data'] == status['categories']
    rig.executor.remove_node(detector)
    detector.destroy_node()


def test_patrol_speed_is_read_and_set_within_the_limits(rig):
    """The handheld sees the path follower's speed and sets it, never above its maximum."""
    follower = fake_node(rig, 'path_follower_node', {'speed.cruise': 0.4})
    a = rig.client()
    assert wait_until(rig, rig.node.calls.speed_client.service_is_ready)
    rig.send(a, {'t': 'hello'})
    states = []
    wait_until(
        rig,
        lambda: (
            states.extend(m for m in rig.received(a) if m['t'] == 'state')
            or (states and states[-1]['speed'] is not None)
        ),
    )
    assert states[-1]['speed'] == {'target': 0.4, 'min': 0.1, 'max': 0.6}

    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'nav_speed', 'args': {'speed': 0.55}})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'nav_speed', 'args': {'speed': 0.9}})
    rig.send(a, {'t': 'cmd', 'id': 3, 'name': 'nav_speed', 'args': {}})
    acks = {}
    wait_until(rig, lambda: acks.update(acks_of(rig, a)) or len(acks) >= 3)
    assert acks[1]['ok'] and not acks[2]['ok'] and not acks[3]['ok']
    assert '0.6' in acks[2]['msg']
    assert follower.get_parameter('speed.cruise').value == 0.55
    rig.executor.remove_node(follower)
    follower.destroy_node()


def test_people_detection_is_switched_and_shown_with_the_system_load(rig):
    """The handheld turns the person detector off; people, CPU, GPU and RAM reach it."""
    detector = fake_node(rig, 'person_detector_node', {'enabled': True})
    people_pub = detector.create_publisher(String, 'perception/people_status', LATCHED)
    health_pub = detector.create_publisher(HealthStatus, 'health/status', LATCHED)
    findings_pub = detector.create_publisher(DiagnosticArray, '/diagnostics', 10)
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    welcome = next(m for m in rig.received(a) if m['t'] == 'welcome')
    assert 'people_set' in welcome['commands']
    assert wait_until(rig, rig.node.calls.table['people_set'][0].service_is_ready)

    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'people_set', 'args': {'enabled': False}})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'people_set', 'args': {'enabled': 'no'}})
    acks = {}
    wait_until(rig, lambda: acks.update(acks_of(rig, a)) or len(acks) >= 2)
    assert acks[1]['ok'] and not acks[2]['ok']
    assert detector.get_parameter('enabled').value is False

    people = {'enabled': True, 'state': 'running', 'count': 3, 'latency_ms': 80, 'at': time.time()}
    people['boxes'] = [[0.1, 0.2, 0.3, 0.9, 0.87], [0, 0, 1.5, 0.5, 0.5], [1, 2], 'x']
    people_pub.publish(String(data=json.dumps(people)))
    findings = {
        'system:cpu': {'value': '23.4'},
        'system:gpu': {'value': '41.2'},
        'system:memory': {'value': '33.3', 'used_gb': '5.12', 'total_gb': '15.3'},
        'system:temperature': {'value': '52.6'},
    }
    array = DiagnosticArray(
        status=[
            DiagnosticStatus(
                name=f'health: {name}', values=[KeyValue(key=k, value=v) for k, v in kv.items()]
            )
            for name, kv in findings.items()
        ]
    )
    for _ in range(3):  # volatile: the gateway may not have discovered the publisher yet
        findings_pub.publish(array)
        rig.spin(0.2)
    health_pub.publish(HealthStatus(level=HealthStatus.OK))
    states = []

    def arrived():
        states.extend(m for m in rig.received(a) if m['t'] == 'state')
        return bool(states) and states[-1]['people'] is not None and states[-1]['health']

    assert wait_until(rig, arrived)
    state = states[-1]
    assert (state['people']['count'], state['people']['state']) == (3, 'running')
    assert state['people']['boxes'] == [[0.1, 0.2, 0.3, 0.9, 0.87], [0.0, 0.0, 1.0, 0.5, 0.5]]
    assert (state['health']['cpu'], state['health']['gpu'], state['health']['ram_used']) == (
        23.0,
        41.0,
        5.1,
    )
    rig.executor.remove_node(detector)
    detector.destroy_node()
