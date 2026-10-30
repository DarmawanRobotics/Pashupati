from __future__ import annotations

import json

from pashupati_fleet.core.anomaly import parse_result
from pashupati_fleet.core.crowd import CrowdAggregator
from pashupati_fleet.core.routes import summarize_csv
from pashupati_fleet.core.spool import Spool


def test_crowd_keeps_peak_per_cell_and_window():
    """Each cell keeps its busiest frame; the window flushes once."""
    agg = CrowdAggregator(cell_size=1.0, window_sec=60.0)
    agg.add(0.0, [(0.2, 0.2), (0.8, 0.5), (5.0, 5.0)])
    agg.add(1.0, [(0.5, 0.5)])
    assert agg.flush(10.0) is None
    snap = agg.flush(61.0)
    assert snap['cells'] == [[0, 0, 2], [5, 5, 1]]
    assert snap['peak_people'] == 3
    assert agg.flush(200.0) is None


def test_spool_roundtrip(tmp_path):
    """Spooled items survive until removed."""
    spool = Spool(str(tmp_path))
    uid = spool.put('anomaly', {'category': 'trash'}, {'image.jpg': b'jpg'})
    (item,) = spool.items()
    kind, meta, files = Spool.load(item)
    assert (kind, meta['uid'], files) == ('anomaly', uid, {'image.jpg': b'jpg'})
    Spool.remove(item)
    assert spool.items() == []


def test_parse_result_variants():
    """Different detector JSON shapes normalise to one record."""
    assert parse_result(json.dumps({'is_anomaly': False, 'label': 'trash'})) is None
    assert parse_result(json.dumps({'label': 'none'})) is None
    r = parse_result(json.dumps({'anomaly': True, 'category': 'Floor Damage', 'score': 0.7}))
    assert r == {'category': 'floor_damage', 'confidence': 0.7, 'description': ''}


def test_route_summary():
    """Length, stops and loop closure from a CSV."""
    s = summarize_csv('# x,y,yaw,dwell\n0,0,0,0\n3,4,0,5\n0,0.5,0,0\n')
    assert s == {'length_m': 9.61, 'points': 3, 'stops': 1, 'closed': True}


def test_spool_quarantines_unreadable_items(tmp_path):
    """A corrupted item is moved aside instead of blocking every later upload."""
    spool = Spool(str(tmp_path))
    spool.put('anomaly', {'category': 'trash'}, {'image.jpg': b'jpg'})
    (item,) = spool.items()
    (tmp_path / item.split('/')[-1] / 'meta.json').write_text('{broken')
    try:
        Spool.load(item)
        raise AssertionError('corrupted meta.json must not load')
    except ValueError:
        spool.quarantine(item)
    assert spool.items() == []
    assert len(list((tmp_path / '.failed').iterdir())) == 1


def test_route_ids_cannot_escape_the_routes_dir():
    """Only plain ids are accepted for downloaded routes."""
    from pashupati_fleet.core.routes import safe_route_id

    for good in ('42', 42, 'route-7', 'a1b2_c3.v2'):
        assert safe_route_id(good)
    for bad in ('', None, '../etc/passwd', '/abs', 'a/b', '.hidden', 'x' * 65, ['1']):
        assert not safe_route_id(bad)


def test_live_switches_from_the_command_center():
    """set_inspection / set_people map to the detectors' live parameters; junk is refused."""
    from pashupati_fleet.core.switches import live_parameters, status_summary
    import pytest

    assert live_parameters('set_people', {'enabled': False}) == [('enabled', False)]
    assert live_parameters('set_inspection', {'model': ' moondream ', 'interval': -9}) == [
        ('model', 'moondream'),
        ('interval_sec', -1.0),
    ]
    for command, args in (
        ('set_people', {}),
        ('set_people', {'model': 'x'}),  # people have no model to choose
        ('set_inspection', {'enabled': 1}),
        ('set_inspection', {'interval': float('nan')}),
    ):
        with pytest.raises(ValueError):
            live_parameters(command, args)
    status = {'enabled': True, 'state': 'running', 'models': [], 'count': 3}
    assert status_summary(status) == {'enabled': True, 'state': 'running', 'count': 3}
    assert status_summary('junk') is None


def test_queued_finding_takes_the_pose_of_when_it_was_seen():
    """A frame inspected late is placed where the robot was when the camera took it."""

    from pashupati_fleet.core.poses import pose_at

    history = [(100.0, {'x': 0.0}), (101.0, {'x': 0.5}), (102.0, {'x': 1.0})]
    assert pose_at(history, 101.3) == {'x': 0.5}
    assert pose_at(history, 107.5) is None  # nothing recorded within 5 s
    assert pose_at([], 101.0) is None
