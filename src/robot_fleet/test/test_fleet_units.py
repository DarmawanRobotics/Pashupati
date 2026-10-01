import json

from robot_fleet.anomaly import parse_result
from robot_fleet.crowd import CrowdAggregator
from robot_fleet.routes import summarize_csv
from robot_fleet.spool import Spool


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
