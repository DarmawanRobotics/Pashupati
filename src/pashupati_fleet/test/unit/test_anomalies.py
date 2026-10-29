from __future__ import annotations

import json

from pashupati_fleet.core.anomalies import Anomalies, stamp_key
from pashupati_fleet.core.poses import PoseHistory


def result(**extra) -> str:
    return json.dumps({'is_anomaly': True, 'category': 'trash', 'confidence': 0.9, **extra})


def test_a_finding_takes_its_analysed_frame():
    anomalies = Anomalies(0.5)
    key = stamp_key(10, 5)
    assert anomalies.finding(result(evidence_stamp=key, captured_at=100.0), {'patrol_id': 1},
                             {'x': 1}, 3, PoseHistory(), 100.5)  # fmt: skip
    assert anomalies.ready(100.6) == ([], 0)  # waits for its frame
    anomalies.evidence.append((key, b'jpg'))
    ((meta, jpeg),), dropped = anomalies.ready(100.7)
    assert (jpeg, dropped, meta['evidence'], meta['stop_index']) == (
        b'jpg',
        0,
        'analysed_frame',
        3,
    )
    assert meta['patrol_id'] == 1 and meta['detector'] == 'robot'


def test_without_its_frame_the_latest_one_is_used_or_it_is_dropped():
    anomalies = Anomalies(0.5)
    anomalies.finding(result(), {}, None, None, PoseHistory(), 100.0)
    anomalies.latest = (99.0, b'latest')
    ((meta, jpeg),), _ = anomalies.ready(100.0)
    assert jpeg == b'latest' and meta['evidence'] == 'latest_frame'
    anomalies.finding(result(), {}, None, None, PoseHistory(), 200.0)
    assert anomalies.ready(200.0) == ([], 1)  # the latest frame is too old


def test_a_late_finding_gets_the_pose_and_time_of_its_frame():
    poses = PoseHistory()
    poses.add(50.0, {'x': 0.5})
    poses.add(90.0, {'x': 9.0})
    anomalies = Anomalies(0.5)
    anomalies.finding(result(captured_at=50.2), {'ts': 100.0}, {'x': 9.0}, 2, poses, 100.0)
    meta = anomalies.pending[0][1]
    assert (meta['ts'], meta['pose'], meta['stop_index']) == (50.2, {'x': 0.5}, None)


def test_weak_or_no_findings_are_ignored():
    anomalies = Anomalies(0.5)
    assert not anomalies.finding(result(confidence=0.2), {}, None, None, PoseHistory(), 0.0)
    assert not anomalies.finding(json.dumps({'category': 'none'}), {}, None, None,
                                 PoseHistory(), 0.0)  # fmt: skip
