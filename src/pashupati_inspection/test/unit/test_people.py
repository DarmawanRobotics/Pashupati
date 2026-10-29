from __future__ import annotations

import math

from pashupati_inspection.core.people import (
    NO_CAMERA_SEC,
    PeopleState,
    box_fractions,
    detection_rate,
)


def test_auto_rate_by_backend():
    assert detection_rate(0.0, 'onnxruntime-cuda') == 5.0
    assert detection_rate(0.0, 'onnxruntime-cpu') == 2.0
    assert detection_rate(0.0, 'ultralytics') == 5.0
    assert detection_rate(1.5, 'onnxruntime-cuda') == 1.5


def test_boxes_as_image_fractions():
    assert box_fractions([(64, 48, 320, 240, 0.876)], 640, 480) == [[0.1, 0.1, 0.5, 0.5, 0.88]]
    assert len(box_fractions([(0, 0, 1, 1, 0.5)] * 30, 10, 10)) == 20


def test_states():
    people = PeopleState(True, '/m/yolo.onnx', '/cam')
    assert people.state(0.0, False) == ('loading', '')  # no frame yet
    people.frame_time = 10.0
    assert people.state(10.5, True) == ('running', '')
    state, error = people.state(10.0 + NO_CAMERA_SEC + 1.0, True)
    assert state == 'error' and '/cam' in error
    people.load_error = 'no model'
    assert people.state(10.5, True) == ('error', 'no model')
    people.enabled = False
    assert people.state(10.5, True) == ('off', '')


def test_status_after_a_detection():
    people = PeopleState(True, '/m/yolo.onnx', '/cam')
    people.frame_time = 1.0
    people.detected([(0, 0, 10, 10, 0.9)], 100, 100, placed=1, latency=0.02)
    status = people.status(0.0, True, now=1.5)
    assert status['count'] == 1 and status['placed'] == 1 and status['latency_ms'] == 20
    assert status['model'] == 'yolo.onnx' and status['state'] == 'running'
    assert status['rate_hz'] == 2.0
    assert not math.isnan(status['at'])
