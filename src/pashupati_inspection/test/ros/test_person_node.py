"""In-process tests of person_detector_node without a model: status, live on / off."""

from __future__ import annotations

import json

from pashupati_inspection.person_detector_node import PersonDetectorNode
import pytest
from rclpy.parameter import Parameter

from dharma_utils.ros import compat


@pytest.fixture
def detector(tmp_path):
    compat.init(
        args=[
            '--ros-args',
            '-p', f'state_file:={tmp_path / "people.json"}',
            '-p', f'model_path:={tmp_path / "missing.onnx"}',
            '-p', 'image_topic:=/test/people/image',
        ]
    )  # fmt: skip
    node = PersonDetectorNode()
    yield node
    node.destroy_node()
    compat.try_shutdown()


def test_status_without_a_camera(detector):
    status = detector.status()
    assert status['state'] == 'loading' and status['enabled'] is True
    assert status['model'] == 'missing.onnx' and status['rate_hz'] == 2.0


def test_switching_off_is_kept(detector, tmp_path):
    assert detector.set_parameters([Parameter('enabled', value=False)])[0].successful
    assert detector.status()['state'] == 'off'
    assert json.loads((tmp_path / 'people.json').read_text()) == {'enabled': False}
    again = PersonDetectorNode()
    try:
        assert again.status()['enabled'] is False
    finally:
        again.destroy_node()


def test_a_missing_model_is_an_error(detector):
    detector.people.frame_time = 0.0
    assert not detector.ensure_detector()
    assert 'missing.onnx' in detector.status()['error'] or detector.status()['state'] == 'error'


def test_bad_rates_are_refused(detector):
    result = detector.set_parameters([Parameter('rate_hz', value=0.05)])[0]
    assert not result.successful and 'rate_hz' in result.reason
