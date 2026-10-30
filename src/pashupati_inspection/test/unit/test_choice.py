from __future__ import annotations

import json

from pashupati_inspection.core.choice import read_choice, restored, write_choice

KEYS = ('enabled', 'model', 'interval_sec', 'categories', 'alarm_categories')


def test_only_valid_values_are_restored():
    data = {
        'enabled': 1,  # not a bool
        'model': ' moondream ',
        'interval_sec': float('nan'),
        'categories': ['trash', 'bag'],
        'alarm_categories': [],
        'unknown': True,
    }
    assert restored(data, KEYS) == {
        'model': 'moondream',
        'categories': ['trash', 'bag'],
        'alarm_categories': [''],  # never an empty list for rclpy
    }
    assert restored('junk', KEYS) == {}


def test_round_trip(tmp_path):
    path = str(tmp_path / 'sub' / 'choice.json')
    assert read_choice(path, KEYS) == ({}, None)  # no file yet: no choice, no error
    assert write_choice(path, {'enabled': False, 'interval_sec': 5.0}) is None
    assert read_choice(path, KEYS) == ({'enabled': False, 'interval_sec': 5.0}, None)
    assert json.loads((tmp_path / 'sub' / 'choice.json').read_text())['enabled'] is False


def test_broken_file_is_reported(tmp_path):
    path = tmp_path / 'choice.json'
    path.write_text('{broken')
    values, error = read_choice(str(path), KEYS)
    assert values == {} and 'choice.json' in error


def test_no_state_file_means_nothing_kept():
    assert read_choice('', KEYS) == ({}, None)
    assert write_choice('', {'enabled': True}) is None
