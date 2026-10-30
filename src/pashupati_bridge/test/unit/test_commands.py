from __future__ import annotations

from pashupati_bridge.core import commands
import pytest


def test_every_trigger_is_a_dharma_service():
    assert commands.TRIGGERS['estop'] == 'control/estop/engage'
    assert commands.TRIGGERS['nav_start'] == 'navigation/start'
    assert commands.TRIGGERS['record_start'] == 'route/start'
    assert commands.TRIGGERS['dock'] == 'docking/dock'


def test_modes():
    assert commands.mode_of({'mode': 'REMOTE'}) == 'remote'
    assert commands.mode_of({}) == 'auto'
    with pytest.raises(ValueError):
        commands.mode_of({'mode': 'fly'})


def test_dwell():
    assert commands.dwell_seconds(5) == 5.0
    for bad in (0, -1, 4000, 'x', float('nan'), True):
        with pytest.raises(ValueError):
            commands.dwell_seconds(bad)


def test_inspection_arguments():
    assert commands.inspection_parameters({'interval': 0.05}) == [('interval_sec', 0.1)]
    assert commands.people_parameters({'enabled': False}) == [('enabled', False)]
    with pytest.raises(ValueError):
        commands.people_parameters({'enabled': 0})


def test_patrol_speed():
    assert commands.patrol_speed(0.456, 0.1, 0.6) == 0.46
    for bad in (0.05, 0.7, None, 'fast'):
        with pytest.raises(ValueError, match=r'0\.1 to 0\.6'):
            commands.patrol_speed(bad, 0.1, 0.6)


def test_ids_and_camera_rate():
    assert commands.command_id(3) == 3 and commands.command_id('a') == 'a'
    assert commands.command_id(True) is None and commands.command_id([1]) is None
    assert commands.camera_period(2.0, 8.0) == 0.5
    assert commands.camera_period(100.0, 8.0) == 1.0 / 8.0
    assert commands.camera_period(float('nan'), 8.0) is None
    assert commands.camera_period(None, 8.0) is None
