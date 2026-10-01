import pytest
from robot_bridge.mux import ControlMux


def test_auto_forwards_navigation_only():
    """Auto mode passes navigation and drops teleop."""
    mux = ControlMux('auto')
    assert mux.on_nav('nav') == 'nav'
    assert mux.on_remote('joy', 0.0) is None


def test_remote_forwards_teleop_only_and_stops_when_silent():
    """Remote mode passes teleop, drops navigation and zeroes once after the timeout."""
    mux = ControlMux('auto', remote_timeout=0.3)
    assert mux.set_mode('remote')
    assert mux.watchdog(0.0)  # zero right after the switch
    assert mux.on_nav('nav') is None
    assert mux.on_remote('joy', 1.0) == 'joy'
    assert not mux.watchdog(1.2)
    assert mux.watchdog(1.4)
    assert not mux.watchdog(1.5)


def test_unknown_mode_rejected():
    """Only auto and remote exist."""
    with pytest.raises(ValueError):
        ControlMux('manual')
