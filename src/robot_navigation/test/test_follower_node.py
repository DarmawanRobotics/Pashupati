"""In-process tests of path_follower_node behaviour that needs a real rclpy node."""

import pytest

rclpy = pytest.importorskip('rclpy')
BatteryState = pytest.importorskip('sensor_msgs.msg').BatteryState
Trigger = pytest.importorskip('std_srvs.srv').Trigger
PathFollowerNode = pytest.importorskip(
    'robot_navigation.path_follower_node'
).PathFollowerNode


@pytest.fixture
def follower():
    """Provide a path follower node."""
    rclpy.init()
    node = PathFollowerNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_status_changes_between_info_and_warn_states(follower):
    """FOLLOWING -> BLOCKED -> FOLLOWING must not raise (rclpy severity per call site)."""
    stamp = follower.get_clock().now().to_msg()
    for state in ('FOLLOWING', 'BLOCKED', 'FOLLOWING', 'OFF_PATH', 'PAUSED', 'EMERGENCY_STOP'):
        follower.publish_status(stamp, state, 'test')


def test_low_battery_flag_clears_after_charging(follower):
    """The low-battery flag sets below the threshold and clears 5 % above it."""
    def battery(pct):
        msg = BatteryState()
        msg.percentage = pct
        follower.battery_callback(msg)
        return follower._battery_low

    assert not battery(0.5)
    assert battery(0.15)
    assert battery(0.22)  # within the hysteresis band
    assert battery(float('nan'))  # unknown readings change nothing
    assert not battery(0.3)


def test_stop_reports_inspection_when_every_inspector_is_down(follower):
    """With no inspection service up, 'inspected' still goes out (saying so) at the stop."""
    events = []
    follower.publish_stop_event = lambda event, message='', index=None: events.append(
        (event, message, index)
    )
    follower._inspection_clients = [follower.create_client(Trigger, 'no_such_inspector')]
    follower._next_stop_index = 0
    follower.start_inspection()
    assert len(events) == 1
    event, message, index = events[0]
    assert (event, index) == ('inspected', 0)
    assert 'no_such_inspector: not available' in message
    assert follower._inspection_pending == 0
