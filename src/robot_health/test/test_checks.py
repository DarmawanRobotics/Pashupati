from robot_health.checks import (
    disk_free_gb,
    missing_nodes,
    overall_level,
    parse_topic_specs,
    RateTracker,
    state_issues,
)


def test_rate_of_a_10hz_topic():
    """Ten messages a second read as ~10 Hz."""
    tracker = RateTracker(window_sec=5.0)
    for i in range(30):
        tracker.tick(i * 0.1)
    assert abs(tracker.rate(2.95) - 10.0) < 0.5


def test_silent_topic_drops_to_zero():
    """A topic that stopped publishing reads as 0 Hz."""
    tracker = RateTracker(window_sec=5.0)
    for i in range(20):
        tracker.tick(i * 0.1)
    assert tracker.rate(10.0) == 0.0


def test_topic_specs():
    """'topic:hz' strings parse, bare topics get 0 Hz."""
    expected = {'/a': 5.0, '/b/c': 12.5, '/d': 0.0}
    assert parse_topic_specs(['/a:5', '/b/c:12.5', '/d', '']) == expected


def test_disk_free_is_reported():
    """Disk space of the home directory is readable."""
    assert disk_free_gb('~') > 0


def test_missing_nodes_ignore_leading_slash():
    """Expected names match running names with or without a leading slash."""
    running = ['control_mux_node', '/path_follower_node']
    expected = ['/control_mux_node', 'path_follower_node', 'teleop_udp_node']
    assert missing_nodes(expected, running) == ['teleop_udp_node']
    assert missing_nodes([''], running) == []


def test_state_issues_and_level():
    """Lost odometry, a stale safety scan or the e-stop make the robot unable to patrol."""
    assert state_issues('ok', 'clear', False) == []
    assert state_issues('', '', False) == []
    issues = state_issues('lost: no odometry for 2.0 s', 'stale', True)
    assert len(issues) == 3
    assert overall_level([], []) == 'ok'
    assert overall_level(['/livox/lidar 2.0/5 Hz'], []) == 'warn'
    assert overall_level(issues, issues) == 'error'
