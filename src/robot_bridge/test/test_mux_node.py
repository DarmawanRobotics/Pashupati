"""In-process tests of control_mux_node: e-stop service, request topic and forwarding."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')
Twist = pytest.importorskip('geometry_msgs.msg').Twist
std_msgs = pytest.importorskip('std_msgs.msg')
SetBool = pytest.importorskip('std_srvs.srv').SetBool
qos = pytest.importorskip('rclpy.qos')
SingleThreadedExecutor = pytest.importorskip('rclpy.executors').SingleThreadedExecutor
ControlMuxNode = pytest.importorskip('robot_bridge.control_mux_node').ControlMuxNode


class Rig:
    """The mux, a fake navigation pause service and recorders for its outputs."""

    def __init__(self):
        rclpy.init()
        self.mux = ControlMuxNode()
        self.helper = rclpy.create_node('mux_test_helper')
        self.out, self.engaged, self.pauses = [], [], []
        self.helper.create_subscription(Twist, 'cmd_vel', self.out.append, 50)
        latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.helper.create_subscription(
            std_msgs.Bool, 'bridge/estop_engaged', lambda m: self.engaged.append(m.data), latched
        )
        self.helper.create_service(SetBool, 'navigation/pause', self.on_pause)
        self.nav = self.helper.create_publisher(Twist, 'bridge/nav_cmd_vel', 10)
        self.request = self.helper.create_publisher(std_msgs.String, 'bridge/estop_request', 10)
        self.estop = self.helper.create_client(SetBool, 'bridge/estop')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.mux)
        self.executor.add_node(self.helper)

    def on_pause(self, request, response):
        """Record navigation/pause calls."""
        self.pauses.append(request.data)
        response.success = True
        return response

    def spin(self, seconds: float):
        """Spin both nodes."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def call(self, data: bool):
        """Call bridge/estop and wait for the answer."""
        future = self.estop.call_async(SetBool.Request(data=data))
        end = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
        return future.result()

    def drive(self, vx: float, seconds: float = 0.3):
        """Publish a navigation command and spin."""
        cmd = Twist()
        cmd.linear.x = vx
        self.nav.publish(cmd)
        self.spin(seconds)

    def close(self):
        """Tear down."""
        self.executor.shutdown()
        self.helper.destroy_node()
        self.mux.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def rig():
    """Provide a running mux."""
    r = Rig()
    r.spin(0.5)  # discovery
    yield r
    r.close()


def test_estop_service_holds_the_robot_still(rig):
    """Engaged: nav is blocked, zero is streamed, navigation paused; release lets nav through."""
    rig.drive(0.4)
    assert rig.out and rig.out[-1].linear.x == 0.4
    assert rig.call(True).success
    rig.spin(0.3)
    assert rig.engaged[-1] is True and rig.pauses == [True]
    rig.out.clear()
    rig.drive(0.4, 0.5)
    assert rig.out and all(t.linear.x == 0.0 for t in rig.out)  # only the hold stream
    assert rig.call(False).success
    rig.spin(0.2)
    assert rig.engaged[-1] is False and rig.pauses == [True]  # release never resumes
    rig.drive(0.3)
    assert rig.out[-1].linear.x == 0.3


def test_estop_request_topic_engages(rig):
    """Any node can engage the e-stop with a reason on bridge/estop_request."""
    rig.request.publish(std_msgs.String(data='unit test'))
    rig.spin(0.3)
    assert rig.engaged[-1] is True
    assert rig.mux._mux.estop_reason == 'unit test'
