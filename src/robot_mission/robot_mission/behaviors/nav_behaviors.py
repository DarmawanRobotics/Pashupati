import py_trees

from rclpy.action import ActionClient

from std_msgs.msg import Empty

from robot_interfaces.action import NavigateRoute


class NavigateRouteAction(py_trees.behaviour.Behaviour):
    """Sends one NavigateRoute action goal, RUNNING until it succeeds/fails/is canceled, and
    publishes mission/inspection_trigger once each time feedback reports entering DWELLING.
    Cancels the outstanding goal if the tree tears this behaviour down mid-navigation."""

    def __init__(self, node, name='NavigateRoute'):
        super().__init__(name)
        self._client = ActionClient(node, NavigateRoute, 'navigate_route')
        self._trigger_pub = node.create_publisher(Empty, 'mission/inspection_trigger', 10)

        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._final_result = None
        self._last_feedback_state = None
        self._already_triggered = False

    def feedback_callback(self, feedback_msg):
        """Fire the inspection trigger once per DWELLING entry, reset once the state moves on."""
        state = feedback_msg.feedback.state
        if state == 'DWELLING' and self._last_feedback_state != 'DWELLING' and not self._already_triggered:
            self._trigger_pub.publish(Empty())
            self._already_triggered = True
        if state != 'DWELLING':
            self._already_triggered = False
        self._last_feedback_state = state

    def result_callback(self, future):
        """Cache the action result once it arrives."""
        self._final_result = future.result().result

    def reset_goal_state(self):
        """Clear all per-goal state so the next tick starts a fresh goal from scratch."""
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._final_result = None
        self._last_feedback_state = None
        self._already_triggered = False

    def update(self):
        """Send the goal on first tick, then wait for it to accept, feed back, and finish."""
        if self._goal_handle is None:
            if self._send_goal_future is None:
                if not self._client.wait_for_server(timeout_sec=0.0):
                    return py_trees.common.Status.RUNNING
                self._send_goal_future = self._client.send_goal_async(
                    NavigateRoute.Goal(), feedback_callback=self.feedback_callback
                )
                return py_trees.common.Status.RUNNING

            if not self._send_goal_future.done():
                return py_trees.common.Status.RUNNING

            self._goal_handle = self._send_goal_future.result()
            if not self._goal_handle.accepted:
                self.reset_goal_state()
                return py_trees.common.Status.FAILURE

            self._result_future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(self.result_callback)
            return py_trees.common.Status.RUNNING

        if self._final_result is None:
            return py_trees.common.Status.RUNNING

        success = self._final_result.success
        self.reset_goal_state()
        return py_trees.common.Status.SUCCESS if success else py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """Cancel the outstanding goal if this behaviour is torn down while still navigating."""
        if new_status == py_trees.common.Status.INVALID and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self.reset_goal_state()
