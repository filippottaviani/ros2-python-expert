import sys

from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future


class PreemptableActionNode(Node):
    """Production template demonstrating both a preemptable ActionServer and an ActionClient."""

    def __init__(self) -> None:
        super().__init__('preemptable_action_node')

        self._rate_hz: float = self.declare_parameter('execution_rate_hz', 5.0).value
        self._max_order: int = self.declare_parameter('max_allowed_order', 25).value
        self._cancel_threshold: int = self.declare_parameter('client_cancel_threshold', 50).value

        self._reentrant_group = ReentrantCallbackGroup()
        self._active_goal_handle: ClientGoalHandle | None = None
        self._send_goal_future: Future | None = None
        self._get_result_future: Future | None = None
        self._cancel_goal_future: Future | None = None
        self._cancellation_in_progress: bool = False

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'compute_fibonacci',
            execute_callback=self._execute_goal_callback,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._reentrant_group,
        )

        self._action_client = ActionClient(
            self,
            Fibonacci,
            'compute_fibonacci',
            callback_group=self._reentrant_group,
        )

        self.get_logger().info('PreemptableActionNode initialized and ready.')

    def _on_goal_request(self, goal_request: Fibonacci.Goal) -> GoalResponse:
        """Validate incoming goal boundaries before accepting execution."""
        if goal_request.order <= 0 or goal_request.order > self._max_order:
            self.get_logger().warn(
                f'Rejected goal with invalid order {goal_request.order} (max: {self._max_order}).'
            )
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepted goal request with order {goal_request.order}.')
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept preemption and cancellation requests from any connected client."""
        self.get_logger().info('Received request to cancel active goal.')
        return CancelResponse.ACCEPT

    def _execute_goal_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        """Execute long-running goal iteratively while monitoring cancellation requests."""
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        result = Fibonacci.Result()
        loop_rate = self.create_rate(self._rate_hz)

        try:
            for i in range(1, goal_handle.request.order):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.sequence = feedback_msg.sequence
                    self.get_logger().info('Goal execution preempted and marked as CANCELED.')
                    return result

                next_val = feedback_msg.sequence[i] + feedback_msg.sequence[i - 1]
                feedback_msg.sequence.append(next_val)
                goal_handle.publish_feedback(feedback_msg)
                loop_rate.sleep()
        finally:
            self.destroy_rate(loop_rate)

        goal_handle.succeed()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Goal execution completed successfully.')
        return result

    def dispatch_goal_async(self, order: int) -> bool:
        """Send an asynchronous action goal if the server is ready."""
        if not self._action_client.server_is_ready():
            self.get_logger().warn('Action server is not ready; aborting goal dispatch.')
            return False

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self._cancellation_in_progress = False

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_client_feedback,
        )
        self._send_goal_future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future: Future) -> None:
        """Handle goal acceptance or rejection from the ActionServer."""
        goal_handle: ClientGoalHandle | None = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by the remote ActionServer.')
            return

        self._active_goal_handle = goal_handle
        self.get_logger().info('Goal accepted by server; awaiting asynchronous result.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._on_result_received)

    def _on_client_feedback(self, feedback_msg: Fibonacci.Impl.FeedbackMessage) -> None:
        """Monitor intermediate feedback and trigger async cancellation if threshold exceeded."""
        current_seq = feedback_msg.feedback.sequence
        latest_val = current_seq[-1] if current_seq else 0
        self.get_logger().info(f'Feedback sequence latest value: {latest_val}')

        if (
            latest_val > self._cancel_threshold
            and self._active_goal_handle is not None
            and not self._cancellation_in_progress
        ):
            self._cancellation_in_progress = True
            self.get_logger().warn(
                f'Value {latest_val} exceeded threshold {self._cancel_threshold}; canceling goal.'
            )
            self._cancel_goal_future = self._active_goal_handle.cancel_goal_async()
            self._cancel_goal_future.add_done_callback(self._on_cancel_done)

    def _on_cancel_done(self, future: Future) -> None:
        """Confirm that the cancellation request was processed by the server."""
        cancel_response = future.result()
        if cancel_response is not None and len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Active goal successfully transitioned to canceling state.')
        else:
            self.get_logger().error('Server failed to cancel the requested goal.')

    def _on_result_received(self, future: Future) -> None:
        """Process terminal state and final payload of the action."""
        wrapped_result = future.result()
        self._active_goal_handle = None
        if wrapped_result is None:
            self.get_logger().error('Received empty result wrapper from action server.')
            return

        status: int = wrapped_result.status
        sequence: list[int] = list(wrapped_result.result.sequence)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Action SUCCEEDED with final sequence: {sequence}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'Action CANCELED smoothly; partial sequence: {sequence}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Action ABORTED by server.')


def main(args: list[str] | None = None) -> None:
    """Entry point for the preemptable action node."""
    rclpy.init(args=args)
    node = PreemptableActionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
