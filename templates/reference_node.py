import sys

import rclpy
from example_interfaces.action import Fibonacci
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class ProductionReferenceNode(Node):
    """Reference node demonstrating thread-safe Topics, Services, and Actions in ROS 2."""

    def __init__(self) -> None:
        super().__init__("production_reference_node")

        self._control_frequency: float = self.declare_parameter("control_frequency", 50.0).value
        self._max_velocity: float = self.declare_parameter(
            "max_velocity",
            2.5,
            ParameterDescriptor(
                description="Symmetric command clamp [m/s].",
                floating_point_range=[FloatingPointRange(from_value=0.01, to_value=50.0)],
            ),
        ).value
        self._sensor_timeout_sec: float = self.declare_parameter("sensor_timeout_sec", 0.5).value
        self._is_enabled: bool = False
        # (value, receive time in ns) is replaced as one tuple, so readers never see a torn pair.
        self._latest_sensor: tuple[float, int] = (0.0, 0)
        self._cmd_msg = Float64()

        self.add_on_set_parameters_callback(self._on_parameters_set)

        self._reentrant_group = ReentrantCallbackGroup()
        self._control_group = MutuallyExclusiveCallbackGroup()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sensor_sub = self.create_subscription(
            Float64,
            "sensor/data",
            self._on_sensor_data,
            sensor_qos,
            callback_group=self._reentrant_group,
        )
        self._cmd_pub = self.create_publisher(Float64, "control/command", command_qos)
        self._enable_srv = self.create_service(
            SetBool,
            "control/enable",
            self._on_enable_request,
            callback_group=self._reentrant_group,
        )
        self._srv_client = self.create_client(
            SetBool,
            "external_system/ready",
            callback_group=self._reentrant_group,
        )
        self._action_server = ActionServer(
            self,
            Fibonacci,
            "execute_sequence",
            execute_callback=self._execute_action_callback,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._reentrant_group,
        )
        self._control_timer = self.create_timer(
            1.0 / self._control_frequency,
            self._control_loop_callback,
            callback_group=self._control_group,
        )

        self.get_logger().info("Node initialized and ready.")

    def _on_parameters_set(self, parameters: list[Parameter]) -> SetParametersResult:
        """Apply dynamic updates; the descriptor range has already validated the value."""
        for param in parameters:
            if param.name == "max_velocity":
                self._max_velocity = float(param.value)
            elif param.name == "sensor_timeout_sec":
                self._sensor_timeout_sec = float(param.value)
        return SetParametersResult(successful=True)

    def _on_sensor_data(self, msg: Float64) -> None:
        """Store the newest sample together with its arrival time."""
        self._latest_sensor = (msg.data, self.get_clock().now().nanoseconds)

    def _publish_command(self, value: float) -> None:
        self._cmd_msg.data = value
        self._cmd_pub.publish(self._cmd_msg)

    def _control_loop_callback(self) -> None:
        """Clamp the latest sample; command zero when the sensor goes silent."""
        if not self._is_enabled:
            return
        value, stamp_ns = self._latest_sensor
        age_sec = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if age_sec > self._sensor_timeout_sec:
            self.get_logger().warn(
                "Sensor data stale; commanding zero.", throttle_duration_sec=2.0
            )
            self._publish_command(0.0)
            return
        self._publish_command(max(min(value, self._max_velocity), -self._max_velocity))

    def _on_enable_request(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Enable or disable the controller; disabling publishes a final zero command."""
        self._is_enabled = request.data
        if not self._is_enabled:
            self._publish_command(0.0)
        response.success = True
        response.message = f"Controller enabled state set to {self._is_enabled}."
        return response

    def trigger_external_check_async(self, state: bool) -> None:
        """Send an asynchronous service request without blocking the executor."""
        if not self._srv_client.service_is_ready():
            self.get_logger().warn("External service unavailable; request aborted.")
            return
        req = SetBool.Request()
        req.data = state
        future = self._srv_client.call_async(req)
        future.add_done_callback(self._on_external_check_done)

    def _on_external_check_done(self, future: Future) -> None:
        """Handle completion of the asynchronous service call."""
        try:
            self.get_logger().info(f"External check result: {future.result().message}")
        except Exception as exc:
            self.get_logger().error(f"Service call failed: {exc}")

    def _on_goal_request(self, goal_request: Fibonacci.Goal) -> GoalResponse:
        """Validate incoming action goal parameters."""
        return GoalResponse.REJECT if goal_request.order < 0 else GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept cancellation requests during action execution."""
        return CancelResponse.ACCEPT

    def _execute_action_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        """Execute a long-running goal with feedback and preemption support."""
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        result = Fibonacci.Result()
        loop_rate = self.create_rate(10.0)

        try:
            for i in range(1, goal_handle.request.order):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.sequence = feedback_msg.sequence
                    return result
                seq = feedback_msg.sequence
                seq.append(seq[i] + seq[i - 1])
                goal_handle.publish_feedback(feedback_msg)
                loop_rate.sleep()
        finally:
            self.destroy_rate(loop_rate)

        goal_handle.succeed()
        result.sequence = feedback_msg.sequence
        return result


def main(args: list[str] | None = None) -> None:
    """Entry point for the ROS 2 node."""
    rclpy.init(args=args)
    node = ProductionReferenceNode()
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


if __name__ == "__main__":
    main(sys.argv)
