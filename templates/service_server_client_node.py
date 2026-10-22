import sys

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger


class AsyncServiceOrchestratorNode(Node):
    """Production template for non-blocking Service Server and Client orchestration."""

    def __init__(self) -> None:
        super().__init__("async_service_orchestrator_node")

        self._check_period: float = self.declare_parameter("health_check_period_sec", 2.0).value
        self._timeout_sec: float = self.declare_parameter("service_timeout_sec", 1.0).value

        self._system_armed: bool = False
        self._pending_futures: dict[Future, float] = {}

        self._reentrant_group = ReentrantCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()

        self._arming_server = self.create_service(
            SetBool,
            "orchestrator/set_armed",
            self._on_set_armed_request,
            callback_group=self._reentrant_group,
        )

        self._diagnostics_client = self.create_client(
            Trigger,
            "hardware/verify_readiness",
            callback_group=self._reentrant_group,
        )

        self._poll_timer = self.create_timer(
            self._check_period,
            self._on_periodic_health_check,
            callback_group=self._timer_group,
        )

        self.get_logger().info("AsyncServiceOrchestratorNode initialized successfully.")

    def _on_set_armed_request(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Handle incoming service requests swiftly without blocking I/O."""
        if request.data and not self._diagnostics_client.service_is_ready():
            response.success = False
            response.message = "Cannot arm: hardware readiness service is offline."
            self.get_logger().warn(response.message)
            return response

        self._system_armed = request.data
        response.success = True
        response.message = f"System armed state transitioned to {self._system_armed}."
        self.get_logger().info(response.message)
        return response

    def _on_periodic_health_check(self) -> None:
        """Dispatch asynchronous service calls and prune timed-out futures."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        expired_futures = [
            fut
            for fut, start_time in self._pending_futures.items()
            if (now_sec - start_time) > self._timeout_sec
        ]
        for fut in expired_futures:
            fut.cancel()
            self._diagnostics_client.remove_pending_request(fut)
            self._pending_futures.pop(fut, None)
            self.get_logger().error("Asynchronous service request timed out and was canceled.")

        if not self._diagnostics_client.service_is_ready():
            self.get_logger().warn(
                "Hardware readiness service not available.",
                throttle_duration_sec=5.0,
            )
            return

        request = Trigger.Request()
        future = self._diagnostics_client.call_async(request)
        self._pending_futures[future] = now_sec
        future.add_done_callback(self._on_readiness_response)

    def _on_readiness_response(self, future: Future) -> None:
        """Process asynchronous service response safely inside the reentrant group."""
        self._pending_futures.pop(future, None)
        if future.cancelled():
            return

        try:
            result: Trigger.Response | None = future.result()
            if result is not None and result.success:
                self.get_logger().debug(f"Hardware check OK: {result.message}")
            else:
                self._system_armed = False
                msg = result.message if result is not None else "Empty response"
                self.get_logger().error(f"Hardware check failed ({msg}); disarming system.")
        except Exception as exc:
            self._system_armed = False
            self.get_logger().error(f"Service invocation raised exception: {exc}")


def main(args: list[str] | None = None) -> None:
    """Entry point for the asynchronous service orchestrator node."""
    rclpy.init(args=args)
    node = AsyncServiceOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
