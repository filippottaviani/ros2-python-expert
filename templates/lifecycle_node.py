import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecyclePublisher, State, TransitionCallbackReturn
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.timer import Timer
from std_msgs.msg import Float64


class ManagedControllerNode(LifecycleNode):
    """Production-grade Lifecycle Node for hardware or control loop management."""

    def __init__(self, node_name: str = "managed_controller_node") -> None:
        super().__init__(node_name)
        self.declare_parameter("publish_rate_hz", 100.0)
        self._pub: LifecyclePublisher | None = None
        self._timer: Timer | None = None
        self._msg = Float64()
        self.get_logger().info("Lifecycle Node constructed. State: UNCONFIGURED")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Allocate resources; parameters are re-read so a reconfigure picks up new values."""
        self.get_logger().info(f"Configuring from {state.label}...")
        publish_rate_hz: float = self.get_parameter("publish_rate_hz").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_lifecycle_publisher(Float64, "managed/output", qos)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer_tick)
        self._timer.cancel()

        self.get_logger().info("Configured successfully. State: INACTIVE")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate lifecycle publishers and start the timer."""
        self.get_logger().info(f"Activating from {state.label}...")
        if self._timer is not None:
            self._timer.reset()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timer so no further commands are issued."""
        self.get_logger().info(f"Deactivating from {state.label}...")
        if self._timer is not None:
            self._timer.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy everything created in on_configure; also used by on_shutdown and on_error."""
        self.get_logger().info(f"Cleaning up from {state.label}...")
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release resources on the terminal transition."""
        return self.on_cleanup(state)

    def on_error(self, state: State) -> TransitionCallbackReturn:
        """Release resources so a later configure does not duplicate them."""
        self.get_logger().error(f"Error triggered while in state {state.label}.")
        return self.on_cleanup(state)

    def _on_timer_tick(self) -> None:
        """Publish control data only while active."""
        if self._pub is not None and self._pub.is_activated:
            self._msg.data = 1.0
            self._pub.publish(self._msg)


def main(args: list[str] | None = None) -> None:
    """Entry point for the managed lifecycle node."""
    rclpy.init(args=args)
    node = ManagedControllerNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
