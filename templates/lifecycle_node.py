import sys
from typing import List, Optional

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from std_msgs.msg import Float64


class ManagedControllerNode(LifecycleNode):
    """Production-grade Lifecycle Node for hardware or control loop management."""

    def __init__(self, node_name: str = "managed_controller_node") -> None:
        super().__init__(node_name)
        self.declare_parameter("publish_rate_hz", 100.0)
        self._publish_rate_hz: float = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._pub: Optional[rclpy.lifecycle.LifecyclePublisher] = None
        self._timer: Optional[rclpy.timer.Timer] = None
        self._msg = Float64()
        self.get_logger().info("Lifecycle Node constructed. State: UNCONFIGURED")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Allocate resources, read static configurations, and initialize handles."""
        self.get_logger().info(f"Configuring from {state.label}...")
        self._publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_lifecycle_publisher(Float64, "managed/output", qos)

        timer_period = 1.0 / self._publish_rate_hz
        self._timer = self.create_timer(timer_period, self._on_timer_tick)
        self._timer.cancel()

        self.get_logger().info("Configured successfully. State: INACTIVE")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Enable hardware interfaces, activate lifecycle publishers, and start timers."""
        self.get_logger().info(f"Activating from {state.label}...")
        if self._timer is not None:
            self._timer.reset()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Pause hardware communication and stop timers to prevent command execution."""
        self.get_logger().info(f"Deactivating from {state.label}...")
        if self._timer is not None:
            self._timer.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy resources created during on_configure."""
        self.get_logger().info(f"Cleaning up from {state.label}...")
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Handle terminal state shutdown sequence."""
        self.get_logger().info(f"Shutting down from {state.label}...")
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        """Handle errors occurring during transitions."""
        self.get_logger().error(f"Error triggered while in state {state.label}.")
        if self._timer is not None:
            self._timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def _on_timer_tick(self) -> None:
        """Publish control data only while active."""
        if self._pub is not None and self._pub.is_activated:
            self._msg.data = 1.0
            self._pub.publish(self._msg)


def main(args: Optional[List[str]] = None) -> None:
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