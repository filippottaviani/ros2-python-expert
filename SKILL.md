---
name: ros2-python-expert
description: Advanced engineering guidelines and patterns for developing high-performance, production-grade ROS 2 Python nodes (rclpy). Trigger when writing, refactoring, or optimizing ROS 2 topics, services, actions, TF2 transforms, QoS policies, executors, callback groups, lifecycle nodes, launch files, or package configuration (setup.py, package.xml).
allowed-tools:
  - Read
  - Edit
  - Write
  - Bash(colcon build*)
  - Bash(colcon test*)
  - Bash(ros2 *)
  - Bash(pytest*)
  - Bash(ruff*)
  - Bash(flake8*)
---

# ROS 2 Python (`rclpy`) Engineering Skill

## 1. Coding Standards & Conventions
- **PEP 8 Strict Compliance**: Follow PEP 8 guidelines rigorously. Use `snake_case` for functions, methods, variables, and ROS topic/service/action names. Use `PascalCase` for classes.
- **Language**: Write all docstrings, variable names, function names, and logging messages in **English**.
- **Clean Code**: Do not include inline comments or trailing whitespace. Rely on self-explanatory naming, explicit type hints (`typing`), and concise docstrings.
- **Type Hinting**: Annotate all method signatures, class attributes, and return values.

---

## 2. Node Architecture & OOP Best Practices
- **Inheritance**: Always subclass `rclpy.node.Node` (or `rclpy.lifecycle.Node` for managed nodes). Never write procedural scripts without a class wrapper.
- **Resource Initialization**: Declare parameters first, read parameters second, initialize QoS profiles third, and instantiate communication handles and timers last.
- **Clean Shutdown**: Always handle `KeyboardInterrupt` and `ExternalShutdownException` gracefully. Explicitly call `node.destroy_node()` and `rclpy.try_shutdown()` inside a `finally` block.
- **Avoid Global State**: Encapsulate all state variables inside the Node class instance.

---

## 3. Quality of Service (QoS) Policies
Never rely on default QoS profiles without verification. Refer to `./references/qos_cheat_sheet.md` for compatibility checks and common presets.

- **Sensor Streams (LiDAR, Cameras, IMU)**:
  - `ReliabilityPolicy.BEST_EFFORT`, `HistoryPolicy.KEEP_LAST` (Depth: 1-5), `DurabilityPolicy.VOLATILE`.
- **Control Commands & Critical Telemetry**:
  - `ReliabilityPolicy.RELIABLE`, `HistoryPolicy.KEEP_LAST` (Depth: 1-10), `DurabilityPolicy.VOLATILE`.
- **Latched Configuration / Static Maps / TF Static**:
  - `ReliabilityPolicy.RELIABLE`, `HistoryPolicy.KEEP_LAST` (Depth: 1), `DurabilityPolicy.TRANSIENT_LOCAL`.

---

## 4. Concurrency, Executors & Callback Groups
Default single-threaded executors and mutual exclusion cause silent blocking and deadlocks when invoking services or actions inside subscriber callbacks.

- **Never Block a Callback**: Avoid `time.sleep()`, synchronous `client.call()`, or waiting inside callbacks (`wait_for_service`).
- **Callback Groups**:
  - Use `ReentrantCallbackGroup` when multiple callbacks or action steps must run concurrently without blocking each other.
  - Use separate `MutuallyExclusiveCallbackGroup` instances to isolate deterministic high-frequency control loops from slow I/O or service calls.
- **Asynchronous Service Calls**: Always check `client.service_is_ready()` before calling `client.call_async()`, combining it with `future.add_done_callback(handler)`. Never invoke `rclpy.spin_until_future_complete` inside a spinning node.

---

## 5. Communication Primitives Rules

### 5.1 Topics (Publish / Subscribe)
- Pre-allocate message instances in `__init__` for high-frequency loops (≥ 50 Hz) to eliminate garbage collection jitter.
- Offload heavy compute tasks (neural net inference, intensive point cloud processing) to worker threads or process pools via thread-safe bounded queues (`queue.Queue(maxsize=1)`).

### 5.2 Services (Client / Server)
- Restrict services to short, non-blocking configuration requests.
- Check service availability non-blockingly using `client.service_is_ready()` or handle initialization checks before the executor starts spinning.

### 5.3 Actions (ActionClient / ActionServer)
- Use for long-running, preemptable goals that provide periodic feedback.
- Always attach both `ActionServer` and `ActionClient` to a `ReentrantCallbackGroup` running under a `MultiThreadedExecutor`.
- Check `goal_handle.is_cancel_requested` at each loop cycle inside `execute_callback`. On preemption, transition state using `goal_handle.canceled()` and exit immediately.

---

## 6. Lifecycle Management & Managed Nodes
When building nodes that interface directly with hardware drivers, state estimators, or safety-critical controllers, use Managed Lifecycle Nodes instead of standard nodes.

- **Reference Implementation**: Inspect and follow `./templates/lifecycle_node.py`.
- **State Machine Rules**:
  - Allocate resources and publishers during `on_configure()`.
  - Start timers and hardware transmission during `on_activate()`.
  - Pause timers and neutralize commands during `on_deactivate()`.
  - Destroy communication interfaces and free memory during `on_cleanup()`.
- Use `create_lifecycle_publisher()` and verify `pub.is_activated` before publishing.

---

## 7. Modular Launch System
Launch scripts must be modular, deterministic, and easily composable across environments (simulation vs real hardware).

- **Reference Implementation**: Inspect and follow `./templates/modular_launch.py`.
- **Best Practices**:
  - Expose environment parameters via `DeclareLaunchArgument`.
  - Use `PathJoinSubstitution` and `get_package_share_directory` instead of raw string concatenations.
  - Group nodes logically using `GroupAction` and apply hierarchical namespacing via `PushRosNamespace`.
  - Always set `emulate_tty=True` and `output="screen"` for structured, colored terminal logs.

---

## 8. TF2 Transforms (`tf2_ros`) Best Practices
- **Initialization**: Instantiate a single `tf2_ros.Buffer()` and `tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=False)` in `__init__`. Rely on the node's `MultiThreadedExecutor` rather than spawning an extra background thread unless strictly isolated.
- **Non-Blocking Lookups**: Inside control loops or sensor callbacks, check transform availability with `self._tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time())` or pass `timeout=Duration(seconds=0.0)` to `lookup_transform()`. Never block a high-frequency callback waiting for a future transform.
- **Exception Handling**: Wrap `lookup_transform()` in a `try...except TransformException as exc:` block (`from tf2_ros import TransformException`) and log warnings using `throttle_duration_sec=2.0`.

---

## 9. Python Packaging (`setup.py` & `package.xml`)
Whenever creating new nodes, launch files, or config files, update `setup.py` and `package.xml`:
- **`setup.py` Data Files**: Import `os` and `glob`, then dynamically include launch and YAML configuration files in `data_files`:
  ```python
  (os.path.join("share", package_name, "launch"), glob("launch/*launch.[pxy][yma]*")),
  (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
  ```
- **`setup.py` Entry Points**: Register every executable node under `entry_points={"console_scripts": ["node_name = pkg_name.module_name:main"]}`.
- **`package.xml`**: Declare all runtime dependencies using `<exec_depend>` (for example, `rclpy`, `std_msgs`, `tf2_ros`, and `launch_ros`).

---

## 10. Auxiliary Resources & Templates Registry
When generating or refactoring specific components, read and adapt the corresponding template:

- **QoS Rules & Compatibility Matrix**: `./references/qos_cheat_sheet.md`
- **Managed Lifecycle Node**: `./templates/lifecycle_node.py`
- **Modular Multi-Node Launch File**: `./templates/modular_launch.py`

---

## 11. Standard Reference Implementation

```python
import sys
from typing import List

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from example_interfaces.action import Fibonacci
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class ProductionReferenceNode(Node):
    """Reference node demonstrating thread-safe Topics, Services, and Actions in ROS 2."""

    def __init__(self) -> None:
        super().__init__("production_reference_node")

        self.declare_parameter("control_frequency", 50.0)
        self.declare_parameter("max_velocity", 2.5)

        self._control_frequency: float = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )
        self._max_velocity: float = (
            self.get_parameter("max_velocity").get_parameter_value().double_value
        )
        self._is_enabled: bool = False
        self._latest_sensor_value: float = 0.0
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

        self._cmd_pub = self.create_publisher(
            Float64,
            "control/command",
            command_qos,
            callback_group=self._control_group,
        )

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

        timer_period = 1.0 / self._control_frequency
        self._control_timer = self.create_timer(
            timer_period,
            self._control_loop_callback,
            callback_group=self._control_group,
        )

        self.get_logger().info("Node initialized and ready.")

    def _on_parameters_set(self, parameters: List[Parameter]) -> SetParametersResult:
        """Validate dynamic parameter updates before applying them."""
        for param in parameters:
            if param.name == "max_velocity":
                if param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="max_velocity must be strictly positive.",
                    )
                self._max_velocity = float(param.value)
        return SetParametersResult(successful=True)

    def _on_sensor_data(self, msg: Float64) -> None:
        """Process incoming high-frequency sensor stream."""
        self._latest_sensor_value = msg.data

    def _control_loop_callback(self) -> None:
        """Execute deterministic periodic control step."""
        if not self._is_enabled:
            return
        clamped_val = max(min(self._latest_sensor_value, self._max_velocity), -self._max_velocity)
        self._cmd_msg.data = clamped_val
        self._cmd_pub.publish(self._cmd_msg)

    def _on_enable_request(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Handle synchronous service request to enable or disable the controller."""
        self._is_enabled = request.data
        response.success = True
        response.message = f"Controller enabled state set to {self._is_enabled}."
        return response

    def trigger_external_check_async(self, state: bool) -> None:
        """Send an asynchronous service request non-blockingly."""
        if not self._srv_client.service_is_ready():
            self.get_logger().warn("External service unavailable; request aborted.")
            return
        req = SetBool.Request()
        req.data = state
        future = self._srv_client.call_async(req)
        future.add_done_callback(self._on_external_check_done)

    def _on_external_check_done(self, future: rclpy.task.Future) -> None:
        """Handle completion of the asynchronous service call."""
        try:
            response = future.result()
            self.get_logger().info(f"External check result: {response.message}")
        except Exception as exc:
            self.get_logger().error(f"Service call failed: {exc}")

    def _on_goal_request(self, goal_request: Fibonacci.Goal) -> GoalResponse:
        """Validate incoming action goal parameters."""
        if goal_request.order < 0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept cancellation requests during action execution."""
        return CancelResponse.ACCEPT

    def _execute_action_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        """Execute long-running action with feedback and preemption support."""
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        loop_rate = self.create_rate(10.0)

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i - 1]
            )
            goal_handle.publish_feedback(feedback_msg)
            loop_rate.sleep()

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result


def main(args: List[str] | None = None) -> None:
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
```

## 12. Anti-Patterns Checklist (Strict Prohibitions)

1. NO time.sleep() inside callbacks: Blocks the executor thread. Use self.create_timer() or self.create_rate() with MultiThreadedExecutor.

2. NO synchronous client.call(req) without dedicated callback groups: Triggers immediate deadlocks on standard executors.

3. NO wait_for_service() inside active callbacks: Blocks the executor thread. Use non-blocking service_is_ready() before calling call_async().

4. NO rclpy.spin_until_future_complete(self, future) inside a callback: The node is already spinning in the executor; nested spin calls fail or hang.

5. NO heavy CPU work in subscriber callbacks: Decouple and offload computation to keep callback queues drain rate high.

6. NO hardcoded topic or service strings: Use parameter-driven configuration or relative names supporting launch file remappings.

7. NO raw print() statements: Use self.get_logger() with throttle flags (throttle_duration_sec) inside control loops.