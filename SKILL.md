---
name: ros2-python-expert
description: Engineering guidelines and templates for production ROS 2 Python (rclpy) nodes on Humble. Trigger when writing, refactoring or debugging rclpy topics, services, actions, TF2, QoS, executors, callback groups, parameters, lifecycle nodes, message_filters, diagnostic_updater, rosbag2_py, RViz markers, launch files, launch_testing, or ament_python packaging (setup.py, package.xml). Python only; for C++, Nav2, MoveIt or ros2_control use ros2-engineering-skills.
allowed-tools:
  - Read
  - Edit
  - Write
  - Bash(colcon build*)
  - Bash(colcon test*)
  - Bash(ros2 *)
  - Bash(launch_test*)
  - Bash(python3 scripts/check_templates.py)
  - Bash(pytest*)
  - Bash(ruff*)
  - Bash(flake8*)
---

# ROS 2 Python (`rclpy`) Engineering Skill

Target: **ROS 2 Humble, Python 3.10** (`/opt/ros/humble`). Verify an rclpy API against the installed source before relying on it. On newer distros some things differ: `create_timer(autostart=...)` and `EventsExecutor` do not exist in Humble.

## 1. Coding Standards
- PEP 8 with a 99-column limit (what `ament_flake8` enforces). `snake_case` for functions, variables and ROS names, `PascalCase` for classes.
- English for docstrings, identifiers and log messages.
- Full type hints on signatures and attributes. Use native generics (`list[str]`, `X | None`), not `typing.List`/`Optional`.
- Docstrings describe behaviour. Add a comment only for a non-obvious *why* (a driver quirk, a QoS constraint, a ceiling); never to restate the code.

## 2. Node Architecture
- Always subclass `rclpy.node.Node` (or `rclpy.lifecycle.Node`). No procedural scripts.
- `__init__` order: parameters, QoS profiles, communication handles and timers last.
- Declare and read in one line: `self._rate: float = self.declare_parameter("rate_hz", 10.0).value`. Do not redeclare `use_sim_time`; every node already has it.
- Parameter validation: use `ParameterDescriptor(floating_point_range=[FloatingPointRange(from_value=..., to_value=...)])` or `read_only=True` instead of hand-written checks. An `add_on_set_parameters_callback` then only applies the new value.
- Shutdown: catch `KeyboardInterrupt` and `ExternalShutdownException`; in `finally` call `executor.shutdown()`, `node.destroy_node()`, `rclpy.try_shutdown()`. Override `destroy_node()` to close files/bags you own.
- State lives on the instance, never in module globals.
- Time: use `self.get_clock().now()`, never `time.time()`, so `use_sim_time` works.
- **Executor choice**: a node with a single timer or only fast callbacks uses `rclpy.spin(node)`. Use `MultiThreadedExecutor` only when callback groups must really run concurrently (services/actions that wait, a control loop beside slow I/O); it has real overhead.

## 3. Quality of Service
Never trust default QoS without checking. Full matrix and presets: `./references/qos_cheat_sheet.md`.

- **Sensor streams**: `qos_profile_sensor_data` (BEST_EFFORT, KEEP_LAST 5, VOLATILE) or an explicit profile with depth 1-5.
- **Commands and critical telemetry**: RELIABLE, KEEP_LAST 1-10, VOLATILE.
- **Latched config, maps, static TF**: RELIABLE, KEEP_LAST 1, TRANSIENT_LOCAL.
- A silent no-data problem is almost always a QoS mismatch: run `ros2 topic info -v <topic>` and compare both endpoints. In code, log it with `SubscriptionEventCallbacks(incompatible_qos=cb)` from `rclpy.qos_event`.
- Safety-critical topics: set `deadline` and `liveliness` and handle the matching event callbacks instead of polling timestamps.

## 4. Concurrency, Callback Groups, Thread Safety
- **Never block a callback**: no `time.sleep()`, no `client.call()`, no `wait_for_service()`, no `spin_until_future_complete()`.
- **Callback groups**: `ReentrantCallbackGroup` for callbacks that may overlap; a separate `MutuallyExclusiveCallbackGroup` to isolate a deterministic control timer from slow I/O.
- **Calling a service from a callback** (verified on Humble):
  - Preferred: `async def` callback and `response = await client.call_async(req)`. The awaiting callback and the client **must not share a `MutuallyExclusiveCallbackGroup`** or the response can never be dispatched (deadlock). Put the client in a reentrant group.
  - Alternative: `client.call_async(req).add_done_callback(handler)` after checking `client.service_is_ready()`.
  - On timeout call `future.cancel()` **and** `client.remove_pending_request(future)`, or pending requests leak.
- **Thread safety**: with a multi-threaded executor, callbacks run in parallel. Replacing one attribute is safe; keep related values together by replacing one tuple (`self._latest = (value, stamp_ns)`). Read-modify-write (`+=`, append-then-read, check-then-act) needs a `threading.Lock`.

## 5. Communication Primitives
### 5.1 Topics
- Pre-allocate message objects for loops of 50 Hz and above.
- Move heavy compute (inference, point clouds) to a worker thread fed by `queue.Queue(maxsize=1)`.
- A controller must fail safe: command zero when its input goes stale (see `templates/reference_node.py`) and when it is disabled.

### 5.2 Services
Short, non-blocking configuration requests only. Anything long is an action.

### 5.3 Actions
- Long-running, preemptable goals with feedback.
- Server and client in a `ReentrantCallbackGroup`, run under a `MultiThreadedExecutor`.
- Check `goal_handle.is_cancel_requested` every loop; on preemption call `goal_handle.canceled()` and return.
- A `create_rate()` created per goal must be released with `destroy_rate()` in `finally`.

## 6. Lifecycle Nodes
Use for hardware drivers, estimators and safety-critical controllers. Template: `./templates/lifecycle_node.py`.

- `on_configure`: create publishers, timers (cancelled) and read parameters.
- `on_activate`: start timers; call `super().on_activate(state)` to activate lifecycle publishers.
- `on_deactivate`: stop timers; neutralise commands.
- `on_cleanup`: destroy everything from `on_configure`. Delegate `on_shutdown` and `on_error` to it so a later `configure` never duplicates resources.
- Publish only when `pub.is_activated`.
- A lifecycle node started by a launch file stays UNCONFIGURED until something triggers it; `modular_launch.py` shows the `ChangeState` events.

## 7. Launch
Template: `./templates/modular_launch.py`.

- Expose environment choices with `DeclareLaunchArgument`; build paths with `PathJoinSubstitution` and `FindPackageShare` (resolved at launch time), not `get_package_share_directory` or string concatenation.
- Group with `GroupAction` + `PushRosNamespace`. **Do not also pass `namespace=` to nodes inside the group** or the namespace doubles (`/robot_0/robot_0/node`). `LifecycleNode` requires the argument, so pass `namespace=""`.
- Set `use_sim_time` once with `SetParameter(name="use_sim_time", value=...)` inside the group instead of on every node.
- `output="screen"` and `emulate_tty=True` for readable logs.

## 8. TF2
- One `Buffer()` and one `TransformListener(self._tf_buffer, self, spin_thread=False)` per node; rely on the node's executor.
- In callbacks use `lookup_transform(target, source, rclpy.time.Time(), timeout=Duration(seconds=0.0))` inside `try ... except TransformException` (`from tf2_ros import TransformException`). A zero timeout never blocks, so a separate `can_transform` call is redundant.
- Log failures with `throttle_duration_sec=2.0`.

## 9. Packaging
Generate, do not hand-write: `ros2 pkg create --build-type ament_python --node-name <node> <pkg>`. Then add launch/config `data_files`, entry points and `<exec_depend>`s. Details, required files, custom interfaces and build commands: `./references/packaging.md`.

## 10. Testing
- Smoke-check the templates: `source /opt/ros/humble/setup.bash && python3 scripts/check_templates.py`.
- Integration test pattern (service, topic and exit-code checks via `launch_testing`): `templates/launch_integration_test.py`, run with `launch_test templates/launch_integration_test.py`.
- In tests spin with the **default** context (`rclpy.init()`). A custom `rclpy.Context()` needs an executor bound to it; a bare `rclpy.spin_once(node)` raises `AttributeError: __enter__`.
- Use a distinct `ROS_DOMAIN_ID` per run to avoid cross-talk.
- Ship `ament_flake8`, `ament_pep257` and `ament_copyright` tests (generated by `ros2 pkg create`) and run `colcon test`.
- More depth (mocks, simulation, replay): see the `robotics-testing` skill.

## 11. Templates and References Registry
Read and adapt the matching file before generating code.

| Need | File |
| :--- | :--- |
| Topics + service + action + parameters in one node (reference implementation) | `templates/reference_node.py` |
| Async service client/server with timeouts | `templates/service_server_client_node.py` |
| Preemptable ActionServer + ActionClient | `templates/action_server_client_node.py` |
| Managed lifecycle node | `templates/lifecycle_node.py` |
| message_filters sync + TF2 + diagnostic_updater + rosbag2 MCAP | `templates/advanced_perception_recorder_node.py` (needs `ros-humble-rosbag2-storage-mcap` when recording) |
| RViz `MarkerArray` publisher | `templates/rviz_marker_publisher_node.py` |
| Modular launch with namespace, sim time, lifecycle activation | `templates/modular_launch.py` |
| launch_testing integration test | `templates/launch_integration_test.py` |
| QoS compatibility matrix and presets | `references/qos_cheat_sheet.md` |
| ament_python packaging, custom interfaces, build/test | `references/packaging.md` |
| Template smoke check | `scripts/check_templates.py` |

## 12. Anti-Patterns
1. `time.sleep()` in a callback: blocks an executor thread. Use a timer or `create_rate()` under a multi-threaded executor.
2. Synchronous `client.call()` inside a callback: deadlocks.
3. `wait_for_service()` in an active callback: use `service_is_ready()`.
4. `spin_until_future_complete()` inside a callback: the node is already spinning.
5. Heavy CPU work in subscriber callbacks: offload it.
6. Hardcoded topic or service names: use relative names so launch remappings work, or parameters.
7. `print()`: use `self.get_logger()`, with `throttle_duration_sec` in loops.
8. An `await`ing callback and its client in the same mutually exclusive group: the response is never dispatched.
9. Deleting a user-supplied path (for example a bag directory) to "start clean": write to a new timestamped path instead.
10. `namespace=` on nodes already inside `PushRosNamespace`: the namespace is applied twice.
