---
name: ros2-python-expert
description: Engineering guidelines and templates for production ROS 2 Python (rclpy) nodes on Humble and Jazzy, with notes for legacy Foxy. Trigger when writing, refactoring or debugging rclpy topics, services, actions, TF2, QoS, executors, callback groups, parameters, lifecycle nodes, message_filters, diagnostic_updater, rosbag2_py, RViz markers, launch files, launch_testing, or ament_python packaging (setup.py, package.xml). Python only; for C++, Nav2, MoveIt or ros2_control use ros2-engineering-skills.
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

# ROS 2 Python (`rclpy`): verified gotchas

Targets Humble and Jazzy. Foxy: `references/foxy.md`.

## Style (Jazzy's `ament_flake8` enforces it; Humble's does not)
Single-quoted strings, triple-double-quoted docstrings, imports sorted by module name (google style, `import` and `from` mixed), 99 columns, native type hints (`list[str]`, `X | None`).

## Gotchas
- Create packages with `ros2 pkg create --build-type ament_python --node-name <node> <pkg>`; then add launch/config `data_files` and `<exec_depend>`s (`references/packaging.md`).
- `self._x = self.declare_parameter('x', 1.0).value`. Parameter YAML: the **later** entry wins, so put `/**` first.
- Launch: `GroupAction` + `PushRosNamespace`, and no `namespace=` on the nodes inside (it doubles; `LifecycleNode` needs `namespace=''`). Set `use_sim_time` once with `SetParameter`. Give every `DeclareLaunchArgument` a `description`.
- Lifecycle auto-activate: `OnStateTransition(..., start_state='configuring', goal_state='inactive')`, otherwise every manual `deactivate` re-activates the node. `on_shutdown`/`on_error` delegate to `on_cleanup`; publish a zero command in `on_deactivate`. Template: `templates/lifecycle_node.py`, launch: `templates/modular_launch.py`.
- Callbacks: never block (`sleep`, `client.call`, `wait_for_service`, `spin_until_future_complete`). Use `async def` + `await client.call_async()` or `add_done_callback`. An awaiting callback and its client must not share a `MutuallyExclusiveCallbackGroup` (deadlock). On timeout: `future.cancel()` and `client.remove_pending_request(future)`.
- Executor: `rclpy.spin(node)` for simple nodes; `MultiThreadedExecutor` only when callback groups must run concurrently, with `ReentrantCallbackGroup` for callbacks that may overlap.
- QoS: always explicit. Sensors `qos_profile_sensor_data`; commands RELIABLE; latched TRANSIENT_LOCAL. No data usually means a mismatch: `ros2 topic info -v <topic>` (`references/qos_cheat_sheet.md`).
- A controller commands zero when its input goes stale or it is disabled. `create_rate()` in an action goal needs `destroy_rate()` in `finally`.
- Never loop over points in Python: `sensor_msgs_py.point_cloud2.read_points_numpy`.
- Time: `self.get_clock().now()`, never `time.time()`.
- `rosbag2_py.TopicMetadata`: Jazzy requires `id`, Humble rejects it (see `templates/advanced_perception_recorder_node.py`).

## Detailed rules (read a section only when the task needs it)
`references/guidelines.md`: node architecture, QoS, concurrency and thread safety, lifecycle, launch, TF2, testing, anti-patterns. Also `references/packaging.md`, `references/qos_cheat_sheet.md`, `references/foxy.md`.

## Templates (adapt, do not rewrite)
`templates/reference_node.py` (topic+service+action+params), `service_server_client_node.py`, `action_server_client_node.py`, `lifecycle_node.py`, `advanced_perception_recorder_node.py` (message_filters, TF2, diagnostics, rosbag2), `rviz_marker_publisher_node.py`, `modular_launch.py`, `launch_integration_test.py`.

## Verify
`source /opt/ros/<distro>/setup.bash && python3 scripts/check_templates.py`; `launch_test templates/launch_integration_test.py`; `colcon test`.
