# ROS 2 Foxy Notes

Foxy is end-of-life (June 2023, Python 3.8). Use these notes only to maintain existing code; new work should target Humble or Jazzy. The templates in this skill are **not** Foxy-compatible, and CI skips them on Foxy (`SKIP` in `scripts/check_templates.py`, one entry per row below). The Foxy job still runs lint and checks `modular_launch.py`.

Evidence: CI (the Foxy job fails at smoke-check and integration test without the skip-list), the `foxy` branches of `ros2/rclpy` and `ros2/launch_ros`, and the ROS apt index for Ubuntu Focal.

## What breaks and the replacement

| Problem on Foxy | Replacement |
| :--- | :--- |
| Python 3.8: `list[str]`, `tuple[float, int]`, `X \| None` in signatures raise `TypeError` at import | Add `from __future__ import annotations`, or use `typing.List` / `Optional` |
| `rclpy.try_shutdown()` does not exist | `if rclpy.ok(): rclpy.shutdown()` in `finally` |
| `rclpy.lifecycle` does not exist (no Python lifecycle nodes) | Write the node in C++ (`rclcpp_lifecycle`) or model the states by hand |
| `rosbag2_py` is not packaged (`ros-foxy-rosbag2-py` is absent from the apt index) | Record with `ros2 bag record` or the C++ API |

## Present on Foxy (checked in the sources)

`ExternalShutdownException`, `Node.create_rate` / `destroy_rate`, `Client.remove_pending_request`, `declare_parameter(...)` returning a `Parameter` (so `.value` works), `qos_profile_sensor_data`, `rclpy.qos_event`, `async def` callbacks, and in `launch_ros`: `SetParameter`, `FindPackageShare`, `LifecycleNode`, `OnStateTransition(start_state=...)`.

## Not verified

Behaviour of these APIs at runtime on Foxy (CI only imports and instantiates), and Foxy's `ament_flake8`, which is plain flake8 like Humble's.
