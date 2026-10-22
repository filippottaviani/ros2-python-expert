# ros2-python-expert skill — maintenance

- Target is ROS 2 Humble (`/opt/ros/humble`, Python 3.10). Verify rclpy APIs against the installed source, not memory.
- `source /opt/ros/humble/setup.bash && python3 scripts/check_templates.py` — compile, instantiate every node, walk lifecycle, build the launch.
- `launch_test templates/launch_integration_test.py` — end-to-end test against `reference_node.py` (set a unique `ROS_DOMAIN_ID`).
- `ament_flake8 templates/*.py scripts/*.py` — must report "No problems found" (99 cols, final newline).
- Every new template or reference must be listed in SKILL.md §11 and the README tree, or the skill never reads it.
- CI (`.github/workflows/check.yml`) runs flake8 + check_templates + launch_test on Humble, Jazzy and Foxy. Foxy is EOL and partial: incompatible templates go in `SKIP` in `scripts/check_templates.py`, each mirrored in `references/foxy.md`, only with a failing CI log behind it.
- Evals: `evals/evals.json` (task prompts) and `evals/trigger_cases.json`. skill-creator's `run_eval.py` undercounts here because the real skill is installed globally and competes with its temp copy; probe with `claude -p` and record which skill fires instead.
- Gotchas learned: in parameter YAML the later entry wins, not the more specific one (put `/**` first); the lifecycle auto-activate `OnStateTransition` needs `start_state="configuring"`; bare `rclpy.spin_once` with a custom `rclpy.Context` crashes; `PushRosNamespace` plus `namespace=` doubles the namespace (`LifecycleNode` needs `namespace=""`); an `await`ing callback and its client in one `MutuallyExclusiveCallbackGroup` deadlocks.
- MCAP recording needs `ros-humble-rosbag2-storage-mcap`; the smoke check disables it with `enable_bag_recording:=false`.
- Scope is rclpy/Python only; Nav2, MoveIt, ros2_control and C++ belong to `ros2-engineering-skills`, test strategy to `robotics-testing`.
