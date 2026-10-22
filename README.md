# ROS 2 Python Expert Skill (`ros2-python-expert`)

Agent skill, rulebook and template registry for building production-grade ROS 2 **Humble and Jazzy** packages with `rclpy`. It targets the mistakes generic assistants make most often:

- deadlocks from synchronous service/action calls and shared mutually exclusive callback groups;
- QoS mismatches that silently drop messages;
- blocking calls in callbacks;
- `ament_python` packages that build but do not install (missing marker, `setup.cfg`, `data_files`);
- doubled launch namespaces and lifecycle nodes that never activate.

## Layout

```text
ros2-python-expert/
├── SKILL.md                       # Rules and the template registry (what the agent reads)
├── references/
│   ├── qos_cheat_sheet.md         # QoS compatibility matrix, presets, debugging
│   ├── packaging.md               # ament_python files, custom interfaces, build/test
│   └── foxy.md                    # What breaks on EOL Foxy and the replacements
├── templates/
│   ├── reference_node.py          # Topics + service + action + parameters
│   ├── service_server_client_node.py
│   ├── action_server_client_node.py
│   ├── lifecycle_node.py
│   ├── advanced_perception_recorder_node.py   # message_filters, TF2, diagnostics, rosbag2
│   ├── rviz_marker_publisher_node.py
│   ├── modular_launch.py
│   └── launch_integration_test.py
├── scripts/
│   └── check_templates.py         # Compile + instantiate every template (+ MCAP when available)
├── evals/
│   ├── evals.json                 # 3 task prompts with known traps (skill-creator schema)
│   └── trigger_cases.json         # Should / should-not trigger queries
└── .github/workflows/check.yml    # CI: Humble, Jazzy, Foxy (partial)
```

Install under `~/.claude/skills/` (global) or `<workspace>/.claude/skills/` (project). The skill activates from its `description`; no `CLAUDE.md` entry is needed.

## Verify

```bash
source /opt/ros/humble/setup.bash
python3 scripts/check_templates.py
launch_test templates/launch_integration_test.py
```

Prompt to confirm the skill is used:

```text
Create a ROS 2 Python node that subscribes to an IMU topic at 100 Hz, runs a PID loop at 50 Hz, and exposes a trigger service to reset internal biases.
```

Expected: `Node` subclass, sensor QoS on the IMU subscription, control timer isolated in its own callback group, no blocking calls, full type hints, and a package created with `ros2 pkg create`.

## Related skills

Scope is rclpy/Python only. Nav2, MoveIt, ros2_control and C++ belong to `ros2-engineering-skills`; test strategy beyond the integration template belongs to `robotics-testing`.
