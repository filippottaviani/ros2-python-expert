# ROS 2 Python Expert Skill (`ros2-python-expert`)

A comprehensive agent skill, architectural rulebook, and boilerplate registry for Claude Code, Cursor, Windsurf, and LLM-based coding assistants. Designed to build high-performance, deterministic, and production-grade ROS 2 packages using `rclpy`.

---

## 1. Overview

Writing Python code for ROS 2 (`rclpy`) introduces challenges rarely handled correctly by generic AI coding assistants:
- Single-threaded executors deadlocking on synchronous service/action invocations.
- Misconfigured QoS policies silently dropping messages between publishers and subscribers.
- Blocking functions (`time.sleep()`, synchronous client calls, or long timeouts) halting entire callback queues.
- Failure to expose dynamic parameters or launch file remappings.
- Missing `data_files` and `entry_points` declarations in `setup.py`, leading to broken colcon installations.

This skill equips the code assistant with strict guidelines, lifecycle state machines, thread-safe asynchronous concurrency patterns, and modular templates.

---

## 2. Directory Structure

Place this skill within your workspace repository under `.claude/skills/` (project-scoped) or inside `~/.claude/skills/` (global user profile):

```text
.claude/skills/ros2-python-expert/
├── README.md                           # Skill overview, installation, and testing instructions
├── SKILL.md                            # Primary instructions, rules, and code reference
├── references/
│   └── qos_cheat_sheet.md              # QoS compatibility matrix and policy presets
└── templates/
    ├── lifecycle_node.py               # Managed lifecycle node template (on_configure, on_activate, etc.)
    └── modular_launch.py               # Composable LaunchDescription boilerplate with parameters and namespaces
```

---

## 3. Core Architectural Rules Enforced

The assistant adheres to these architectural invariants:

### 3.1 Concurrency & Execution
- **Multi-Threaded Execution**: Always use `MultiThreadedExecutor` for production nodes that handle concurrent I/O.
- **Callback Groups**: Separate periodic control timers (`MutuallyExclusiveCallbackGroup`) from asynchronous services, actions, or incoming telemetry (`ReentrantCallbackGroup`).
- **Zero Blocking Calls**: Prohibit `time.sleep()`, synchronous `client.call()`, and blocking `wait_for_service()` inside active callbacks. Non-blocking `service_is_ready()` checks and `future.add_done_callback()` are strictly required.

### 3.2 Quality of Service (QoS)
- Explicit QoS declaration matching the data semantic (Best Effort for high-rate sensor streams, Reliable for commands, Transient Local for latched maps and static transforms).
- Full adherence to the DDS compatibility matrix documented in `references/qos_cheat_sheet.md`.

### 3.3 Managed Lifecycle Nodes
- Direct hardware drivers and critical controllers must inherit from `rclpy.lifecycle.Node`.
- State transitions (`on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, `on_shutdown`) manage resources and publishers dynamically.

### 3.4 Packaging & Launch Integration
- Every node must be properly exposed via `console_scripts` in `setup.py`.
- Dynamic resource globs for `launch/` and `config/` are automatically configured in `setup.py` and `package.xml`.
- Launch files follow modular parameters (`DeclareLaunchArgument`, `PathJoinSubstitution`, `PushRosNamespace`).

---

## 4. Setup & Integration

### Step 1: Link the Skill in `CLAUDE.md`
Add the following directive to your workspace root configuration file (`<ros2_ws>/CLAUDE.md`):

```markdown
# ROS 2 Workspace Guidelines

## Agent Directives
- When creating, refactoring, or testing ROS 2 nodes, packages, or launch files, strictly adhere to `.claude/skills/ros2-python-expert/SKILL.md`.
- Enforce strict PEP 8 compliance, English naming/docstrings, and multi-threaded executor patterns.
- Read auxiliary templates in `.claude/skills/ros2-python-expert/templates/` before writing lifecycle nodes or launch descriptions.
```

### Step 2: Set File Permissions
Ensure the assistant CLI has proper read permissions across the skill tree:

```bash
chmod -R 644 .claude/skills/ros2-python-expert/*.md
chmod -R 644 .claude/skills/ros2-python-expert/references/*.md
chmod -R 644 .claude/skills/ros2-python-expert/templates/*.py
```

---

## 5. Verification & Testing

Verify that the assistant correctly loads the skill by issuing this prompt in Claude Code:

```text
Create a ROS 2 Python node that subscribes to an IMU topic at 100 Hz, executes a PID control loop at 50 Hz, and exposes a trigger service to reset internal biases.
```

### Expected Agent Behavior:
1. Subclasses `rclpy.node.Node` with a `MultiThreadedExecutor`.
2. Assigns the IMU subscriber to a `ReentrantCallbackGroup` and the 50 Hz timer to a `MutuallyExclusiveCallbackGroup`.
3. Uses `ReliabilityPolicy.BEST_EFFORT` and `DurabilityPolicy.VOLATILE` for the IMU subscription.
4. Pre-allocates message structures to reduce Python garbage collection pauses.
5. Avoids blocking calls inside callbacks and provides complete typing annotations.

---

## 6. Build & Linting Tooling

When verifying packages generated with this skill, use the standard workspace build and lint commands:

```bash
# Build the package with symlink support
colcon build --symlink-install --packages-select <package_name>

# Run PEP 8 and static analysis
colcon test --packages-select <package_name> && colcon test-result --verbose

# Run standalone linter check
ruff check src/<package_name>
```
