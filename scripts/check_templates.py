"""Smoke-check every template: compile, instantiate nodes, walk the lifecycle, build the launch.

Usage: source /opt/ros/humble/setup.bash && python3 scripts/check_templates.py
"""
import importlib.util
import inspect
import pathlib
import py_compile
import sys

import rclpy
from rclpy.node import Node

TEMPLATES = pathlib.Path(__file__).resolve().parent.parent / "templates"
NODE_FILES = [
    "reference_node",
    "action_server_client_node",
    "service_server_client_node",
    "rviz_marker_publisher_node",
    "advanced_perception_recorder_node",
    "lifecycle_node",
]


def load(name: str):
    spec = importlib.util.spec_from_file_location(name, TEMPLATES / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def node_class(module):
    return next(
        c for _, c in inspect.getmembers(module, inspect.isclass)
        if issubclass(c, Node) and c.__module__ == module.__name__
    )


def main() -> int:
    for path in sorted(TEMPLATES.glob("*.py")):
        py_compile.compile(str(path), doraise=True)
    print("compile: OK")

    rclpy.init(args=["--ros-args", "-p", "enable_bag_recording:=false"])
    try:
        for name in NODE_FILES:
            node = node_class(load(name))()
            if name == "lifecycle_node":
                for step, expected in [
                    ("trigger_configure", "inactive"),
                    ("trigger_activate", "active"),
                    ("trigger_deactivate", "inactive"),
                    ("trigger_cleanup", "unconfigured"),
                ]:
                    getattr(node, step)()
                    state = node._state_machine.current_state[1]
                    assert state == expected, f"{step}: {state} != {expected}"
            node.destroy_node()
            print(f"node {name}: OK")
    finally:
        rclpy.try_shutdown()

    description = load("modular_launch").generate_launch_description()
    assert description.entities, "empty launch description"
    print("launch modular_launch: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
