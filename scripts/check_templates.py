"""Smoke-check every template: compile, instantiate nodes, walk the lifecycle, build the launch.

Usage: source /opt/ros/humble/setup.bash && python3 scripts/check_templates.py
"""
import importlib.util
import inspect
import os
import pathlib
import py_compile
import sys
import tempfile

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
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
# Templates proven incompatible with a distro by CI, with the reason. Mirror every entry in
# references/<distro>.md; never add one without a failing CI log behind it.
SKIP: dict = {}


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


def has_package(name: str) -> bool:
    try:
        get_package_prefix(name)
        return True
    except PackageNotFoundError:
        return False


def main() -> int:
    skip = SKIP.get(os.environ.get("ROS_DISTRO", ""), {})
    if len(sys.argv) == 3 and sys.argv[1] == "--is-skipped":
        return 0 if sys.argv[2] in skip else 1
    for name, reason in skip.items():
        print(f"SKIP {name} ({reason})")

    for path in sorted(TEMPLATES.glob("*.py")):
        if path.stem not in skip:
            py_compile.compile(str(path), doraise=True)
    print("compile: OK")

    bag_dir = tempfile.mkdtemp(prefix="check_templates_")
    record_bag = has_package("rosbag2_storage_mcap")
    rclpy.init(args=[
        "--ros-args",
        "-p", f"enable_bag_recording:={str(record_bag).lower()}",
        "-p", f"bag_output_dir:={bag_dir}",
    ])
    try:
        for name in NODE_FILES:
            if name in skip:
                continue
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

    if record_bag and "advanced_perception_recorder_node" not in skip:
        bags = list(pathlib.Path(bag_dir).glob("perception_*/*.mcap"))
        assert bags, f"no .mcap written under {bag_dir}"
        print(f"mcap: OK ({bags[0].name})")
    else:
        print("mcap: not checked (rosbag2_storage_mcap not installed)")

    if "modular_launch" in skip:
        return 0
    description = load("modular_launch").generate_launch_description()
    assert description.entities, "empty launch description"
    print("launch modular_launch: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
