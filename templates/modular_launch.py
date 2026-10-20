from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Generate a production-ready launch description with configurable parameters."""

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true.",
    )
    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot_0",
        description="Top-level namespace for nodes and topics.",
    )
    enable_managed_arg = DeclareLaunchArgument(
        "enable_managed_node",
        default_value="true",
        description="Launch managed lifecycle controller if true.",
    )
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("example_package"),
                "config",
                "default_params.yaml",
            ]
        ),
        description="Full path to the ROS 2 parameters YAML file.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")
    enable_managed_node = LaunchConfiguration("enable_managed_node")
    config_file = LaunchConfiguration("config_file")

    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(robot_namespace),
            Node(
                package="example_package",
                executable="production_reference_node",
                name="production_reference_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    config_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("sensor/data", "hardware/sensor_stream"),
                    ("control/command", "actuators/target_command"),
                ],
            ),
            LifecycleNode(
                package="example_package",
                executable="managed_controller_node",
                name="managed_controller_node",
                namespace=robot_namespace,
                output="screen",
                emulate_tty=True,
                parameters=[
                    config_file,
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(enable_managed_node),
            ),
        ]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            robot_namespace_arg,
            enable_managed_arg,
            config_file_arg,
            LogInfo(msg=["Launching system in namespace: ", robot_namespace]),
            namespaced_group,
        ]
    )