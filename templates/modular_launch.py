import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace, SetParameter
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare


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
                FindPackageShare("example_package"),
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

    managed_node = LifecycleNode(
        package="example_package",
        executable="managed_controller_node",
        name="managed_controller_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[config_file],
        condition=IfCondition(enable_managed_node),
    )

    def change_state(transition_id: int) -> EmitEvent:
        return EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(managed_node),
                transition_id=transition_id,
            ),
            condition=IfCondition(enable_managed_node),
        )

    configure_managed_node = change_state(lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE)
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=managed_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[change_state(lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE)],
        ),
        condition=IfCondition(enable_managed_node),
    )

    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(robot_namespace),
            SetParameter(name="use_sim_time", value=use_sim_time),
            Node(
                package="example_package",
                executable="production_reference_node",
                name="production_reference_node",
                output="screen",
                emulate_tty=True,
                parameters=[config_file],
                remappings=[
                    ("sensor/data", "hardware/sensor_stream"),
                    ("control/command", "actuators/target_command"),
                ],
            ),
            managed_node,
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
            activate_after_configure,
            configure_managed_node,
        ]
    )
