import time
import unittest

import launch
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the production reference node and signal readiness for active tests."""
    dut_node = launch_ros.actions.Node(
        package="example_package",
        executable="production_reference_node",
        name="production_reference_node",
        parameters=[{"control_frequency": 50.0, "max_velocity": 2.0}],
        output="screen",
    )

    return (
        launch.LaunchDescription(
            [
                dut_node,
                TimerAction(
                    period=0.5,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ]
        ),
        {"dut_node": dut_node},
    )


class TestProductionNodeActive(unittest.TestCase):
    """Active integration tests verifying service activation and topic clamping."""

    @classmethod
    def setUpClass(cls) -> None:
        cls._context = rclpy.Context()
        rclpy.init(context=cls._context)
        cls._test_node = Node("integration_test_harness", context=cls._context)

    @classmethod
    def tearDownClass(cls) -> None:
        cls._test_node.destroy_node()
        rclpy.try_shutdown(context=cls._context)

    def test_enable_service_and_clamped_control_output(self) -> None:
        """Verify that enabling the controller clamps sensor inputs to max_velocity."""
        received_commands: list[float] = []

        sub = self._test_node.create_subscription(
            Float64,
            "control/command",
            lambda msg: received_commands.append(msg.data),
            10,
        )
        pub = self._test_node.create_publisher(Float64, "sensor/data", 10)
        client = self._test_node.create_client(SetBool, "control/enable")

        self.assertTrue(
            client.wait_for_service(timeout_sec=5.0),
            "Service control/enable did not become available.",
        )

        req = SetBool.Request()
        req.data = True
        future = client.call_async(req)

        end_time = time.time() + 5.0
        while time.time() < end_time and not future.done():
            rclpy.spin_once(self._test_node, timeout_sec=0.05)

        self.assertTrue(future.done(), "Enable service call timed out.")
        self.assertTrue(future.result().success)

        sensor_msg = Float64()
        sensor_msg.data = 10.0

        publish_deadline = time.time() + 2.0
        while time.time() < publish_deadline and len(received_commands) == 0:
            pub.publish(sensor_msg)
            rclpy.spin_once(self._test_node, timeout_sec=0.05)

        self._test_node.destroy_subscription(sub)
        self._test_node.destroy_publisher(pub)
        self._test_node.destroy_client(client)

        self.assertGreater(len(received_commands), 0, "No control command received.")
        self.assertAlmostEqual(received_commands[-1], 2.0, places=3)


@launch_testing.post_shutdown_test()
class TestProductionNodeShutdown(unittest.TestCase):
    """Post-shutdown tests verifying clean process termination."""

    def test_exit_codes(self, proc_info) -> None:
        """Ensure all launched processes exited with code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)