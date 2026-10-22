import os
import sys
import time
import unittest

import launch
from launch.actions import ExecuteProcess, TimerAction
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

NODE_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'reference_node.py')
MAX_VELOCITY = 2.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the reference node as a plain process and signal readiness for active tests."""
    dut_process = ExecuteProcess(
        cmd=[
            sys.executable,
            '-u',
            NODE_SCRIPT,
            '--ros-args',
            '-p',
            f'max_velocity:={MAX_VELOCITY}',
        ],
        output='screen',
    )
    return launch.LaunchDescription(
        [
            dut_process,
            TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )


class TestReferenceNodeActive(unittest.TestCase):
    """Active tests verifying service activation, clamping and the stale-sensor watchdog."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls._test_node = Node('integration_test_harness')

    @classmethod
    def tearDownClass(cls) -> None:
        cls._test_node.destroy_node()
        rclpy.try_shutdown()

    def _spin_until(self, predicate, timeout_sec: float, on_tick=None) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            if on_tick is not None:
                on_tick()
            rclpy.spin_once(self._test_node, timeout_sec=0.05)
        return predicate()

    def test_enable_service_and_clamped_control_output(self) -> None:
        """Enabling the controller clamps sensor input to max_velocity."""
        received: list[float] = []
        sub = self._test_node.create_subscription(
            Float64, 'control/command', lambda msg: received.append(msg.data), 10
        )
        pub = self._test_node.create_publisher(Float64, 'sensor/data', 10)
        client = self._test_node.create_client(SetBool, 'control/enable')
        self.addCleanup(self._test_node.destroy_subscription, sub)
        self.addCleanup(self._test_node.destroy_publisher, pub)
        self.addCleanup(self._test_node.destroy_client, client)

        self.assertTrue(client.wait_for_service(timeout_sec=5.0), 'control/enable unavailable.')
        future = client.call_async(SetBool.Request(data=True))
        self.assertTrue(self._spin_until(future.done, 5.0), 'Enable call timed out.')
        self.assertTrue(future.result().success)

        sensor_msg = Float64(data=10.0)
        clamped = self._spin_until(
            lambda: bool(received) and abs(received[-1] - MAX_VELOCITY) < 1e-3,
            timeout_sec=3.0,
            on_tick=lambda: pub.publish(sensor_msg),
        )
        self.assertTrue(clamped, f'Expected {MAX_VELOCITY}, got {received[-3:]}.')

        stale = self._spin_until(lambda: received[-1] == 0.0, timeout_sec=3.0)
        self.assertTrue(stale, 'Stale sensor did not command zero.')


@launch_testing.post_shutdown_test()
class TestReferenceNodeShutdown(unittest.TestCase):
    """Post-shutdown tests verifying clean process termination."""

    def test_exit_codes(self, proc_info) -> None:
        """Ensure all launched processes exited with code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)
