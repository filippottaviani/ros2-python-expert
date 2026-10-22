import os
import shutil
import sys
from typing import List, Optional

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from geometry_msgs.msg import TransformStamped
import message_filters
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import FluidPressure, Temperature
from tf2_ros import Buffer, TransformException, TransformListener


class AdvancedPerceptionRecorderNode(Node):
    """Production node integrating message_filters, TF2, diagnostic_updater, and rosbag2_py."""

    def __init__(self) -> None:
        super().__init__("advanced_perception_recorder_node")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("source_frame", "sensor_mount_link")
        self.declare_parameter("sync_slop_sec", 0.05)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("max_temperature_c", 75.0)
        self.declare_parameter("bag_output_uri", "/tmp/perception_telemetry_bag")
        self.declare_parameter("enable_bag_recording", True)

        self._target_frame: str = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self._source_frame: str = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )
        self._sync_slop: float = (
            self.get_parameter("sync_slop_sec").get_parameter_value().double_value
        )
        self._queue_size: int = (
            self.get_parameter("sync_queue_size").get_parameter_value().integer_value
        )
        self._max_temp_c: float = (
            self.get_parameter("max_temperature_c").get_parameter_value().double_value
        )
        self._bag_uri: str = (
            self.get_parameter("bag_output_uri").get_parameter_value().string_value
        )
        self._record_bag: bool = (
            self.get_parameter("enable_bag_recording").get_parameter_value().bool_value
        )

        self._latest_temp: float = 25.0
        self._synced_packets_count: int = 0
        self._tf_lookup_failures: int = 0

        self._reentrant_group = ReentrantCallbackGroup()
        self._diag_group = MutuallyExclusiveCallbackGroup()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )

        self._bag_writer: Optional[rosbag2_py.SequentialWriter] = None
        if self._record_bag:
            self._initialize_mcap_writer()

        self._diag_updater = Updater(self, period=1.0)
        self._diag_updater.setHardwareID("env_sensor_array_v2")
        self._diag_updater.add("Thermal & Sync Health", self._check_system_diagnostics)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._temp_sub = message_filters.Subscriber(
            self, Temperature, "sensors/temperature", qos_profile=sensor_qos
        )
        self._pressure_sub = message_filters.Subscriber(
            self, FluidPressure, "sensors/pressure", qos_profile=sensor_qos
        )

        self._approx_sync = message_filters.ApproximateTimeSynchronizer(
            [self._temp_sub, self._pressure_sub],
            queue_size=self._queue_size,
            slop=self._sync_slop,
        )
        self._approx_sync.registerCallback(self._on_synchronized_telemetry)

        self.get_logger().info("AdvancedPerceptionRecorderNode initialized.")

    def _initialize_mcap_writer(self) -> None:
        """Configure rosbag2_py SequentialWriter with MCAP storage backend."""
        if os.path.exists(self._bag_uri):
            shutil.rmtree(self._bag_uri)

        self._bag_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=self._bag_uri, storage_id="mcap"
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        self._bag_writer.open(storage_options, converter_options)

        temp_meta = rosbag2_py.TopicMetadata(
            name="sensors/temperature",
            type="sensor_msgs/msg/Temperature",
            serialization_format="cdr",
        )
        pressure_meta = rosbag2_py.TopicMetadata(
            name="sensors/pressure",
            type="sensor_msgs/msg/FluidPressure",
            serialization_format="cdr",
        )
        self._bag_writer.create_topic(temp_meta)
        self._bag_writer.create_topic(pressure_meta)

    def _on_synchronized_telemetry(
        self, temp_msg: Temperature, pressure_msg: FluidPressure
    ) -> None:
        """Process time-aligned messages, resolve non-blocking TF2, and record to MCAP."""
        self._latest_temp = temp_msg.temperature
        self._synced_packets_count += 1

        try:
            if self._tf_buffer.can_transform(
                self._target_frame,
                self._source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            ):
                transform: TransformStamped = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    self._source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.0),
                )
                _ = transform.transform.translation.z
            else:
                self._tf_lookup_failures += 1
                self.get_logger().warn(
                    f"Transform {self._source_frame} -> {self._target_frame} unavailable.",
                    throttle_duration_sec=2.0,
                )
        except TransformException as exc:
            self._tf_lookup_failures += 1
            self.get_logger().warn(
                f"TF2 lookup exception: {exc}",
                throttle_duration_sec=2.0,
            )

        if self._bag_writer is not None:
            now_ns = self.get_clock().now().nanoseconds
            self._bag_writer.write(
                "sensors/temperature", serialize_message(temp_msg), now_ns
            )
            self._bag_writer.write(
                "sensors/pressure", serialize_message(pressure_msg), now_ns
            )

    def _check_system_diagnostics(
        self, stat: DiagnosticStatusWrapper
    ) -> DiagnosticStatusWrapper:
        """Populate diagnostic status wrapper for /diagnostics aggregation."""
        if self._latest_temp > self._max_temp_c:
            stat.summary(
                DiagnosticStatus.ERROR,
                f"Overheating detected: {self._latest_temp:.1f} C exceeds limit!",
            )
        elif self._tf_lookup_failures > 10:
            stat.summary(
                DiagnosticStatus.WARN,
                "Degraded spatial tracking: recurring TF2 lookup misses.",
            )
        else:
            stat.summary(DiagnosticStatus.OK, "Perception and recording nominal.")

        stat.add("Latest Temperature (C)", f"{self._latest_temp:.2f}")
        stat.add("Synchronized Packets", str(self._synced_packets_count))
        stat.add("TF2 Lookup Failures", str(self._tf_lookup_failures))
        return stat


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the advanced perception and recorder node."""
    rclpy.init(args=args)
    node = AdvancedPerceptionRecorderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main(sys.argv)
