import math
import sys

from builtin_interfaces.msg import Duration as DurationMsg
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class RvizMarkerArrayPublisherNode(Node):
    """Production template for publishing dynamic 3D MarkerArrays to RViz."""

    def __init__(self) -> None:
        super().__init__('rviz_marker_array_publisher_node')

        self._frame_id: str = self.declare_parameter('reference_frame', 'map').value
        self._rate_hz: float = self.declare_parameter('publish_rate_hz', 10.0).value
        self._radius: float = self.declare_parameter('orbit_radius_m', 2.0).value
        self._phase_rad: float = 0.0

        vis_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._marker_pub = self.create_publisher(
            MarkerArray, 'visualization_marker_array', vis_qos
        )
        self._timer = self.create_timer(1.0 / self._rate_hz, self._on_publish_markers)
        self.get_logger().info('RvizMarkerArrayPublisherNode started.')

    def _on_publish_markers(self) -> None:
        """Construct and publish a batch of 3D markers with valid alpha and quaternion."""
        now_msg = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        for idx in range(4):
            angle = self._phase_rad + (idx * math.pi / 2.0)
            marker = Marker()
            marker.header.frame_id = self._frame_id
            marker.header.stamp = now_msg
            marker.ns = 'dynamic_obstacles'
            marker.id = idx
            marker.type = Marker.SPHERE if idx % 2 == 0 else Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = self._radius * math.cos(angle)
            marker.pose.position.y = self._radius * math.sin(angle)
            marker.pose.position.z = 0.25
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4

            marker.color.r = 0.1 if idx % 2 == 0 else 0.95
            marker.color.g = 0.45
            marker.color.b = 0.9 if idx % 2 == 0 else 0.2
            marker.color.a = 0.85

            marker.lifetime = DurationMsg(sec=0, nanosec=250_000_000)
            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)
        self._phase_rad = (self._phase_rad + 0.05) % (2.0 * math.pi)


def main(args: list[str] | None = None) -> None:
    """Entry point for the RViz MarkerArray publisher node."""
    rclpy.init(args=args)
    node = RvizMarkerArrayPublisherNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
