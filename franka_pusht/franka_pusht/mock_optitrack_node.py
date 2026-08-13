"""Publish deterministic mock OptiTrack rigid-body poses for local testing."""

import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MockOptitrackNode(Node):
    def __init__(self):
        super().__init__("mock_optitrack")
        self.declare_parameter("frame_id", "optitrack_world")
        self.declare_parameter("publish_rate_hz", 120.0)
        self._pose_publishers = {
            "block": self.create_publisher(PoseStamped, "/mock_optitrack/t_marker", 10),
            "ee": self.create_publisher(PoseStamped, "/mock_optitrack/ee_marker", 10),
            "goal": self.create_publisher(PoseStamped, "/mock_optitrack/goal", 10),
        }
        self.create_timer(1.0 / self.get_parameter("publish_rate_hz").value, self._tick)

    def _message(self, position, yaw=0.0):
        msg = PoseStamped(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        msg.pose.orientation.w = math.cos(yaw / 2.0); msg.pose.orientation.z = math.sin(yaw / 2.0)
        return msg

    def _tick(self):
        self._pose_publishers["goal"].publish(self._message((0.50, 0.00, 0.04)))
        self._pose_publishers["block"].publish(self._message((0.60, -0.10, 0.045), math.pi / 2.0))
        self._pose_publishers["ee"].publish(self._message((0.45, 0.10, 0.039), 0.0))


def main(args=None):
    rclpy.init(args=args); node = MockOptitrackNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
