"""Publish the fixed FR3 home state for ROS-distribution-neutral local tests."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MockJointStateNode(Node):
    def __init__(self):
        super().__init__("mock_fr3_joint_state")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("joint_names", [f"fr3_joint{i}" for i in range(1, 8)])
        self.declare_parameter(
            "home_joint_positions",
            [0.51199203, 0.1014329, 0.0, -2.356, 0.0, 1.571, 0.785],
        )
        self._publisher = self.create_publisher(
            JointState, self.get_parameter("joint_state_topic").value, 10)
        self.create_timer(1.0 / self.get_parameter("publish_rate_hz").value, self._tick)

    def _tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.get_parameter("joint_names").value)
        msg.position = list(self.get_parameter("home_joint_positions").value)
        msg.velocity = [0.0] * len(msg.name)
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

