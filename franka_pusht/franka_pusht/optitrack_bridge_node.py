"""Split a mocap4r2 RigidBodies stream into per-marker PoseStamped topics."""

import rclpy
from geometry_msgs.msg import PoseStamped
from mocap4r2_msgs.msg import RigidBodies
from rclpy.node import Node


class OptitrackBridgeNode(Node):
    def __init__(self):
        super().__init__("optitrack_bridge")
        defaults = {
            "rigid_bodies_topic": "/rigid_bodies",
            "block_rigid_body_name": "objectPushT",
            "ee_rigid_body_name": "franka-ck-calibr",
            "block_pose_topic": "/pusht/t_block_pose",
            "ee_pose_topic": "/pusht/ee_marker_pose",
            # Marker rigidly fixed to the robot's stand/base (not the moving arm).
            # Tracked live so the optitrack-world -> robot-base transform
            # self-corrects if the stand is ever bumped or repositioned, instead
            # of relying on a one-off static calibration value.
            "pole_base_rigid_body_name": "REPLACE_WITH_MOTIVE_POLE_BASE_RIGID_BODY_NAME",
            "pole_base_pose_topic": "/pusht/pole_base_pose",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.publishers_by_name = {
            self.get_parameter("block_rigid_body_name").value: self.create_publisher(
                PoseStamped, self.get_parameter("block_pose_topic").value, 10),
            self.get_parameter("ee_rigid_body_name").value: self.create_publisher(
                PoseStamped, self.get_parameter("ee_pose_topic").value, 10),
            self.get_parameter("pole_base_rigid_body_name").value: self.create_publisher(
                PoseStamped, self.get_parameter("pole_base_pose_topic").value, 10),
        }
        self.seen = set()
        self.create_subscription(RigidBodies, self.get_parameter("rigid_bodies_topic").value,
                                 self._on_rigid_bodies, 20)

    def _on_rigid_bodies(self, msg):
        for rb in msg.rigidbodies:
            publisher = self.publishers_by_name.get(rb.rigid_body_name)
            if publisher is None:
                continue
            self.seen.add(rb.rigid_body_name)
            out = PoseStamped()
            out.header = msg.header
            out.pose = rb.pose
            publisher.publish(out)
        missing = self.publishers_by_name.keys() - self.seen
        if missing:
            self.get_logger().warn(f"Never seen rigid bodies named: {sorted(missing)}",
                                   throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
