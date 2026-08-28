"""Build the canonical 24-D PushT observation from tracked poses and joints."""

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray

from .math_utils import compose_pose, relative_pose


def _stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _pose(msg):
    p = msg.pose.position
    q = msg.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.w, q.x, q.y, q.z])


class ObservationNode(Node):
    def __init__(self):
        super().__init__("pusht_observation")
        defaults = {
            "control_rate_hz": 20.0, "pose_timeout_s": 0.15,
            "block_pose_topic": "/mock_optitrack/t_marker",
            "ee_pose_topic": "/mock_optitrack/ee_marker",
            "goal_pose_topic": "/mock_optitrack/goal",
            "goal_is_fixed": False,
            "fixed_goal_position": [0.5, 0.0, 0.04],
            "fixed_goal_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "joint_state_topic": "/joint_states",
            "optitrack_world_frame": "optitrack_world",
            "observation_topic": "/pusht/observation",
            "observation_valid_topic": "/pusht/observation_valid",
            "joint_names": [f"fr3_joint{i}" for i in range(1, 8)],
            "block_marker_to_object_translation": [0.0, 0.0, 0.0],
            "block_marker_to_object_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "ee_marker_to_tcp_translation": [0.0, 0.0, 0.0],
            "ee_marker_to_tcp_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            # EE pose source: real hardware has no marker on the pusher tool itself,
            # so the EE pose comes from forward kinematics (robot_base_frame ->
            # tcp_frame via /tf) composed with the optitrack-world -> robot-base
            # transform, instead of an OptiTrack marker on the tool. The
            # world->base transform itself is computed live each tick from
            # pole_base_pose_topic (a marker fixed to the robot's stand, tracked
            # continuously) composed with a fixed, one-off calibrated offset from
            # that marker's frame to the true fr3_link0 origin -- this way the
            # transform self-corrects if the stand is ever bumped/repositioned,
            # rather than silently going stale like a hardcoded constant would.
            "ee_from_tf": False,
            "robot_base_frame": "fr3_link0",
            "tcp_frame": "fr3_pusher_tcp",
            "pole_base_pose_topic": "/pusht/pole_base_pose",
            "pole_base_to_robot_base_translation": [0.0, 0.0, 0.0],
            "pole_base_to_robot_base_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.rate = self.get_parameter("control_rate_hz").value
        self.timeout = self.get_parameter("pose_timeout_s").value
        self.world = self.get_parameter("optitrack_world_frame").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.block_offset_p = np.array(self.get_parameter("block_marker_to_object_translation").value)
        self.block_offset_q = np.array(self.get_parameter("block_marker_to_object_quaternion_wxyz").value)
        self.ee_offset_p = np.array(self.get_parameter("ee_marker_to_tcp_translation").value)
        self.ee_offset_q = np.array(self.get_parameter("ee_marker_to_tcp_quaternion_wxyz").value)
        self.ee_from_tf = bool(self.get_parameter("ee_from_tf").value)
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.tcp_frame = self.get_parameter("tcp_frame").value
        self.pole_offset_p = np.array(self.get_parameter("pole_base_to_robot_base_translation").value)
        self.pole_offset_q = np.array(self.get_parameter("pole_base_to_robot_base_quaternion_wxyz").value)
        self.poses = {}
        self.joints = None
        self.joint_received_s = None
        self.tf_buffer = None
        if self.ee_from_tf:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Float32MultiArray, self.get_parameter("observation_topic").value, 10)
        self.valid_pub = self.create_publisher(Bool, self.get_parameter("observation_valid_topic").value, 10)
        subscriptions = [("block", "block_pose_topic")]
        if self.ee_from_tf:
            subscriptions.append(("pole_base", "pole_base_pose_topic"))
        else:
            subscriptions.append(("ee", "ee_pose_topic"))
        if not self.get_parameter("goal_is_fixed").value:
            subscriptions.append(("goal", "goal_pose_topic"))
        for key, parameter in subscriptions:
            self.create_subscription(PoseStamped, self.get_parameter(parameter).value,
                                     lambda msg, k=key: self._on_pose(k, msg), 20)
        self.create_subscription(JointState, self.get_parameter("joint_state_topic").value,
                                 self._on_joints, 50)
        self.create_timer(1.0 / self.rate, self._tick)

    def _on_pose(self, key, msg):
        if msg.header.frame_id != self.world:
            self.get_logger().error(f"{key} frame {msg.header.frame_id!r} != {self.world!r}")
            return
        # Motive publishes exactly (0,0,0) with quaternion (0,0,0,-1) for a rigid
        # body it knows about but cannot currently see. That is structurally
        # valid and arrives at full rate, so the freshness checks below would
        # accept it: the observation would be built from a bogus pose while
        # observation_valid still reported true, silently corrupting recorded
        # data rather than failing. A body at the exact mocap origin is never a
        # real measurement here.
        p = msg.pose.position
        if abs(p.x) < 1e-9 and abs(p.y) < 1e-9 and abs(p.z) < 1e-9:
            self.get_logger().warning(
                f"{key} is not visible to OptiTrack (all-zero placeholder pose)",
                throttle_duration_sec=5.0)
            return
        self.poses[key] = (msg, _stamp_seconds(msg.header.stamp))

    def _on_joints(self, msg):
        if len(msg.velocity) != len(msg.name):
            self.joints = None
            return
        lookup = {name: (position, velocity) for name, position, velocity in zip(msg.name, msg.position, msg.velocity)}
        if any(name not in lookup for name in self.joint_names):
            self.joints = None
            return
        self.joints = lookup
        self.joint_received_s = self.get_clock().now().nanoseconds * 1e-9

    def _lookup_ee_from_tf(self, now):
        try:
            tf = self.tf_buffer.lookup_transform(self.robot_base_frame, self.tcp_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        if abs(now - _stamp_seconds(tf.header.stamp)) > self.timeout:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([r.w, r.x, r.y, r.z])

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        required_poses = ["block"]
        if self.ee_from_tf:
            required_poses.append("pole_base")
        else:
            required_poses.append("ee")
        if not self.get_parameter("goal_is_fixed").value:
            required_poses.append("goal")
        valid = all(k in self.poses for k in required_poses) and self.joints is not None
        if valid:
            valid = all(abs(now - self.poses[k][1]) <= self.timeout for k in required_poses)
            valid = valid and abs(now - self.joint_received_s) <= self.timeout
        ee_base = None
        if valid and self.ee_from_tf:
            ee_base = self._lookup_ee_from_tf(now)
            valid = ee_base is not None
        observation = None
        if valid:
            try:
                block_marker_p, block_marker_q = _pose(self.poses["block"][0])
                if self.get_parameter("goal_is_fixed").value:
                    goal_p = np.array(self.get_parameter("fixed_goal_position").value)
                    goal_q = np.array(self.get_parameter("fixed_goal_quaternion_wxyz").value)
                else:
                    goal_p, goal_q = _pose(self.poses["goal"][0])
                block_p, block_q = compose_pose(block_marker_p, block_marker_q,
                                                self.block_offset_p, self.block_offset_q)
                if self.ee_from_tf:
                    pole_p, pole_q = _pose(self.poses["pole_base"][0])
                    world_base_p, world_base_q = compose_pose(pole_p, pole_q,
                                                              self.pole_offset_p, self.pole_offset_q)
                    ee_base_p, ee_base_q = ee_base
                    ee_p, ee_q = compose_pose(world_base_p, world_base_q, ee_base_p, ee_base_q)
                else:
                    ee_marker_p, ee_marker_q = _pose(self.poses["ee"][0])
                    ee_p, ee_q = compose_pose(ee_marker_p, ee_marker_q, self.ee_offset_p, self.ee_offset_q)
                block_rel_p, block_rel_q = relative_pose(block_p, block_q, goal_p, goal_q)
                ee_rel_p, _ = relative_pose(ee_p, ee_q, goal_p, goal_q)
                q = [self.joints[name][0] for name in self.joint_names]
                dq = [self.joints[name][1] for name in self.joint_names]
                observation = np.concatenate((block_rel_p, block_rel_q, ee_rel_p, q, dq)).astype(np.float32)
                valid = observation.shape == (24,) and np.all(np.isfinite(observation))
            except ValueError:
                valid = False
        self.valid_pub.publish(Bool(data=bool(valid)))
        if valid:
            self.pub.publish(Float32MultiArray(data=observation.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = ObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
