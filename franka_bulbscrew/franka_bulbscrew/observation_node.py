"""Build the canonical 25-D bulb-screwing observation from tracked poses and joints.

Layout, matching bulbscrew_mjx's get_observation() with the tactile dims removed
(the real Franka Hand has no fingertip sensors, so the sim drops them too --
see SIM_ALIGNMENT.md):

    0:3    bulb screw-tip position relative to the socket seat
    3:7    bulb orientation quaternion (w, x, y, z), in the seat frame
    7:10   end-effector position relative to the bulb neck (the grasp point)
    10     gripper opening width (both fingers, metres)
    11:18  arm joint positions
    18:25  arm joint velocities

The sim's 27-D layout has fingertip touch at 11:12; dropping it shifts arm_qpos
from 13:20 to 11:18 and arm_qvel from 20:27 to 18:25. Both sides must agree --
this is exactly the class of silent mismatch that cost days on PushT.

Frame conventions (read SIM_ALIGNMENT.md section 0 before touching any of this):
mocap4r2 publishes /rigid_bodies Y-UP in frame "map", while bulbscrew_mjx is
Z-UP. The observation is expressed relative to the socket seat, in the seat's
own frame, so what fixes its axis layout is the *seat frame's orientation* --
set by fixed_socket_quaternion_wxyz. Get that wrong and the vertical axis lands
in the wrong slot while everything still looks plausible.
"""

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray

from .math_utils import compose_pose, quaternion_multiply, quaternion_inverse, \
    quaternion_to_matrix, normalize_quaternion


def _stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _pose(msg):
    p = msg.pose.position
    q = msg.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.w, q.x, q.y, q.z])


class ObservationNode(Node):
    def __init__(self):
        super().__init__("bulbscrew_observation")
        defaults = {
            "control_rate_hz": 20.0, "pose_timeout_s": 0.15,
            "bulb_pose_topic": "/bulbscrew/bulb_pose",
            "pole_base_pose_topic": "/bulbscrew/pole_base_pose",
            "joint_state_topic": "/joint_states",
            "gripper_joint_state_topic": "/fr3_gripper/joint_states",
            "gripper_joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"],
            "optitrack_world_frame": "map",
            "observation_topic": "/bulbscrew/observation",
            "observation_valid_topic": "/bulbscrew/observation_valid",
            "joint_names": [f"fr3_joint{i}" for i in range(1, 8)],
            # Socket is bolted down and measured once, like PushT's goal. The
            # seat is the point the bulb's screw tip must reach; SEAT_OFF in the
            # sim puts it 18 mm above the socket body origin.
            "fixed_socket_position": [0.0, 0.0, 0.0],
            "fixed_socket_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "socket_seat_offset": [0.0, 0.0, 0.018],
            # Motive's bulb rigid body -> the sim's canonical bulb body frame.
            # Calibrate exactly as PushT's block was: the marker frame's origin
            # and axes are whatever Motive assigned and will NOT match the sim.
            "bulb_rigid_body_name": "bulb-trirl",
            "bulb_marker_to_object_translation": [0.0, 0.0, 0.0],
            "bulb_marker_to_object_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            # Points along the bulb body's -z axis, from bulbscrew_mjx.
            "bulb_tip_offset": -0.053,
            "bulb_neck_offset": -0.006,
            # EE pose from forward kinematics, anchored to the OptiTrack world
            # through a marker on the robot's stand (same scheme as PushT).
            "robot_base_frame": "fr3_link0",
            "tcp_frame": "fr3_hand_tcp",
            "pole_base_to_robot_base_translation": [0.0, 0.0, 0.0],
            "pole_base_to_robot_base_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        g = self.get_parameter
        self.rate = g("control_rate_hz").value
        self.timeout = g("pose_timeout_s").value
        self.world = g("optitrack_world_frame").value
        self.joint_names = list(g("joint_names").value)
        self.gripper_joint_names = list(g("gripper_joint_names").value)
        self.bulb_offset_p = np.array(g("bulb_marker_to_object_translation").value)
        self.bulb_offset_q = np.array(g("bulb_marker_to_object_quaternion_wxyz").value)
        self.tip_off = float(g("bulb_tip_offset").value)
        self.neck_off = float(g("bulb_neck_offset").value)
        self.pole_offset_p = np.array(g("pole_base_to_robot_base_translation").value)
        self.pole_offset_q = np.array(g("pole_base_to_robot_base_quaternion_wxyz").value)
        self.robot_base_frame = g("robot_base_frame").value
        self.tcp_frame = g("tcp_frame").value

        self.poses = {}
        self.joints = None
        self.joint_received_s = None
        self.gripper_width = None
        self.gripper_received_s = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Float32MultiArray, g("observation_topic").value, 10)
        self.valid_pub = self.create_publisher(Bool, g("observation_valid_topic").value, 10)
        for key, parameter in (("bulb", "bulb_pose_topic"),
                               ("pole_base", "pole_base_pose_topic")):
            self.create_subscription(PoseStamped, g(parameter).value,
                                     lambda msg, k=key: self._on_pose(k, msg), 20)
        self.create_subscription(JointState, g("joint_state_topic").value, self._on_joints, 50)
        self.create_subscription(JointState, g("gripper_joint_state_topic").value,
                                 self._on_gripper, 20)
        self.create_timer(1.0 / self.rate, self._tick)

    def _on_pose(self, key, msg):
        if msg.header.frame_id != self.world:
            self.get_logger().error(f"{key} frame {msg.header.frame_id!r} != {self.world!r}",
                                    throttle_duration_sec=5.0)
            return
        self.poses[key] = (msg, _stamp_seconds(msg.header.stamp))

    def _on_joints(self, msg):
        if len(msg.velocity) != len(msg.name):
            self.joints = None
            return
        lookup = {n: (p, v) for n, p, v in zip(msg.name, msg.position, msg.velocity)}
        if any(n not in lookup for n in self.joint_names):
            self.joints = None
            return
        self.joints = lookup
        self.joint_received_s = self._now()

    def _on_gripper(self, msg):
        lookup = dict(zip(msg.name, msg.position))
        if any(n not in lookup for n in self.gripper_joint_names):
            return
        # Total opening, matching the sim, which sums both finger joints.
        self.gripper_width = float(sum(lookup[n] for n in self.gripper_joint_names))
        self.gripper_received_s = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup_tcp(self, now):
        try:
            tf = self.tf_buffer.lookup_transform(self.robot_base_frame, self.tcp_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        if abs(now - _stamp_seconds(tf.header.stamp)) > self.timeout:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([r.w, r.x, r.y, r.z])

    def _tick(self):
        now = self._now()
        required = ["bulb", "pole_base"]
        valid = (all(k in self.poses for k in required)
                 and self.joints is not None and self.gripper_width is not None)
        if valid:
            valid = all(abs(now - self.poses[k][1]) <= self.timeout for k in required)
            valid = valid and abs(now - self.joint_received_s) <= self.timeout
            valid = valid and abs(now - self.gripper_received_s) <= self.timeout
        tcp = self._lookup_tcp(now) if valid else None
        valid = valid and tcp is not None

        observation = None
        if valid:
            try:
                observation = self._build(tcp)
                valid = observation.shape == (25,) and np.all(np.isfinite(observation))
            except ValueError:
                valid = False
        self.valid_pub.publish(Bool(data=bool(valid)))
        if valid:
            self.pub.publish(Float32MultiArray(data=observation.tolist()))

    def _build(self, tcp):
        g = self.get_parameter
        # --- bulb: marker pose -> canonical body frame -> tip and neck points ---
        marker_p, marker_q = _pose(self.poses["bulb"][0])
        bulb_p, bulb_q = compose_pose(marker_p, marker_q, self.bulb_offset_p, self.bulb_offset_q)
        # The tip and neck lie along the bulb body's own z axis, as in the sim.
        axis_z = quaternion_to_matrix(bulb_q)[:, 2]
        tip = bulb_p + axis_z * self.tip_off
        neck = bulb_p + axis_z * self.neck_off

        # --- socket seat: fixed, measured once ---
        socket_p = np.array(g("fixed_socket_position").value)
        socket_q = normalize_quaternion(np.array(g("fixed_socket_quaternion_wxyz").value))
        seat = socket_p + quaternion_to_matrix(socket_q) @ np.array(g("socket_seat_offset").value)

        # --- end effector: forward kinematics, anchored via the stand marker ---
        pole_p, pole_q = _pose(self.poses["pole_base"][0])
        base_p, base_q = compose_pose(pole_p, pole_q, self.pole_offset_p, self.pole_offset_q)
        ee_p, _ = compose_pose(base_p, base_q, tcp[0], tcp[1])

        # --- express everything in the seat frame, so the layout is Z-up like sim ---
        R_seat_inv = quaternion_to_matrix(socket_q).T
        tip_rel_seat = R_seat_inv @ (tip - seat)
        ee_rel_neck = R_seat_inv @ (ee_p - neck)
        bulb_quat_rel_seat = normalize_quaternion(
            quaternion_multiply(quaternion_inverse(socket_q), bulb_q))

        q = [self.joints[n][0] for n in self.joint_names]
        dq = [self.joints[n][1] for n in self.joint_names]
        return np.concatenate((tip_rel_seat, bulb_quat_rel_seat, ee_rel_neck,
                               [self.gripper_width], q, dq)).astype(np.float32)


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
