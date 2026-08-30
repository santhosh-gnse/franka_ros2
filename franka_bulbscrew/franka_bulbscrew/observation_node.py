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

# /rigid_bodies is Y-up ("map"); the driver's TF frame "optitrack" is Z-up.
# p_map = Rx(-90 deg) @ p_optitrack. See SIM_ALIGNMENT.md section 0 -- mixing
# the two silently produces a metre-scale error that looks like a real
# miscalibration.
Q_OPTITRACK_TO_MAP = np.array([0.7071068, -0.7071068, 0.0, 0.0])


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
            # Anchor from hand_eye_calibration, taken VERBATIM as that tool
            # emits it: a tf2 parent->child transform with parent fr3_link0 and
            # child optitrack, i.e.
            #     p_fr3_link0 = R(quaternion) @ p_optitrack + translation
            # so the pair is the pose of the OPTITRACK frame expressed in
            # fr3_link0. Reading it the other way round moves the socket 75 mm
            # and looks exactly like a failed calibration.
            #
            # This replaces the franka_pole_base marker chain: those markers
            # cannot be kept visible on this rig, and the anchor does not need
            # them. The cost is that the transform no longer self-corrects if
            # the robot stand is moved -- re-run the calibration if it is.
            "optitrack_to_robot_base_translation": [0.0, 0.0, 0.0],
            "optitrack_to_robot_base_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            # --- sanity gates on the finished observation ----------------------
            # A tracked pose can be live, fresh and non-zero and still be wrong.
            # In the first kinesthetic demo 41% of frames put the bulb ~1.86 m
            # from the seat -- behind the robot -- because Motive matched the
            # bulb's marker set to some other object in the volume. The zero-pose
            # check above cannot catch that: the readings jitter like a real
            # measurement. Reject anything outside the robot's reach instead of
            # recording nonsense.
            "max_bulb_distance_from_seat": 0.6,
            # With the jaws closed on the bulb, the tool must be AT the grasp
            # point. If it is not, the bulb's axial geometry or the mocap-to-
            # robot anchor is wrong, and every ee_rel_neck in the dataset is
            # off by that amount -- which is exactly what happened (see
            # FINDINGS B16). Warn rather than reject: the observation is still
            # internally consistent, and refusing to run would block the very
            # session needed to re-measure.
            "grasp_closed_width": 0.065,
            "grasp_consistency_warn": 0.03,
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
        # Everything downstream is expressed in the Y-up "map" frame that
        # /rigid_bodies uses, so fold the anchor into that frame once here:
        # invert the emitted transform to get fr3_link0 in "optitrack", then
        # rotate Z-up -> Y-up. Doing it at startup keeps the per-tick path a
        # single compose and keeps the frame conversion in exactly one place.
        he_t = np.array(g("optitrack_to_robot_base_translation").value)
        he_q = normalize_quaternion(np.array(g("optitrack_to_robot_base_quaternion_wxyz").value))
        q_base_in_optitrack = quaternion_inverse(he_q)
        p_base_in_optitrack = -(quaternion_to_matrix(q_base_in_optitrack) @ he_t)
        self.base_p, self.base_q = compose_pose(
            np.zeros(3), Q_OPTITRACK_TO_MAP, p_base_in_optitrack, q_base_in_optitrack)
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
        self.create_subscription(PoseStamped, g("bulb_pose_topic").value,
                                 lambda msg: self._on_pose("bulb", msg), 20)
        self.create_subscription(JointState, g("joint_state_topic").value, self._on_joints, 50)
        self.create_subscription(JointState, g("gripper_joint_state_topic").value,
                                 self._on_gripper, 20)
        self.create_timer(1.0 / self.rate, self._tick)

    def _on_pose(self, key, msg):
        if msg.header.frame_id != self.world:
            self.get_logger().error(f"{key} frame {msg.header.frame_id!r} != {self.world!r}",
                                    throttle_duration_sec=5.0)
            return
        # Motive publishes exactly (0,0,0) with quaternion (0,0,0,-1) for a rigid
        # body it knows about but cannot currently see. That is structurally
        # valid and arrives at full rate, so freshness checks alone would accept
        # it and the observation would be built from a bogus pose while
        # observation_valid still reported true. Reject it explicitly: a body at
        # the exact mocap origin is never a real measurement here.
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
        required = ["bulb"]
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

        # --- end effector: forward kinematics, anchored by hand-eye ---
        ee_p, _ = compose_pose(self.base_p, self.base_q, tcp[0], tcp[1])

        # --- express everything in the seat frame, so the layout is Z-up like sim ---
        R_seat_inv = quaternion_to_matrix(socket_q).T
        tip_rel_seat = R_seat_inv @ (tip - seat)
        ee_rel_neck = R_seat_inv @ (ee_p - neck)
        bulb_quat_rel_seat = normalize_quaternion(
            quaternion_multiply(quaternion_inverse(socket_q), bulb_q))

        reach = float(g("max_bulb_distance_from_seat").value)
        if np.linalg.norm(tip_rel_seat) > reach:
            self.get_logger().error(
                f"bulb pose implausible: {np.linalg.norm(tip_rel_seat):.2f} m from the seat "
                f"(limit {reach} m) -- OptiTrack is tracking something that is not the bulb",
                throttle_duration_sec=2.0)
            raise ValueError("implausible bulb pose")

        if self.gripper_width < float(g("grasp_closed_width").value):
            miss = float(np.linalg.norm(ee_rel_neck))
            if miss > float(g("grasp_consistency_warn").value):
                self.get_logger().warning(
                    f"jaws closed on the bulb but the tool is {1000*miss:.0f} mm from the grasp "
                    f"point (axial {1000*float(ee_rel_neck @ (R_seat_inv @ axis_z)):+.0f} mm) -- check "
                    "bulb_tip_offset and the base anchor before trusting this data",
                    throttle_duration_sec=5.0)

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
