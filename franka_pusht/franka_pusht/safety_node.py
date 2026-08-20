"""Fail-closed action arbitration and task-level safety for MoveIt Servo."""

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32MultiArray

from .math_utils import orientation_error_vector, quaternion_to_matrix


class SafetyNode(Node):
    def __init__(self):
        super().__init__("pusht_safety")
        defaults = {
            "control_rate_hz": 20.0, "max_linear_speed": 0.05,
            "action_timeout_s": 0.15, "observation_timeout_s": 0.15,
            "command_source": "teleop", "robot_base_frame": "fr3_link0",
            "teleop_action_topic": "/pusht/teleop_action", "policy_action_topic": "/pusht/policy_action",
            "executed_action_topic": "/pusht/executed_action", "deadman_topic": "/pusht/deadman",
            "observation_topic": "/pusht/observation", "observation_valid_topic": "/pusht/observation_valid",
            "servo_command_topic": "/servo_node/delta_twist_cmds",
            "calibration_configured": False,
            "goal_to_robot_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "workspace_configured": False, "workspace_x": [0.0, 0.0],
            "workspace_y": [0.0, 0.0], "workspace_z": [0.0, 0.0],
            # Pose hold (see _pose_hold): the 2-D action only spans the
            # horizontal plane, so height and orientation must be actively
            # servoed to fixed targets -- the pusht_mjx sim does exactly this
            # inside its differential IK every substep (Z_HOLD / GOAL_QUAT_EE),
            # which is why its policy never had to learn to hold them.
            "z_hold_enabled": True, "z_hold_target": 0.060,
            "z_hold_gain": 2.0, "max_z_hold_speed": 0.05,
            "orientation_hold_enabled": False,
            "orientation_hold_quaternion_wxyz": [0.0, 1.0, 0.0, 0.0],
            "orientation_hold_gain": 1.0, "max_angular_speed": 0.5,
            "tcp_frame": "fr3_pusher_tcp",
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.actions = {}; self.action_times = {}; self.observation = None; self.obs_time = None
        self.obs_valid = False; self.deadman = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        for source, topic_parameter in (("teleop", "teleop_action_topic"), ("policy", "policy_action_topic")):
            self.create_subscription(Float32MultiArray, self.get_parameter(topic_parameter).value,
                                     lambda msg, s=source: self._on_action(s, msg), 10)
        self.create_subscription(Float32MultiArray, self.get_parameter("observation_topic").value, self._on_observation, 10)
        self.create_subscription(Bool, self.get_parameter("observation_valid_topic").value,
                                 lambda msg: setattr(self, "obs_valid", bool(msg.data)), 10)
        self.create_subscription(Bool, self.get_parameter("deadman_topic").value,
                                 lambda msg: setattr(self, "deadman", bool(msg.data)), 10)
        self.action_pub = self.create_publisher(Float32MultiArray, self.get_parameter("executed_action_topic").value, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.get_parameter("servo_command_topic").value, 10)
        self.create_timer(1.0 / self.get_parameter("control_rate_hz").value, self._tick)

    def _now(self): return self.get_clock().now().nanoseconds * 1e-9

    def _tcp_pose(self):
        """Pusher pose in the robot base frame, from forward kinematics.

        Deliberately read from /tf rather than the observation's ee_pos_rel_goal:
        FK is exact robot kinematics, whereas the observation's EE estimate is
        anchored through the OptiTrack pole_base calibration, so its height
        carries that calibration's error and mocap jitter. A height hold must
        not chase either.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("robot_base_frame").value,
                self.get_parameter("tcp_frame").value, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([r.w, r.x, r.y, r.z])

    def _pose_hold(self):
        """Base-frame [vz, wx, wy, wz] holding the pusher's height/orientation.

        Mirrors pusht_mjx's differential IK, which feeds (Z_HOLD - ee_z) and the
        orientation error into the same Jacobian solve as the commanded planar
        velocity. Servo must be configured with
        apply_twist_commands_about_ee_frame:=false for these to mean base-frame
        vertical/angular motion (otherwise +Z is along the downward-pointing
        tool axis and the height correction inverts).
        """
        hold_z = self.get_parameter("z_hold_enabled").value
        hold_orientation = self.get_parameter("orientation_hold_enabled").value
        if not (hold_z or hold_orientation):
            return 0.0, np.zeros(3)
        pose = self._tcp_pose()
        if pose is None:
            # Skip the correction rather than zeroing the whole command: a
            # missing transform must not deadlock teleop (see the workspace
            # clamp note below for the same reasoning).
            self.get_logger().warning(
                f"no {self.get_parameter('tcp_frame').value} transform; pose hold inactive",
                throttle_duration_sec=5.0)
            return 0.0, np.zeros(3)
        position, orientation = pose
        vz = 0.0
        if hold_z:
            max_vz = self.get_parameter("max_z_hold_speed").value
            error = self.get_parameter("z_hold_target").value - position[2]
            vz = float(np.clip(self.get_parameter("z_hold_gain").value * error, -max_vz, max_vz))
        angular = np.zeros(3)
        if hold_orientation:
            max_w = self.get_parameter("max_angular_speed").value
            error = orientation_error_vector(
                self.get_parameter("orientation_hold_quaternion_wxyz").value, orientation)
            angular = np.clip(self.get_parameter("orientation_hold_gain").value * error, -max_w, max_w)
        return vz, angular

    def _on_action(self, source, msg):
        self.actions[source] = np.asarray(msg.data, dtype=np.float64); self.action_times[source] = self._now()
    def _on_observation(self, msg):
        self.observation = np.asarray(msg.data, dtype=np.float64); self.obs_time = self._now()

    def _tick(self):
        now = self._now(); source = self.get_parameter("command_source").value
        safe = source in ("teleop", "policy")
        safe &= self.get_parameter("calibration_configured").value
        safe &= self.get_parameter("workspace_configured").value
        safe &= self.obs_valid and self.observation is not None and self.observation.shape == (24,)
        safe &= self.obs_time is not None and now - self.obs_time <= self.get_parameter("observation_timeout_s").value
        safe &= source in self.actions and self.actions[source].shape == (2,)
        safe &= source in self.action_times and now - self.action_times[source] <= self.get_parameter("action_timeout_s").value
        if source == "teleop": safe &= self.deadman
        action = np.clip(self.actions[source], -1.0, 1.0) if safe else np.zeros(2)
        safe &= bool(np.all(np.isfinite(action)))
        q_robot_goal = self.get_parameter("goal_to_robot_quaternion_wxyz").value
        if safe:
            ee = self.observation[7:10]
            bounds = [self.get_parameter(n).value for n in ("workspace_x", "workspace_y", "workspace_z")]
            # Per-axis clamp only: block driving further past a bound, but
            # always allow motion back toward the safe region. This must NOT
            # be an all-or-nothing "safe" gate -- Z isn't commanded by this
            # 2-D action at all (only X/Y are), so an all-or-nothing gate on
            # all three axes would deadlock X/Y teleop permanently the moment
            # Z alone drifted outside its bound (e.g. from goal_to_robot
            # rotation coupling), with no action able to ever recover it.
            if ee[0] <= bounds[0][0] and action[0] < 0 or ee[0] >= bounds[0][1] and action[0] > 0: action[0] = 0.0
            if ee[1] <= bounds[1][0] and action[1] < 0 or ee[1] >= bounds[1][1] and action[1] > 0: action[1] = 0.0
        else:
            action = np.zeros(2)
        speed = self.get_parameter("max_linear_speed").value
        velocity_robot = quaternion_to_matrix(q_robot_goal) @ np.array([speed * action[0], speed * action[1], 0.0])
        # Hold height/orientation whenever the pipeline is live, including while
        # the action is zero: drift accumulates from gravity, redundancy
        # resolution and Servo's near-limit velocity scaling, not only from
        # commanded motion, so the hold must keep working between pushes.
        vz, angular = self._pose_hold() if safe else (0.0, np.zeros(3))
        twist = TwistStamped(); twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.get_parameter("robot_base_frame").value
        twist.twist.linear.x = float(velocity_robot[0]); twist.twist.linear.y = float(velocity_robot[1])
        twist.twist.linear.z = float(velocity_robot[2] + vz)
        twist.twist.angular.x = float(angular[0]); twist.twist.angular.y = float(angular[1])
        twist.twist.angular.z = float(angular[2])
        self.twist_pub.publish(twist)
        self.action_pub.publish(Float32MultiArray(data=action.astype(np.float32).tolist()))


def main(args=None):
    rclpy.init(args=args); node = SafetyNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
