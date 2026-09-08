"""Fail-closed action arbitration and task-level safety for 2-D bulb slotting.

Turns the 3-D action into the 6-D twist that servo_ik_node resolves into joint
velocities, plus a gripper width command:

    action[0] -> velocity along Y_PRIME (the home-seat line), x max_linear_speed
    action[1] -> vertical velocity (Z), x max_linear_speed
    action[2] -> gripper width, [-1, 1] mapped to [0, grip_max] per finger

Unlike franka_bulbscrew's safety_node, action[0:3] does NOT map directly onto
raw fr3_link0 x/y/z: the home (bulb-in-hand) pose and the socket seat pose are
two fixed points that do not happen to lie along any single robot axis, so the
task's real "forward" direction is Y_PRIME, a horizontal direction yawed to
pass through both points (computed once at startup from plane_yaw_rad -- see
README.md for the derivation from the two calibrated points). X_PRIME, the
horizontal direction perpendicular to that, is never commanded: it is actively
held at x_prime_hold_target, the (shared, by construction) X_PRIME coordinate
of both home and seat -- the same gain-and-clamp shape as a z_hold in other
packages here, just projected onto a different axis. Z is unrotated world Z,
same as everywhere else in this workspace.

Orientation is held in full on all 3 axes via hold_quaternion_wxyz /
orientation_error_vector -- there is no commanded-yaw override here the way
franka_bulbscrew has one for its screwing DoF, because this task's action has
no yaw dimension at all.

Requires Servo's apply_twist_commands_about_ee_frame:=false, so a base-frame
twist means base-frame motion (same requirement as franka_bulbscrew/franka_pusht).
"""

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Float32MultiArray

from .math_utils import orientation_error_vector


class SafetyNode(Node):
    def __init__(self):
        super().__init__("bulbslot2d_safety")
        defaults = {
            "control_rate_hz": 20.0,
            "max_linear_speed": 0.10,
            "grip_max": 0.04,
            "action_timeout_s": 0.15, "observation_timeout_s": 0.15,
            "command_source": "teleop", "robot_base_frame": "fr3_link0",
            "tcp_frame": "fr3_hand_tcp",
            "teleop_action_topic": "/bulbslot2d/teleop_action",
            "policy_action_topic": "/bulbslot2d/policy_action",
            "executed_action_topic": "/bulbslot2d/executed_action",
            "deadman_topic": "/bulbslot2d/deadman",
            "observation_topic": "/bulbslot2d/observation",
            "observation_valid_topic": "/bulbslot2d/observation_valid",
            "servo_command_topic": "/bulbslot2d/desired_twist",
            "gripper_command_topic": "/bulbslot2d/gripper_width",
            "calibration_configured": False,
            "workspace_configured": False,
            # The plane: a single yaw (radians, about world Z) defines both
            # Y_PRIME (actuated, home->seat direction) and X_PRIME (held,
            # perpendicular). See README.md for how this was derived from the
            # calibrated home/seat positions.
            "plane_yaw_rad": 0.0,
            # X_PRIME . home == X_PRIME . seat, by construction -- the single
            # out-of-plane coordinate to hold constant.
            "x_prime_hold_target": 0.0,
            "x_prime_hold_gain": 2.0,
            "max_x_prime_hold_speed": 0.05,
            # Bounds on the TOOL (fr3_hand_tcp) in the plane: workspace_yprime
            # is a range along Y_PRIME, workspace_z is literal world Z
            # (unchanged shape from franka_bulbscrew's per-axis bounds).
            "workspace_yprime": [0.0, 0.0], "workspace_z": [0.0, 0.0],
            # Orientation hold, held on ALL 3 axes (no yaw override -- this
            # task has no commanded yaw at all).
            "orientation_hold_gain": 3.0,
            "max_angular_speed": 1.5,
            "hold_quaternion_wxyz": [0.0, 1.0, 0.0, 0.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.actions = {}
        self.action_times = {}
        self.observation = None
        self.obs_time = None
        self.obs_valid = False
        self.deadman = False
        # Start open; only an explicit command changes this.
        self.last_width = float(self.get_parameter("grip_max").value)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        yaw = float(self.get_parameter("plane_yaw_rad").value)
        self.y_prime = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        self.x_prime = np.array([-np.sin(yaw), np.cos(yaw), 0.0])

        g = self.get_parameter
        for source, topic_parameter in (("teleop", "teleop_action_topic"),
                                        ("policy", "policy_action_topic")):
            self.create_subscription(Float32MultiArray, g(topic_parameter).value,
                                     lambda msg, s=source: self._on_action(s, msg), 10)
        self.create_subscription(Float32MultiArray, g("observation_topic").value,
                                 self._on_observation, 10)
        self.create_subscription(Bool, g("observation_valid_topic").value,
                                 lambda m: setattr(self, "obs_valid", bool(m.data)), 10)
        self.create_subscription(Bool, g("deadman_topic").value,
                                 lambda m: setattr(self, "deadman", bool(m.data)), 10)
        self.action_pub = self.create_publisher(Float32MultiArray,
                                                g("executed_action_topic").value, 10)
        self.twist_pub = self.create_publisher(TwistStamped, g("servo_command_topic").value, 10)
        self.grip_pub = self.create_publisher(Float32, g("gripper_command_topic").value, 10)
        self.create_timer(1.0 / g("control_rate_hz").value, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_action(self, source, msg):
        self.actions[source] = np.asarray(msg.data, dtype=np.float64)
        self.action_times[source] = self._now()

    def _on_observation(self, msg):
        self.observation = np.asarray(msg.data, dtype=np.float64)
        self.obs_time = self._now()

    def _tool_pose(self):
        """Tool position in the robot base frame, from forward kinematics."""
        tf = self._lookup_tool()
        if tf is None:
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z])

    def _lookup_tool(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("robot_base_frame").value,
                self.get_parameter("tcp_frame").value, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        return tf

    def _tool_orientation(self):
        tf = self._lookup_tool()
        if tf is None:
            return None
        r = tf.transform.rotation
        return np.array([r.w, r.x, r.y, r.z])

    def _tick(self):
        g = self.get_parameter
        now = self._now()
        source = g("command_source").value
        pipeline_ok = source in ("teleop", "policy")
        pipeline_ok &= bool(g("calibration_configured").value)
        pipeline_ok &= bool(g("workspace_configured").value)
        pipeline_ok &= self.obs_valid and self.observation is not None and self.observation.shape == (25,)
        pipeline_ok &= self.obs_time is not None and now - self.obs_time <= g("observation_timeout_s").value
        pipeline_ok &= source in self.actions and self.actions[source].shape == (3,)
        pipeline_ok &= (source in self.action_times
                        and now - self.action_times[source] <= g("action_timeout_s").value)
        action = np.clip(self.actions[source], -1.0, 1.0) if pipeline_ok else np.zeros(3)
        pipeline_ok &= bool(np.all(np.isfinite(action)))
        if not pipeline_ok:
            action = np.zeros(3)

        # The gripper does NOT need the dead-man: releasing L1 must not drop
        # the bulb, and opening/closing the jaws is not the kind of
        # unintended-arm-motion risk the dead-man exists for. Only
        # linear/angular motion (action[0:2]) is gated on it -- action[2]
        # (gripper) survives below even when motion_safe is False.
        safe = motion_safe = pipeline_ok and (self.deadman if source == "teleop" else True)

        tool = self._tool_pose() if safe else None
        v_hold = 0.0
        if safe and tool is not None:
            # Bound the TOOL IN the plane, not the raw fr3_link0 axes: Y_PRIME
            # is the only actuated horizontal direction, so that is what a
            # workspace bound has to be measured against. Per-axis clamp only
            # (block driving further past a bound, always allow motion back)
            # -- same reasoning as franka_bulbscrew: an all-or-nothing gate
            # deadlocks the arm the moment any axis drifts out.
            yprime_pos = float(self.y_prime @ tool)
            xprime_pos = float(self.x_prime @ tool)
            lo, hi = g("workspace_yprime").value
            if (yprime_pos <= lo and action[0] < 0) or (yprime_pos >= hi and action[0] > 0):
                action[0] = 0.0
            lo, hi = g("workspace_z").value
            if (tool[2] <= lo and action[1] < 0) or (tool[2] >= hi and action[1] > 0):
                action[1] = 0.0
            error_xp = g("x_prime_hold_target").value - xprime_pos
            max_hold = g("max_x_prime_hold_speed").value
            v_hold = float(np.clip(g("x_prime_hold_gain").value * error_xp, -max_hold, max_hold))
        elif safe:
            # No transform: skip the clamp/hold rather than zeroing everything,
            # so a missing TF can never deadlock the arm.
            self.get_logger().warning("no tool transform; workspace clamp and X' hold inactive",
                                      throttle_duration_sec=5.0)
        else:
            action[0] = 0.0
            action[1] = 0.0

        speed = g("max_linear_speed").value
        linear = (speed * action[0] * self.y_prime
                 + np.array([0.0, 0.0, speed * action[1]])
                 + v_hold * self.x_prime)
        angular = np.zeros(3)
        if safe:
            orientation = self._tool_orientation()
            if orientation is None:
                self.get_logger().warning("no tool transform; orientation hold inactive",
                                          throttle_duration_sec=5.0)
            else:
                max_w = g("max_angular_speed").value
                error = orientation_error_vector(g("hold_quaternion_wxyz").value, orientation)
                angular = np.clip(g("orientation_hold_gain").value * error, -max_w, max_w)
            # No yaw override here: unlike franka_bulbscrew, this task has no
            # commanded yaw dimension, so the hold above applies unmodified on
            # all 3 axes.

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = g("robot_base_frame").value
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = map(float, linear)
        twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = map(float, angular)
        self.twist_pub.publish(twist)

        # Gripper: [-1, 1] -> [0, grip_max] per finger, as in franka_bulbscrew.
        # Gated on pipeline_ok, NOT motion_safe -- the gripper works without
        # the dead-man held (see the comment above where motion_safe is
        # computed). When the pipeline itself is unhealthy (bad observation,
        # stale action, not calibrated), hold the last commanded width rather
        # than forcing the jaws open: forcing open would drop the bulb on
        # every hiccup, and holding is the safer failure for glass.
        grip_max = g("grip_max").value
        if pipeline_ok:
            self.last_width = (action[2] + 1.0) * 0.5 * grip_max
        width = self.last_width
        self.grip_pub.publish(Float32(data=float(width)))
        self.action_pub.publish(Float32MultiArray(data=action.astype(np.float32).tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
