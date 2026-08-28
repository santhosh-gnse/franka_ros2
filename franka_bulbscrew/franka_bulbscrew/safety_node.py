"""Fail-closed action arbitration and task-level safety for bulb screwing.

Turns the 5-D action into the 6-D twist that servo_ik_node resolves into joint
velocities, plus a gripper width command:

    action[0:3] -> linear velocity, x max_linear_speed, rotated into fr3_link0
    action[3]   -> wrist yaw rate, x max_yaw_rate
    action[4]   -> gripper width, [-1, 1] mapped to [0, grip_max] per finger

The angular part mirrors bulbscrew_mjx's _differential_ik exactly: roll and
pitch are *servoed* to keep the gripper pointing straight down, while yaw is
*commanded* rather than corrected -- that substitution on the z axis is the
screwing degree of freedom, and it is the one piece a PushT-style pure
orientation hold would silently suppress.

    rot = KP_ROT * quat_error(Q_HOLD, ee_quat)
    rot[2] = yaw_rate

Requires Servo's apply_twist_commands_about_ee_frame:=false, so a base-frame
twist means base-frame motion. With the default (true) it is applied about the
tool frame, whose +Z points down when the gripper points down -- which inverts
vertical motion and silently rotates the horizontal plane. That cost a long
debugging session on PushT; see franka_pusht/README.md.
"""

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Float32MultiArray

from .math_utils import orientation_error_vector, quaternion_to_matrix


class SafetyNode(Node):
    def __init__(self):
        super().__init__("bulbscrew_safety")
        defaults = {
            "control_rate_hz": 20.0,
            "max_linear_speed": 0.10,
            "max_yaw_rate": 0.75,
            "grip_max": 0.04,
            "action_timeout_s": 0.15, "observation_timeout_s": 0.15,
            "command_source": "teleop", "robot_base_frame": "fr3_link0",
            "tcp_frame": "fr3_hand_tcp",
            "teleop_action_topic": "/bulbscrew/teleop_action",
            "policy_action_topic": "/bulbscrew/policy_action",
            "executed_action_topic": "/bulbscrew/executed_action",
            "deadman_topic": "/bulbscrew/deadman",
            "observation_topic": "/bulbscrew/observation",
            "observation_valid_topic": "/bulbscrew/observation_valid",
            "servo_command_topic": "/bulbscrew/desired_twist",
            "gripper_command_topic": "/bulbscrew/gripper_width",
            "calibration_configured": False,
            "workspace_configured": False,
            # Bounds on the end effector relative to the socket seat, in the
            # seat frame -- i.e. observation indices 7:10 shifted by the bulb
            # neck. Unlike PushT this task is genuinely 3-D, so z is a real
            # bound and not a formality.
            "workspace_x": [0.0, 0.0], "workspace_y": [0.0, 0.0], "workspace_z": [0.0, 0.0],
            # Orientation hold, matching bulbscrew_mjx's KP_ROT = 3.0.
            "orientation_hold_gain": 3.0,
            "max_angular_speed": 1.5,
            # Gripper-down orientation of the tool in fr3_link0. Measure it with
            #   ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
            # at the home pose and convert xyzw -> wxyz.
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
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

    def _tool_orientation(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("robot_base_frame").value,
                self.get_parameter("tcp_frame").value, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        r = tf.transform.rotation
        return np.array([r.w, r.x, r.y, r.z])

    def _tick(self):
        g = self.get_parameter
        now = self._now()
        source = g("command_source").value
        safe = source in ("teleop", "policy")
        safe &= bool(g("calibration_configured").value)
        safe &= bool(g("workspace_configured").value)
        safe &= self.obs_valid and self.observation is not None and self.observation.shape == (25,)
        safe &= self.obs_time is not None and now - self.obs_time <= g("observation_timeout_s").value
        safe &= source in self.actions and self.actions[source].shape == (5,)
        safe &= (source in self.action_times
                 and now - self.action_times[source] <= g("action_timeout_s").value)
        if source == "teleop":
            safe &= self.deadman
        action = np.clip(self.actions[source], -1.0, 1.0) if safe else np.zeros(5)
        safe &= bool(np.all(np.isfinite(action)))

        if safe:
            # Per-axis clamp only: block driving further past a bound but always
            # allow motion back toward the safe region. An all-or-nothing gate
            # deadlocks the arm the moment any axis drifts out, with no action
            # able to recover it -- that bug bit PushT and is easy to reintroduce.
            ee = self.observation[7:10]
            bounds = [g(n).value for n in ("workspace_x", "workspace_y", "workspace_z")]
            for i in range(3):
                low, high = bounds[i]
                if (ee[i] <= low and action[i] < 0) or (ee[i] >= high and action[i] > 0):
                    action[i] = 0.0
        else:
            action = np.zeros(5)

        speed = g("max_linear_speed").value
        linear = np.array([speed * action[0], speed * action[1], speed * action[2]])
        angular = np.zeros(3)
        if safe:
            orientation = self._tool_orientation()
            if orientation is None:
                # Skip the correction rather than zeroing everything: a missing
                # transform must never deadlock the arm.
                self.get_logger().warning("no tool transform; orientation hold inactive",
                                          throttle_duration_sec=5.0)
            else:
                max_w = g("max_angular_speed").value
                error = orientation_error_vector(g("hold_quaternion_wxyz").value, orientation)
                angular = np.clip(g("orientation_hold_gain").value * error, -max_w, max_w)
            # Yaw is commanded, not corrected. This is the screwing DoF, and it
            # must overwrite the hold's z term rather than add to it.
            angular[2] = float(np.clip(action[3] * g("max_yaw_rate").value, -max_w, max_w))

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = g("robot_base_frame").value
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = map(float, linear)
        twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = map(float, angular)
        self.twist_pub.publish(twist)

        # Gripper: [-1, 1] -> [0, grip_max] per finger, as in the sim. Held open
        # when unsafe rather than closed, so a fault never clamps on the bulb.
        grip_max = g("grip_max").value
        width = (action[4] + 1.0) * 0.5 * grip_max if safe else grip_max
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
