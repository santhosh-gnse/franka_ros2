"""Fail-closed action arbitration and task-level safety for MoveIt Servo."""

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from .math_utils import quaternion_to_matrix


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
            "z_lock_enabled": True, "z_correction_gain": 4.0, "max_z_correction_speed": 0.03,
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.actions = {}; self.action_times = {}; self.observation = None; self.obs_time = None
        self.obs_valid = False; self.deadman = False
        self.z_ref = None; self.was_safe = False
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
        z_correction_robot = 0.0
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
            if self.get_parameter("z_lock_enabled").value:
                # ee is EE-relative-to-goal expressed in the *goal marker's*
                # own frame, which has a small (~1-2 deg) real-world tilt --
                # its "Z" drifts by several mm from horizontal motion alone,
                # with no real height change. Rotate into fr3_link0 (the same
                # frame the twist command itself is in) to get the actual
                # physical height, immune to that tilt, before locking it.
                ee_robot_rel = quaternion_to_matrix(q_robot_goal) @ ee
                # Latch the current height the moment actions resume (deadman
                # pressed / policy enabled after being idle) and hold it from
                # there -- PushT is a fixed-height planar task, so neither
                # teleop nor the policy has any legitimate reason to drift in
                # Z; this only ever fights unintended coupling/drift.
                if not self.was_safe or self.z_ref is None:
                    self.z_ref = ee_robot_rel[2]
                gain = self.get_parameter("z_correction_gain").value
                max_z_speed = self.get_parameter("max_z_correction_speed").value
                z_correction_robot = float(np.clip(gain * (self.z_ref - ee_robot_rel[2]), -max_z_speed, max_z_speed))
                self.get_logger().info(
                    f"z_lock: ref={self.z_ref:.4f} cur={ee_robot_rel[2]:.4f} "
                    f"err={self.z_ref - ee_robot_rel[2]:.4f} corr={z_correction_robot:.4f}",
                    throttle_duration_sec=0.5)
        else:
            action = np.zeros(2)
            self.z_ref = None
        self.was_safe = safe
        speed = self.get_parameter("max_linear_speed").value
        velocity_robot = quaternion_to_matrix(q_robot_goal) @ np.array([speed * action[0], speed * action[1], 0.0])
        twist = TwistStamped(); twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.get_parameter("robot_base_frame").value
        twist.twist.linear.x = float(velocity_robot[0]); twist.twist.linear.y = float(velocity_robot[1])
        twist.twist.linear.z = float(velocity_robot[2] + z_correction_robot)
        self.twist_pub.publish(twist)
        self.action_pub.publish(Float32MultiArray(data=action.astype(np.float32).tolist()))


def main(args=None):
    rclpy.init(args=args); node = SafetyNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
