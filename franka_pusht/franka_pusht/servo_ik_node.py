"""Resolve Cartesian twist into joint velocities the way the pusht_mjx sim does.

MoveIt Servo resolves the FR3's redundancy however it likes. The sim instead
pulls the arm toward a fixed posture every substep via a null-space term, so
its 7 joint angles stay on a narrow manifold. That matters because arm_qpos and
arm_qvel are 14 of the 24 observation dimensions and the trained policy is
acutely sensitive to them: measured on a real observation, letting the arm drift
34.8 deg from the home pose moved the policy's action from 6.7 deg off-target to
153 deg off-target, and collapsed its magnitude from 0.72 to 0.006. Servo's own
resolution wanders that far within seconds, which is why the policy stalls a few
steps into an episode even when it starts perfectly.

So this node takes over redundancy resolution, porting environment.py's
_differential_ik:

    dq = J^+ twist + (I - J^+ J) Kp (q_target - q)

and publishes JointJog, leaving Servo to enforce joint limits and collision
checking. The Jacobian is built from /tf rather than a URDF parse: every FR3
joint has axis "0 0 1" in its child link frame, so with z_i the child frame's z
axis in the base and p_i its origin,

    J_v[:, i] = z_i x (p_ee - p_i),   J_w[:, i] = z_i

Validated against finite differences through /compute_fk to 2.8e-5.

Two deliberate departures from the sim, both because it has no real robot to
damage: joint velocities are clipped to a configurable fraction of the actuator
limits, and the null-space target is the validated real home pose rather than
the sim's QHOME (whose joint 4 sits inside Servo's joint-limit halt margin and
whose tip is below the real table -- see ps5_teleop_node.HOME_JOINTS).
"""

import numpy as np
import rclpy
import tf2_ros
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState

try:
    from moveit_msgs.srv import ServoCommandType
except ImportError:
    ServoCommandType = None

SERVO_COMMAND_TYPE_JOINT_JOG = 0


def _quaternion_matrix(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ServoIkNode(Node):
    def __init__(self):
        super().__init__("pusht_servo_ik")
        defaults = {
            "control_rate_hz": 50.0,
            "desired_twist_topic": "/pusht/desired_twist",
            "joint_command_topic": "/servo_node/delta_joint_cmds",
            "joint_state_topic": "/joint_states",
            "robot_base_frame": "fr3_link0",
            "tcp_frame": "fr3_pusher_tcp",
            "joint_names": [f"fr3_joint{i}" for i in range(1, 8)],
            "link_names": [f"fr3_link{i}" for i in range(1, 8)],
            "twist_timeout_s": 0.15,
            # pusht_mjx uses KP_ORI = 10.0 for this term.
            "nullspace_gain": 10.0,
            # Must track ps5_teleop_node.HOME_JOINTS.
            "nullspace_target": [0.358647, 0.222581, -0.524795, -2.664501,
                                 0.378454, 2.837589, -2.088795],
            # Per-joint actuator limits from the sim model (fr3_vel_mjx_free.xml
            # ctrlrange), scaled down for the real robot.
            "joint_velocity_limits": [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26],
            "joint_velocity_scale": 0.3,
            "set_servo_command_type": True,
            # safety_node publishes a twist every tick regardless, sending all
            # zeros when idle or when its safety gates are closed. Without this
            # threshold the null-space term below would still be applied to
            # those zero twists, so the arm would creep in the null space with
            # the dead-man released -- and would fight MoveGroup's trajectory
            # execution during homing, which fails as "Home failed" even though
            # planning succeeded.
            "min_twist_norm": 1e-4,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.link_names = list(self.get_parameter("link_names").value)
        self.base = self.get_parameter("robot_base_frame").value
        self.tcp = self.get_parameter("tcp_frame").value
        self.q_target = np.array(self.get_parameter("nullspace_target").value, dtype=float)
        self.dq_limit = (np.array(self.get_parameter("joint_velocity_limits").value, dtype=float)
                         * float(self.get_parameter("joint_velocity_scale").value))
        self.twist = None
        self.twist_time = None
        self.joints = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(JointJog, self.get_parameter("joint_command_topic").value, 10)
        self.create_subscription(TwistStamped, self.get_parameter("desired_twist_topic").value,
                                 self._on_twist, 10)
        self.create_subscription(JointState, self.get_parameter("joint_state_topic").value,
                                 self._on_joints, 20)
        self.create_timer(1.0 / self.get_parameter("control_rate_hz").value, self._tick)
        if self.get_parameter("set_servo_command_type").value and ServoCommandType is not None:
            self._switch = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
            self.create_timer(1.0, self._ensure_joint_jog_mode)
            self._mode_set = False

    def _ensure_joint_jog_mode(self):
        # Servo silently ignores commands of the wrong type, and resets its mode
        # whenever servo_node restarts, so keep trying until it takes.
        if self._mode_set or not self._switch.service_is_ready():
            return
        future = self._switch.call_async(
            ServoCommandType.Request(command_type=SERVO_COMMAND_TYPE_JOINT_JOG))
        future.add_done_callback(self._on_mode_set)

    def _on_mode_set(self, future):
        result = future.result()
        if result is not None and result.success:
            self._mode_set = True
            self.get_logger().info("Servo switched to JOINT_JOG command mode")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_twist(self, msg):
        t = msg.twist
        self.twist = np.array([t.linear.x, t.linear.y, t.linear.z,
                               t.angular.x, t.angular.y, t.angular.z])
        self.twist_time = self._now()

    def _on_joints(self, msg):
        lookup = dict(zip(msg.name, msg.position))
        if all(name in lookup for name in self.joint_names):
            self.joints = np.array([lookup[name] for name in self.joint_names])

    def _jacobian(self, ):
        """Geometric Jacobian in the base frame, from /tf."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base, self.tcp, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        p_ee = np.array([t.x, t.y, t.z])
        J = np.zeros((6, len(self.joint_names)))
        for i, link in enumerate(self.link_names):
            try:
                tf_i = self.tf_buffer.lookup_transform(self.base, link, Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return None
            ti, ri = tf_i.transform.translation, tf_i.transform.rotation
            p_i = np.array([ti.x, ti.y, ti.z])
            z_i = _quaternion_matrix(ri.w, ri.x, ri.y, ri.z)[:, 2]
            J[0:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:6, i] = z_i
        return J

    def _tick(self):
        if self.twist is None or self.joints is None or self.twist_time is None:
            return
        if self._now() - self.twist_time > self.get_parameter("twist_timeout_s").value:
            # Let Servo time out rather than holding a stale command.
            return
        if np.linalg.norm(self.twist) < self.get_parameter("min_twist_norm").value:
            # Nothing is being commanded, so command nothing. The null-space
            # term must not run on its own: it would move the arm with the
            # dead-man released and interfere with homing.
            return
        J = self._jacobian()
        if J is None:
            self.get_logger().warning("no transform for the arm chain; not commanding",
                                      throttle_duration_sec=5.0)
            return
        J_pinv = np.linalg.pinv(J)
        dq = J_pinv @ self.twist
        # Null-space term: drive toward the target posture using only motion that
        # does not disturb the commanded end-effector twist. This is what keeps
        # arm_qpos on the manifold the policy was trained on.
        gain = self.get_parameter("nullspace_gain").value
        N = np.eye(len(self.joint_names)) - J_pinv @ J
        dq = dq + N @ (gain * (self.q_target - self.joints))
        dq = np.nan_to_num(dq, nan=0.0, posinf=0.0, neginf=0.0)
        dq = np.clip(dq, -self.dq_limit, self.dq_limit)
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base
        msg.joint_names = self.joint_names
        msg.velocities = dq.tolist()
        msg.duration = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoIkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
