"""Build the 17-D pizza-sauce-spreading observation from joints and FK alone.

Unlike PushT and BulbScrew, nothing external is tracked at runtime: there is no
T-block, no bulb, no object whose pose has to be sensed. The task is spreading
sauce across a fixed board at a fixed height and orientation, so the only state
that matters is where the tool is relative to the board centre and what the arm
is doing to get there.

OptiTrack is used exactly ONCE, offline, to measure home_position
(config/pizza_robot.yaml) via a marker placed at the board's physical centre --
see NEED_TO_CONFIGURE.md. It plays no part in this running pipeline: no
optitrack_bridge_node, no rigid-body subscription, nothing to go stale if a
marker is later removed.

Layout:
    0:3    tool position relative to home (the board centre), in fr3_link0
    3:10   arm joint positions
    10:17  arm joint velocities

Height and orientation are not separate observation dims, because the task
holds both constant rather than varying them: during kinesthetic teaching by
the operator's own contact with the board, and at deployment by
servo_ik_node/safety_node (copied from franka_pusht, not yet wired up here).
So dim 2 (z) should stay near z_hold_target through a good demonstration --
worth checking per episode rather than assumed, the same way FINDINGS checks
franka_bulbscrew's grasp residual.
"""

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray


class ObservationNode(Node):
    def __init__(self):
        super().__init__("pizza_observation")
        defaults = {
            "control_rate_hz": 20.0,
            "pose_timeout_s": 0.15,
            "joint_state_topic": "/joint_states",
            "joint_names": [f"fr3_joint{i}" for i in range(1, 8)],
            "robot_base_frame": "fr3_link0",
            "tcp_frame": "fr3_pusher_tcp",
            "observation_topic": "/pizza/observation",
            "observation_valid_topic": "/pizza/observation_valid",
            # Board centre in fr3_link0. PLACEHOLDER -- see NEED_TO_CONFIGURE.md.
            # On PushT an equivalent placeholder went unnoticed for a whole
            # session; do not enable calibration_configured until this is real.
            "home_position": [0.0, 0.0, 0.0],
            "calibration_configured": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        g = self.get_parameter
        self.rate = g("control_rate_hz").value
        self.timeout = g("pose_timeout_s").value
        self.joint_names = list(g("joint_names").value)
        self.robot_base_frame = g("robot_base_frame").value
        self.tcp_frame = g("tcp_frame").value
        self.home = np.array(g("home_position").value)

        self.joints = None
        self.joint_received_s = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Float32MultiArray, g("observation_topic").value, 10)
        self.valid_pub = self.create_publisher(Bool, g("observation_valid_topic").value, 10)
        self.create_subscription(JointState, g("joint_state_topic").value, self._on_joints, 50)
        self.create_timer(1.0 / self.rate, self._tick)

    def _on_joints(self, msg):
        by_name = dict(zip(msg.name, zip(msg.position, msg.velocity)))
        if not all(n in by_name for n in self.joint_names):
            return
        self.joints = by_name
        self.joint_received_s = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup_tcp(self, now):
        try:
            tf = self.tf_buffer.lookup_transform(self.robot_base_frame, self.tcp_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if abs(now - stamp) > self.timeout:
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z])

    def _tick(self):
        now = self._now()
        valid = self.joints is not None and abs(now - self.joint_received_s) <= self.timeout
        tcp = self._lookup_tcp(now) if valid else None
        valid = valid and tcp is not None
        if not valid:
            self.valid_pub.publish(Bool(data=False))
            return
        try:
            observation = self._build(tcp)
            valid = bool(np.all(np.isfinite(observation)))
        except ValueError:
            valid = False
        self.valid_pub.publish(Bool(data=bool(valid)))
        if valid:
            self.pub.publish(Float32MultiArray(data=observation.tolist()))

    def _build(self, tcp):
        ee_rel_home = tcp - self.home
        q = [self.joints[n][0] for n in self.joint_names]
        dq = [self.joints[n][1] for n in self.joint_names]
        return np.concatenate((ee_rel_home, q, dq)).astype(np.float32)


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
