"""Raise the FR3's collision thresholds for contact-rich sauce spreading.

Franka's defaults are tuned for free-space motion: any unexpected external
force trips a reflex and the robot drops into an error state (the red light).
Spreading is contact-rich by design -- the tool presses against the board the
entire time -- so the defaults fire on forces that are the task working
correctly, not a collision.

The thresholds are runtime state, not configuration: franka_hardware applies its
defaults every time it connects, so a manual service call is lost on the next
moveit.launch.py restart. This node re-applies them on startup and keeps
retrying until the service appears, since it usually comes up after this node
does.

The profile below is copied from franka_bulbscrew's, chosen for a very
different contact task (screwing). Treat it as a starting point, not a
measurement -- tighten scale once real spreading forces are known.

SAFETY: the reflex is what stops the arm when it hits something it should not.
Raising these trades that protection for tolerance -- the arm will push
noticeably harder before reacting. Keep a hand near the stop, and do not run a
policy autonomously until teleop is well behaved. Lower `scale` to tighten.
"""

import rclpy
from rclpy.node import Node

try:
    from franka_msgs.srv import SetForceTorqueCollisionBehavior
except ImportError:
    SetForceTorqueCollisionBehavior = None


class CollisionBehaviorNode(Node):
    def __init__(self):
        super().__init__("pizza_collision_behavior")
        defaults = {
            "collision_service": "/service_server/set_force_torque_collision_behavior",
            # Roughly Franka's "high" preset: near the joint torque limits, i.e.
            # about as permissive as the hardware allows. Per-joint (7) and
            # Cartesian (6, forces then torques). Borrowed from
            # franka_bulbscrew, NOT yet tuned for spreading's actual forces.
            "torque_thresholds": [40.0, 40.0, 38.0, 38.0, 32.0, 28.0, 24.0],
            "force_thresholds": [40.0, 40.0, 40.0, 50.0, 50.0, 50.0],
            # Scale both down together if the arm should react sooner. 1.0 is
            # the preset above; 0.5 is roughly Franka's default sensitivity.
            "scale": 1.0,
            "retry_period_s": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if SetForceTorqueCollisionBehavior is None:
            self.get_logger().error("franka_msgs unavailable: collision thresholds NOT raised")
            return
        self._done = False
        self._client = self.create_client(SetForceTorqueCollisionBehavior,
                                          self.get_parameter("collision_service").value)
        self.create_timer(self.get_parameter("retry_period_s").value, self._apply)

    def _apply(self):
        if self._done:
            return
        if not self._client.service_is_ready():
            self.get_logger().info("waiting for the collision-behaviour service...",
                                   throttle_duration_sec=10.0)
            return
        g = self.get_parameter
        scale = float(g("scale").value)
        torque = [scale * v for v in g("torque_thresholds").value]
        force = [scale * v for v in g("force_thresholds").value]
        request = SetForceTorqueCollisionBehavior.Request()
        request.lower_torque_thresholds_nominal = torque
        request.upper_torque_thresholds_nominal = torque
        request.lower_force_thresholds_nominal = force
        request.upper_force_thresholds_nominal = force
        self._done = True          # one attempt per service appearance
        self._client.call_async(request).add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info(
                f"collision thresholds raised (scale {self.get_parameter('scale').value}) -- "
                "the arm now tolerates spreading contact without a reflex")
        else:
            reason = getattr(result, "error", "no response")
            self.get_logger().error(f"failed to raise collision thresholds: {reason}")
            self._done = False     # allow a retry


def main(args=None):
    rclpy.init(args=args)
    node = CollisionBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
