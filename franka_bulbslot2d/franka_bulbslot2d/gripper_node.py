"""Drive the Franka Hand from a continuous width command.

safety_node publishes a target width at the control rate, but franka_gripper
exposes goal-based actions (Move / Grasp), not a streaming interface. Sending a
new goal every tick would cancel the previous one continuously and the fingers
would never move, so this node only acts on meaningful changes and never has
more than one goal in flight.

Grasp vs Move matters: Move positions the fingers and stops, and will happily
stop short against an object without applying force. Grasp keeps squeezing with
the configured force, which is what actually holds the bulb. So closing commands
go to Grasp and opening commands to Move.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from franka_msgs.action import Grasp, Move
except ImportError:  # allows the rest of the pipeline to run without the gripper
    Grasp = Move = None


class GripperNode(Node):
    def __init__(self):
        super().__init__("bulbslot2d_gripper")
        defaults = {
            "gripper_command_topic": "/bulbslot2d/gripper_width",
            # Named gripper_* deliberately: the configs use a "/**:" wildcard so
            # every parameter reaches every node, and a generic "move_action"
            # here silently overwrote the MoveGroup action name in the
            # kinesthetic recorder, which then waited forever for a MoveGroup
            # server at the gripper's address.
            "gripper_grasp_action": "/fr3/franka_gripper/grasp",
            "gripper_move_action": "/fr3/franka_gripper/move",
            "grip_max": 0.04,
            # Only re-issue a goal when the target moves by more than this, so
            # small command jitter does not spam the action server.
            "width_deadband": 0.004,
            "speed": 0.1,
            "force": 20.0,
            # franka_gripper treats Grasp's epsilon as an acceptance window on
            # the RESULTING width: if the object does not end up within
            # [width - inner, width + outer] the grasp is reported FAILED and
            # the fingers stop applying force, i.e. they let go.
            #
            # We use Grasp as "close and hold with force", not as a size check,
            # so the window must span the gripper's whole range. With the
            # previous 0.05 and a commanded width of 0, anything thicker than
            # 50 mm was judged a failure -- and the bulb head is 60 mm, so every
            # grasp of the head released a moment after closing.
            "grasp_epsilon_inner": 0.08,
            "grasp_epsilon_outer": 0.08,
            # Below this total width a command counts as "close and hold".
            "close_threshold": 0.06,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        g = self.get_parameter
        self.last_target = None
        self.busy = False
        self.grasp = self.move = None
        if Grasp is not None:
            self.grasp = ActionClient(self, Grasp, g("gripper_grasp_action").value)
            self.move = ActionClient(self, Move, g("gripper_move_action").value)
        else:
            self.get_logger().error("franka_msgs unavailable: gripper commands are ignored")
        self.create_subscription(Float32, g("gripper_command_topic").value, self._on_width, 10)

    def _on_width(self, msg):
        g = self.get_parameter
        # safety_node speaks per-finger width, the actions speak total opening.
        target = max(0.0, min(2.0 * g("grip_max").value, 2.0 * float(msg.data)))
        if self.grasp is None or self.busy:
            return
        if self.last_target is not None and abs(target - self.last_target) < g("width_deadband").value:
            return
        closing = target < g("close_threshold").value
        client = self.grasp if closing else self.move
        if not client.server_is_ready():
            self.get_logger().warning(f"{'grasp' if closing else 'move'} action not available",
                                      throttle_duration_sec=5.0)
            return
        if closing:
            goal = Grasp.Goal()
            goal.width = target
            goal.speed = g("speed").value
            goal.force = g("force").value
            goal.epsilon.inner = g("grasp_epsilon_inner").value
            goal.epsilon.outer = g("grasp_epsilon_outer").value
        else:
            goal = Move.Goal()
            goal.width = target
            goal.speed = g("speed").value
        self.last_target = target
        self.busy = True
        future = client.send_goal_async(goal)
        future.add_done_callback(self._on_sent)

    def _on_sent(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.busy = False
            return
        handle.get_result_async().add_done_callback(lambda _f: setattr(self, "busy", False))


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
