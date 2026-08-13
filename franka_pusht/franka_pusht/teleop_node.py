"""Convert PS5 input to a normalized planar PushT action."""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray


class TeleopNode(Node):
    def __init__(self):
        super().__init__("pusht_teleop")
        defaults = {
            "control_rate_hz": 20.0, "joy_topic": "/joy",
            "teleop_action_topic": "/pusht/teleop_action", "deadman_topic": "/pusht/deadman",
            "axis_x": 1, "axis_y": 0, "deadman_button": 4, "speed_axis": 5,
            "deadzone": 0.05, "joy_timeout_s": 0.15,
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.latest = None
        self.latest_s = None
        self.pub = self.create_publisher(Float32MultiArray, self.get_parameter("teleop_action_topic").value, 10)
        self.deadman_pub = self.create_publisher(Bool, self.get_parameter("deadman_topic").value, 10)
        self.create_subscription(Joy, self.get_parameter("joy_topic").value, self._on_joy, 20)
        self.create_timer(1.0 / self.get_parameter("control_rate_hz").value, self._tick)

    def _on_joy(self, msg):
        self.latest = msg
        self.latest_s = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        msg = self.latest
        fresh = msg is not None and now - self.latest_s <= self.get_parameter("joy_timeout_s").value
        deadman_idx = self.get_parameter("deadman_button").value
        enabled = fresh and deadman_idx < len(msg.buttons) and bool(msg.buttons[deadman_idx])
        action = [0.0, 0.0]
        if enabled:
            ix, iy = self.get_parameter("axis_x").value, self.get_parameter("axis_y").value
            speed_i = self.get_parameter("speed_axis").value
            if max(ix, iy, speed_i) < len(msg.axes):
                x, y = float(msg.axes[ix]), float(msg.axes[iy])
                norm = math.hypot(x, y)
                trigger = max(0.0, min(1.0, (1.0 - msg.axes[speed_i]) / 2.0))
                if norm > self.get_parameter("deadzone").value:
                    action = [trigger * x / max(1.0, norm), trigger * y / max(1.0, norm)]
        self.deadman_pub.publish(Bool(data=enabled))
        self.pub.publish(Float32MultiArray(data=action))


def main(args=None):
    rclpy.init(args=args); node = TeleopNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
