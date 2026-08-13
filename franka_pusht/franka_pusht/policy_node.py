"""Deployment shell for an exported deterministic policy.

The final checkpoint exporter must provide a callable implementation before this
node is enabled on hardware. It intentionally publishes zero until model support
is configured, rather than guessing the Flax checkpoint format.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PolicyNode(Node):
    def __init__(self):
        super().__init__("pusht_policy")
        self.declare_parameter("observation_topic", "/pusht/observation")
        self.declare_parameter("policy_action_topic", "/pusht/policy_action")
        self.declare_parameter("model_path", "")
        self.pub=self.create_publisher(Float32MultiArray, self.get_parameter("policy_action_topic").value, 10)
        self.create_subscription(Float32MultiArray, self.get_parameter("observation_topic").value, self._on_obs, 10)
        self.get_logger().warning("Policy loader not configured; this node will publish zero actions")
    def _on_obs(self, msg):
        obs=np.asarray(msg.data, dtype=np.float32)
        action=np.zeros(2, dtype=np.float32)
        if obs.shape != (24,) or not np.all(np.isfinite(obs)): action[:] = 0.0
        self.pub.publish(Float32MultiArray(data=action.tolist()))


def main(args=None):
    rclpy.init(args=args); node=PolicyNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
