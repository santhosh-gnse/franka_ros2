"""Deployment shell for an exported deterministic policy.

Loads a trirl_ppo_fb (trust-region-irl) policy checkpoint's params, exported
as a plain numpy .npz (see franka_pusht/models/pusht_best_policy.npz), and
runs its forward pass directly in numpy -- no jax/flax/orbax dependency at
runtime. Architecture and weights come from the Orbax checkpoint's "policy"
subtree (Dense(512) -> LayerNorm -> elu -> Dense(256) -> elu -> Dense(128)
-> elu -> Dense(2)), matching trust_region_irl/algorithms/trirl_ppo_fb/
flax_full_jit/policy.py. The policy operates on the raw 24-dim observation
(policy_observation_indices defaults to all indices for this env) and was
trained entirely in simulation (config_algorithm.json's data_path points at
the sim expert dataset) -- its actions assume goal_to_robot_quaternion_wxyz
correctly maps goal-frame actions to fr3_link0, same as teleop.

Publishes the policy mean (deterministic action), not a sampled action --
policy_logstd is loaded but unused.
"""

import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def _layer_norm(x, scale, bias, eps=1e-6):
    mean = x.mean(axis=-1, keepdims=True)
    var = x.var(axis=-1, keepdims=True)
    return (x - mean) / np.sqrt(var + eps) * scale + bias


def _elu(x, alpha=1.0):
    return np.where(x > 0, x, alpha * (np.exp(np.minimum(x, 0.0)) - 1.0))


class PolicyNode(Node):
    def __init__(self):
        super().__init__("pusht_policy")
        default_model_path = os.path.join(
            get_package_share_directory("franka_pusht"), "models", "pusht_best_policy.npz")
        self.declare_parameter("observation_topic", "/pusht/observation")
        self.declare_parameter("policy_action_topic", "/pusht/policy_action")
        self.declare_parameter("model_path", default_model_path)
        self.pub = self.create_publisher(Float32MultiArray, self.get_parameter("policy_action_topic").value, 10)
        self.create_subscription(Float32MultiArray, self.get_parameter("observation_topic").value, self._on_obs, 10)

        model_path = self.get_parameter("model_path").value
        try:
            params = np.load(model_path)
            self.weights = {k: params[k] for k in params.files}
            self.get_logger().info(f"Loaded policy weights from {model_path}")
        except Exception as e:
            self.weights = None
            self.get_logger().error(f"Failed to load policy weights from {model_path}: {e}; publishing zero actions")

    def _forward(self, obs):
        w = self.weights
        x = obs @ w["Dense_0.kernel"] + w["Dense_0.bias"]
        x = _layer_norm(x, w["LayerNorm_0.scale"], w["LayerNorm_0.bias"])
        x = _elu(x)
        x = x @ w["Dense_1.kernel"] + w["Dense_1.bias"]
        x = _elu(x)
        x = x @ w["Dense_2.kernel"] + w["Dense_2.bias"]
        x = _elu(x)
        x = x @ w["Dense_3.kernel"] + w["Dense_3.bias"]
        return x

    def _on_obs(self, msg):
        obs = np.asarray(msg.data, dtype=np.float32)
        action = np.zeros(2, dtype=np.float32)
        if self.weights is not None and obs.shape == (24,) and np.all(np.isfinite(obs)):
            action = self._forward(obs).astype(np.float32)
            if not np.all(np.isfinite(action)):
                action = np.zeros(2, dtype=np.float32)
        self.pub.publish(Float32MultiArray(data=action.tolist()))


def main(args=None):
    rclpy.init(args=args); node=PolicyNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
