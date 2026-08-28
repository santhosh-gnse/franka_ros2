"""Record 20 Hz bulb-screwing transitions and save one NPZ file per episode."""

import json
import os
from datetime import datetime
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger


class DataCollectionNode(Node):
    def __init__(self):
        super().__init__("bulbscrew_data_collection")
        defaults = {"control_rate_hz": 20.0, "observation_timeout_s": 0.15,
                    "action_timeout_s": 0.15, "observation_topic": "/bulbscrew/observation",
                    "executed_action_topic": "/bulbscrew/executed_action", "dataset_root": "~/bulbscrew_data"}
        for name, value in defaults.items(): self.declare_parameter(name, value)
        root = Path(os.path.expanduser(self.get_parameter("dataset_root").value))
        self.session = root / datetime.now().strftime("session_%Y%m%d_%H%M%S")
        self.session.mkdir(parents=True, exist_ok=True)
        (self.session / "metadata.json").write_text(json.dumps({
            "control_rate_hz": self.get_parameter("control_rate_hz").value,
            "observation_dim": 25, "action_dim": 5,
            "observation": "bulb_tip_rel_seat_xyz + bulb_quat_rel_seat_wxyz + ee_rel_neck_xyz + gripper_width + q7 + dq7",
        }, indent=2))
        self.obs = None; self.action = None; self.obs_s = None; self.action_s = None
        self.active = False; self.transitions = []; self.episode_id = 0
        self.create_subscription(Float32MultiArray, self.get_parameter("observation_topic").value, self._on_obs, 20)
        self.create_subscription(Float32MultiArray, self.get_parameter("executed_action_topic").value, self._on_action, 20)
        self.create_service(Trigger, "/bulbscrew/start_episode", self._start)
        self.create_service(Trigger, "/bulbscrew/finish_episode", self._finish)
        self.create_service(Trigger, "/bulbscrew/abort_episode", self._abort)
        self.create_timer(1.0 / self.get_parameter("control_rate_hz").value, self._tick)

    def _now(self): return self.get_clock().now().nanoseconds * 1e-9
    def _on_obs(self, msg): self.obs = np.asarray(msg.data, dtype=np.float32); self.obs_s = self._now()
    def _on_action(self, msg): self.action = np.asarray(msg.data, dtype=np.float32); self.action_s = self._now()
    def _fresh(self):
        now = self._now()
        return (self.obs is not None and self.obs.shape == (25,) and self.action is not None and
                self.action.shape == (5,) and now-self.obs_s <= self.get_parameter("observation_timeout_s").value and
                now-self.action_s <= self.get_parameter("action_timeout_s").value and
                np.all(np.isfinite(self.obs)) and np.all(np.isfinite(self.action)))
    def _start(self, _, response):
        if self.active: response.success=False; response.message="episode already active"; return response
        if not self._fresh(): response.success=False; response.message="observation/action unavailable or stale"; return response
        self.transitions=[]; self.previous_obs=self.obs.copy(); self.previous_action=self.action.copy(); self.active=True
        response.success=True; response.message="episode started"; return response
    def _tick(self):
        if not self.active or not self._fresh(): return
        current = self.obs.copy()
        self.transitions.append((self.previous_obs, self.previous_action, current))
        self.previous_obs = current; self.previous_action = self.action.copy()
    def _finish(self, _, response):
        if not self.active or not self.transitions:
            response.success=False; response.message="no active non-empty episode"; return response
        self.active=False
        states=np.stack([x[0] for x in self.transitions]); actions=np.stack([x[1] for x in self.transitions])
        next_states=np.stack([x[2] for x in self.transitions]); absorbing=np.zeros(len(states), dtype=np.float32)
        absorbing[-1]=1.0; rewards=np.zeros(len(states), dtype=np.float32)
        path=self.session / f"episode_{self.episode_id:04d}.npz"
        np.savez(path, states=states, actions=actions, next_states=next_states,
                 absorbing=absorbing, rewards=rewards)
        self.episode_id += 1; self.transitions=[]
        response.success=True; response.message=str(path); return response
    def _abort(self, _, response):
        count=len(self.transitions); self.active=False; self.transitions=[]
        response.success=True; response.message=f"aborted and discarded {count} transitions"; return response


def main(args=None):
    rclpy.init(args=args); node=DataCollectionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
