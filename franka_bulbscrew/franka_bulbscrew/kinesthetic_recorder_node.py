"""Record hand-guided demonstrations and label them with derived actions.

During kinesthetic teaching nobody commands the robot, so there is no
executed_action to record -- the operator moves the arm directly. The actions
are therefore *derived* from the motion that resulted, inverting the same
mapping safety_node applies forward:

    action[0:3] = ee_velocity / max_linear_speed          (in fr3_link0)
    action[3]   = wrist yaw rate / max_yaw_rate
    action[4]   = 2 * gripper_width / grip_max - 1

This is standard for kinesthetic demos, and it keeps the recorded episodes in
exactly the format data_collection_node produces, so both sources train the same
way. Velocities are finite-differenced from the tool pose and low-pass filtered,
because hand motion is noisy at 20 Hz and raw differences would label the expert
as jittering.

Gamepad, since both hands are on the robot:

    Circle (1)     start episode
    Square (3)     stop episode and save
    Cross/X (0)    close the gripper
    Triangle (2)   open the gripper
    PS (10)        home the arm, then hand it back to guiding

Homing matters here even though the arm is moved by hand: bulbscrew_mjx resets
to a fixed arm pose every episode, so demonstrations that each begin from
wherever the arm was left would not share a start state. Pressing PS switches to
fr3_arm_controller, drives to HOME_JOINTS, and switches back to gravity
compensation -- the arm goes briefly stiff and moves on its own, so let go
first.

The recorder drives the gripper itself here. safety_node is deliberately not
running during hand-guiding -- nothing should be commanding the arm -- but it is
what normally publishes the width, so without this the gripper would be dead.

The same services also work, for scripting:
    /bulbscrew/start_episode   /bulbscrew/finish_episode   /bulbscrew/abort_episode

CAVEAT worth knowing before training on this. Guiding by hand means *you* choose
the elbow configuration, while at deployment servo_ik_node chooses it via its
null-space term. arm_qpos and arm_qvel are 14 of the 25 observation dimensions,
so those distributions can diverge -- the same mismatch that made the PushT
policy stall a few steps into every episode. Compare a recorded arm_qpos against
what servo_ik_node produces for the same tool path before trusting a large
dataset; if they differ, replay the tool path through the normal stack and
record that instead.
"""

import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                                 PlanningOptions)
except ImportError:
    MoveGroup = None
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger

from .ps5_teleop_node import HOME_JOINTS, PLANNING_GROUP, JOINT_TOL, VEL_SCALE, ACC_SCALE


class KinestheticRecorder(Node):
    def __init__(self):
        super().__init__("bulbscrew_kinesthetic_recorder")
        defaults = {
            "control_rate_hz": 20.0,
            "observation_topic": "/bulbscrew/observation",
            "observation_valid_topic": "/bulbscrew/observation_valid",
            "dataset_root": "~/bulbscrew_data",
            "robot_base_frame": "fr3_link0",
            "tcp_frame": "fr3_hand_tcp",
            # Must match safety_node, or the derived actions are on a different
            # scale from anything the robot would later execute.
            "max_linear_speed": 0.10,
            "max_yaw_rate": 0.75,
            "grip_max": 0.04,
            # Exponential smoothing on the derived velocity. Hand motion is
            # noisy at 20 Hz; without this the expert looks like it jitters.
            "velocity_filter_alpha": 0.4,
            "gripper_command_topic": "/bulbscrew/gripper_width",
            "joy_topic": "/joy",
            "start_button": 1,    # Circle
            "stop_button": 3,     # Square
            "close_button": 0,    # Cross (X)
            "open_button": 2,     # Triangle
            "home_button": 10,    # PS
            "switch_service": "/controller_manager/switch_controller",
            "move_action": "/move_action",
            "arm_controller": "fr3_arm_controller",
            "gravity_controller": "gravity_compensation_example_controller",
            # MoveIt's controller manager caches controller states. The switch
            # service returns as soon as controller_manager has done the swap,
            # but MoveIt can still believe fr3_arm_controller is inactive and
            # refuse to execute -- planning succeeds, execution fails. Waiting
            # lets that view catch up. Reproduced directly: the same sequence
            # fails with no wait and succeeds with one.
            "controller_settle_s": 0.8,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        g = self.get_parameter
        root = Path(os.path.expanduser(g("dataset_root").value))
        self.session = root / datetime.now().strftime("kinesthetic_%Y%m%d_%H%M%S")
        self.session.mkdir(parents=True, exist_ok=True)
        (self.session / "metadata.json").write_text(json.dumps({
            "control_rate_hz": g("control_rate_hz").value,
            "observation_dim": 25, "action_dim": 5,
            "observation": "bulb_tip_rel_seat_xyz + bulb_quat_rel_seat_wxyz + "
                           "ee_rel_grasp_xyz + gripper_width + q7 + dq7",
            "source": "kinesthetic (hand-guided); actions derived from tool motion",
            "max_linear_speed": g("max_linear_speed").value,
            "max_yaw_rate": g("max_yaw_rate").value,
        }, indent=2))

        self.obs = None
        self.obs_valid = False
        self.prev = None            # (time, position, yaw)
        self.vel = np.zeros(4)      # filtered [vx, vy, vz, yaw_rate]
        self.samples = []
        self.active = False
        self.episode_id = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Float32MultiArray, g("observation_topic").value,
                                 self._on_obs, 20)
        from std_msgs.msg import Bool
        self.create_subscription(Bool, g("observation_valid_topic").value,
                                 lambda m: setattr(self, "obs_valid", bool(m.data)), 10)
        self.grip_pub = self.create_publisher(Float32, g("gripper_command_topic").value, 10)
        self._prev_buttons = {}
        self.create_subscription(Joy, g("joy_topic").value, self._on_joy, 10)
        self.create_service(Trigger, "/bulbscrew/start_episode", self._start)
        self.create_service(Trigger, "/bulbscrew/finish_episode", self._finish)
        self.create_service(Trigger, "/bulbscrew/abort_episode", self._abort)
        self._homing = False
        # The home sequence runs on a background thread (it blocks for seconds),
        # so its service and action clients need a reentrant group served by a
        # MultiThreadedExecutor. Under the default single-threaded spin, a goal
        # sent from another thread never sees the server and wait_for_server
        # simply times out -- which is exactly how homing was failing, silently,
        # after ~5 s. ps5_teleop_node uses the same arrangement for this reason.
        self._cbg = ReentrantCallbackGroup()
        self._switch = self.create_client(SwitchController, g("switch_service").value,
                                          callback_group=self._cbg)
        self._move = (ActionClient(self, MoveGroup, g("move_action").value,
                                   callback_group=self._cbg)
                      if MoveGroup is not None else None)
        self.create_timer(1.0 / g("control_rate_hz").value, self._tick)

    def _on_obs(self, msg):
        self.obs = np.asarray(msg.data, dtype=np.float32)

    def _on_joy(self, msg):
        g = self.get_parameter
        def edge(index):
            if index >= len(msg.buttons):
                return False
            pressed = bool(msg.buttons[index])
            fired = pressed and not self._prev_buttons.get(index, False)
            self._prev_buttons[index] = pressed
            return fired
        if edge(int(g("start_button").value)):
            r = self._start(None, Trigger.Response())
            self.get_logger().info(f"start: {r.message}")
        if edge(int(g("stop_button").value)):
            r = self._finish(None, Trigger.Response())
            self.get_logger().info(f"stop: {r.message}")
        # Gripper: latched, so releasing a button never drops the bulb.
        if edge(int(g("close_button").value)):
            self.grip_pub.publish(Float32(data=0.0))
            self.get_logger().info("gripper: close")
        if edge(int(g("open_button").value)):
            self.grip_pub.publish(Float32(data=float(g("grip_max").value)))
            self.get_logger().info("gripper: open")
        if edge(int(g("home_button").value)) and not self._homing:
            threading.Thread(target=self._home_sequence, daemon=True).start()

    def _switch_controllers(self, activate, deactivate):
        if not self._switch.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("controller_manager unavailable")
            return False
        request = SwitchController.Request()
        request.activate_controllers = [activate]
        request.deactivate_controllers = [deactivate]
        # STRICT: refuse rather than half-switch, which would leave the arm with
        # nothing holding it up.
        request.strictness = SwitchController.Request.STRICT
        future = self._switch.call_async(request)
        t = time.time()
        while not future.done() and time.time() - t < 10.0:
            time.sleep(0.02)
        result = future.result()
        return bool(result is not None and result.ok)

    def _home_sequence(self):
        """Stiffen, drive to HOME_JOINTS, then hand back to guiding."""
        g = self.get_parameter
        self._homing = True
        arm = g("arm_controller").value
        gravity = g("gravity_controller").value
        try:
            if self.active:
                self.get_logger().warning("finish the episode before homing")
                return
            if self._move is None:
                self.get_logger().error("moveit_msgs unavailable: cannot home")
                return
            self.get_logger().warning("HOMING -- the arm is about to stiffen and move; let go")
            if not self._switch_controllers(arm, gravity):
                self.get_logger().error("could not switch to the arm controller; still guiding")
                return
            time.sleep(float(g("controller_settle_s").value))
            ok = self._send_home_goal()
            # Always hand back to guiding, even if the motion failed, so the arm
            # is never left stiff when the operator expects to move it by hand.
            back = self._switch_controllers(gravity, arm)
            self.get_logger().info(
                ("homed" if ok else "home FAILED") +
                ("; guiding again" if back else "; COULD NOT return to guiding"))
        finally:
            self._homing = False

    def _send_home_goal(self):
        if not self._move.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "MoveGroup action server not reachable -- is move_group running?")
            return False
        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = VEL_SCALE
        request.max_acceleration_scaling_factor = ACC_SCALE
        constraints = Constraints()
        for name, position in HOME_JOINTS.items():
            c = JointConstraint()
            c.joint_name = name; c.position = position
            c.tolerance_above = JOINT_TOL; c.tolerance_below = JOINT_TOL; c.weight = 1.0
            constraints.joint_constraints.append(c)
        request.goal_constraints.append(constraints)
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        future = self._move.send_goal_async(goal)
        t = time.time()
        while not future.done() and time.time() - t < 10.0:
            time.sleep(0.02)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MoveGroup rejected the home goal")
            return False
        rf = handle.get_result_async()
        t = time.time()
        while not rf.done() and time.time() - t < 30.0:
            time.sleep(0.02)
        res = rf.result()
        if res is None:
            self.get_logger().error("home goal timed out with no result")
            return False
        code = res.result.error_code.val
        if code != 1:
            self.get_logger().error(f"home motion failed, MoveItErrorCode {code}")
        return code == 1

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tool(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("robot_base_frame").value,
                self.get_parameter("tcp_frame").value, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        q = np.array([r.w, r.x, r.y, r.z])
        # yaw about the world z axis, which is the screwing DoF
        yaw = np.arctan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                         1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))
        return np.array([t.x, t.y, t.z]), float(yaw)

    def _tick(self):
        g = self.get_parameter
        tool = self._tool()
        if tool is None:
            return
        position, yaw = tool
        now = self._now()
        if self.prev is not None:
            dt = now - self.prev[0]
            if dt > 1e-4:
                v = (position - self.prev[1]) / dt
                dyaw = np.arctan2(np.sin(yaw - self.prev[2]), np.cos(yaw - self.prev[2])) / dt
                raw = np.array([v[0], v[1], v[2], dyaw])
                a = float(g("velocity_filter_alpha").value)
                self.vel = a * raw + (1.0 - a) * self.vel
        self.prev = (now, position, yaw)

        if not self.active or self.obs is None or not self.obs_valid:
            return
        if self.obs.shape != (25,) or not np.all(np.isfinite(self.obs)):
            return
        speed = g("max_linear_speed").value
        yaw_rate = g("max_yaw_rate").value
        grip_max = g("grip_max").value
        width = float(self.obs[10])                    # total opening
        action = np.array([
            self.vel[0] / speed, self.vel[1] / speed, self.vel[2] / speed,
            self.vel[3] / yaw_rate,
            np.clip(width / grip_max - 1.0, -1.0, 1.0),   # width is both fingers
        ], dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)
        self.samples.append((self.obs.copy(), action))

    def _start(self, _, response):
        if self.active:
            response.success = False; response.message = "episode already active"
            return response
        if self.obs is None or not self.obs_valid:
            response.success = False; response.message = "observation unavailable or invalid"
            return response
        self.samples = []
        self.vel = np.zeros(4)
        self.active = True
        response.success = True; response.message = "recording (hand-guided)"
        return response

    def _finish(self, _, response):
        if not self.active or len(self.samples) < 2:
            response.success = False; response.message = "no active non-empty episode"
            return response
        self.active = False
        obs = np.stack([s[0] for s in self.samples])
        act = np.stack([s[1] for s in self.samples])
        # transitions: state -> next_state, so the last sample has no successor
        states, actions, next_states = obs[:-1], act[:-1], obs[1:]
        absorbing = np.zeros(len(states), dtype=np.float32)
        absorbing[-1] = 1.0
        path = self.session / f"episode_{self.episode_id:04d}.npz"
        np.savez(path, states=states, actions=actions, next_states=next_states,
                 absorbing=absorbing, rewards=np.zeros(len(states), dtype=np.float32))
        self.episode_id += 1
        self.samples = []
        self.get_logger().info(f"saved {len(states)} transitions -> {path}")
        response.success = True; response.message = str(path)
        return response

    def _abort(self, _, response):
        count = len(self.samples)
        self.active = False
        self.samples = []
        response.success = True
        response.message = f"aborted and discarded {count} samples"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = KinestheticRecorder()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
