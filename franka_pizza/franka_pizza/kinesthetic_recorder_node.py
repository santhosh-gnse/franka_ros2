"""Record hand-guided sauce-spreading demonstrations, labelled with derived
actions.

During kinesthetic teaching nobody commands the robot, so there is no
executed_action to record -- the operator moves the arm directly. The action is
therefore *derived* from the horizontal velocity that resulted:

    action[0:2] = clip(ee_velocity_xy / max_linear_speed, -1, 1)   (in fr3_link0)

Each saved episode also carries `raw_actions_mps`: the same ee_velocity_xy in
m/s, filtered but NOT normalized or clipped. The clip above is lossy and not
always benign -- it applies per-axis, so a step where only one axis exceeds
max_linear_speed saves a DIFFERENT direction than was actually moved, not just
a slower one. Post-process raw_actions_mps if max_linear_speed ever needs
revisiting, rather than re-collecting; `states`/`next_states` were never
scaled at all, so only `actions` needed this.

Height and orientation are NOT part of the action: the task holds both fixed
at deployment time, so there is nothing for a 2-D policy to command there,
matching PushT. During kinesthetic teaching, though, height and orientation
are NOT actively locked -- same as PushT/BulbScrew, guiding_mode.py runs
plain GravityCompensationExampleController (fully compliant, all axes), not
CartesianImpedanceExampleController. An earlier attempt at actively locking
height during teaching (x/y soft, z/roll/pitch/yaw stiff) hit sustained
oscillation the moment the tool touched the board -- a rigid-on-rigid contact
instability inherent to holding a stiff Cartesian spring against an
undamped surface, not a tuning mistake (see franka_example_controllers'
CartesianImpedanceExampleController, which got a real fix for a *separate*
free-space damping bug found along the way, but that controller is not used
here). The operator's own hand keeps height/orientation close enough to
correct during a real demonstration; enforcing the lock on a deployed
policy's actions (which can drift in ways a human demonstrating the task
would not) is a safety_node/servo_ik_node's job, not implemented yet -- see
pizza_common.yaml's z_hold_*/orientation_hold_* keys.
Velocity is finite-differenced from the tool pose and low-pass filtered,
because hand motion is noisy at 20 Hz and raw differences would label the
expert as jittering.

Gamepad, since both hands are on the robot:

    Circle (1)     start episode
    Square (3)     stop episode and save
    PS (10)        home the arm, then hand it back to guiding

Homing matters even though the arm is moved by hand: episodes should share a
start state (the board centre), not wherever the arm was left. Pressing PS
switches to fr3_arm_controller, drives to HOME_JOINTS, and switches back to
gravity-compensated guiding -- the arm goes briefly stiff and moves on its
own, so let go first.

The same services also work, for scripting:
    /pizza/start_episode   /pizza/finish_episode   /pizza/abort_episode

CAVEAT worth knowing before training on this, ported from franka_bulbscrew:
guiding by hand means *you* choose the elbow configuration, while a deployed
policy's controller would choose it via a null-space term (not yet built for
this package). arm_qpos and arm_qvel are 14 of the 17 observation dimensions,
so those distributions can diverge if that controller is ever added. Compare a
recorded arm_qpos against what it produces for the same tool path before
trusting a large dataset.
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
from controller_manager_msgs.srv import ListControllers, SwitchController
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
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger

PLANNING_GROUP = "fr3_arm"
JOINT_TOL = 0.01
VEL_SCALE = 0.2
ACC_SCALE = 0.2

# Arm joint configuration that puts fr3_pusher_tcp at home_position (the board
# centre, 2 cm above the plank), tool pointing straight down. Solved via
# /compute_ik (sweeping wrist yaw, keeping the best joint margin) and
# CONFIRMED on the real robot 2026-09-08: driven there, achieved pose 1.3 mm
# from target, 0.29 deg from straight down. Minimum joint-limit margin
# 64.7 deg (joint 4); every other joint 66-142 deg. Must stay equal to
# pizza_robot.yaml's home_joint_positions.
HOME_JOINTS = {
    "fr3_joint1": -0.423000,
    "fr3_joint2": 0.625100,
    "fr3_joint3": 0.423600,
    "fr3_joint4": -1.912700,
    "fr3_joint5": -0.390000,
    "fr3_joint6": 2.456700,
    "fr3_joint7": -1.858600,
}


class KinestheticRecorder(Node):
    def __init__(self):
        super().__init__("pizza_kinesthetic_recorder")
        defaults = {
            "control_rate_hz": 20.0,
            "observation_topic": "/pizza/observation",
            "observation_valid_topic": "/pizza/observation_valid",
            "dataset_root": "~/pizza_data",
            "robot_base_frame": "fr3_link0",
            "tcp_frame": "fr3_pusher_tcp",
            # Must match whatever later servos a deployed policy's actions, or
            # the recorded labels are on a different scale than anything the
            # robot would execute.
            "max_linear_speed": 0.10,
            # Exponential smoothing on the derived velocity. Hand motion is
            # noisy at 20 Hz; without this the expert looks like it jitters.
            "velocity_filter_alpha": 0.4,
            "joy_topic": "/joy",
            "start_button": 1,    # Circle
            "stop_button": 3,     # Square
            "home_button": 10,    # PS
            "switch_service": "/controller_manager/switch_controller",
            "list_service": "/controller_manager/list_controllers",
            "move_action": "/move_action",
            "arm_controller": "fr3_arm_controller",
            # Plain gravity compensation (see guiding_mode.py) -- height and
            # orientation are NOT locked during teaching, only at deployment.
            "guiding_controller": "gravity_compensation_example_controller",
            # MoveIt's controller manager caches controller states. The switch
            # service returns as soon as controller_manager has done the swap,
            # but MoveIt can still believe fr3_arm_controller is inactive and
            # refuse to execute -- planning succeeds, execution fails. Waiting
            # lets that view catch up (ported from franka_bulbscrew, where this
            # was reproduced directly: the same sequence fails with no wait and
            # succeeds with one).
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
            "observation_dim": 17, "action_dim": 2,
            "observation": "ee_rel_home_xyz + q7 + dq7",
            "source": "kinesthetic (hand-guided); actions derived from tool motion",
            "max_linear_speed": g("max_linear_speed").value,
            "raw_actions_mps": "per-episode field, m/s, filtered but NOT normalized or "
                               "clipped -- actions = clip(raw_actions_mps / max_linear_speed, "
                               "-1, 1). Re-derive actions at a different max_linear_speed from "
                               "this instead of re-collecting.",
        }, indent=2))

        self.obs = None
        self.obs_valid = False
        self.prev = None            # (time, position_xy)
        self.vel = np.zeros(2)      # filtered [vx, vy]
        self.samples = []
        self.active = False
        self.episode_id = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Float32MultiArray, g("observation_topic").value,
                                 self._on_obs, 20)
        self.create_subscription(Bool, g("observation_valid_topic").value,
                                 lambda m: setattr(self, "obs_valid", bool(m.data)), 10)
        self._prev_buttons = {}
        self.create_subscription(Joy, g("joy_topic").value, self._on_joy, 10)
        self.create_service(Trigger, "/pizza/start_episode", self._start)
        self.create_service(Trigger, "/pizza/finish_episode", self._finish)
        self.create_service(Trigger, "/pizza/abort_episode", self._abort)
        self._homing = False
        # The home sequence runs on a background thread (it blocks for
        # seconds), so its service and action clients need a reentrant group
        # served by a MultiThreadedExecutor. Under the default single-threaded
        # spin, a goal sent from another thread never sees the server and
        # wait_for_server simply times out -- this bit franka_bulbscrew
        # silently for a while before being traced to exactly this.
        self._cbg = ReentrantCallbackGroup()
        self._list = self.create_client(ListControllers, g("list_service").value,
                                        callback_group=self._cbg)
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
        if edge(int(g("home_button").value)) and not self._homing:
            threading.Thread(target=self._home_sequence, daemon=True).start()

    def _active_controllers(self):
        """Names of the controllers currently active, or None if unknown."""
        if not self._list.wait_for_service(timeout_sec=5.0):
            return None
        future = self._list.call_async(ListControllers.Request())
        t = time.time()
        while not future.done() and time.time() - t < 5.0:
            time.sleep(0.02)
        result = future.result()
        if result is None:
            return None
        return {c.name for c in result.controller if c.state == "active"}

    def _switch_controllers(self, activate, deactivate):
        if not self._switch.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("controller_manager unavailable")
            return False
        # Ask only for changes that are actually needed. A STRICT switch fails
        # outright if told to deactivate a controller that is not active -- and
        # the gravity controller is not even LOADED unless guiding mode has
        # been entered at least once.
        active = self._active_controllers()
        request = SwitchController.Request()
        if active is None:                       # cannot tell; ask for both
            request.activate_controllers = [activate]
            request.deactivate_controllers = [deactivate]
        else:
            request.activate_controllers = [] if activate in active else [activate]
            request.deactivate_controllers = [deactivate] if deactivate in active else []
            if not request.activate_controllers and not request.deactivate_controllers:
                return True                      # already as requested
        # STRICT: refuse rather than half-switch, which would leave the arm
        # with nothing holding it up.
        request.strictness = SwitchController.Request.STRICT
        future = self._switch.call_async(request)
        t = time.time()
        while not future.done() and time.time() - t < 10.0:
            time.sleep(0.02)
        result = future.result()
        return bool(result is not None and result.ok)

    def _home_sequence(self):
        """Stiffen, drive to HOME_JOINTS, then hand back to gravity-compensated
        guiding. Gravity compensation is stateless and compliant in every
        axis, so switching back to it after homing needs nothing extra --
        unlike a stiffness-holding controller, there is no equilibrium or gain
        state to re-establish.
        """
        g = self.get_parameter
        self._homing = True
        arm = g("arm_controller").value
        guiding = g("guiding_controller").value
        try:
            if self.active:
                self.get_logger().warning("finish the episode before homing")
                return
            if self._move is None:
                self.get_logger().error("moveit_msgs unavailable: cannot home")
                return
            self.get_logger().warning("HOMING -- the arm is about to stiffen and move; let go")
            if not self._switch_controllers(arm, guiding):
                self.get_logger().error("could not switch to the arm controller; still guiding")
                return
            time.sleep(float(g("controller_settle_s").value))
            ok = self._send_home_goal()
            # Always hand back to guiding, even if the motion failed, so the
            # arm is never left stiff when the operator expects to move it by
            # hand.
            back = self._switch_controllers(guiding, arm)
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

    def _tool_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("robot_base_frame").value,
                self.get_parameter("tcp_frame").value, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y])

    def _tick(self):
        g = self.get_parameter
        xy = self._tool_xy()
        if xy is None:
            return
        now = self._now()
        if self.prev is not None:
            dt = now - self.prev[0]
            if dt > 1e-4:
                raw = (xy - self.prev[1]) / dt
                a = float(g("velocity_filter_alpha").value)
                self.vel = a * raw + (1.0 - a) * self.vel
        self.prev = (now, xy)

        if not self.active or self.obs is None or not self.obs_valid:
            return
        if self.obs.shape != (17,) or not np.all(np.isfinite(self.obs)):
            return
        speed = g("max_linear_speed").value
        action = np.clip(self.vel / speed, -1.0, 1.0).astype(np.float32)
        # self.vel is m/s, filtered but otherwise untouched -- saved alongside
        # the clipped/normalized action so max_linear_speed can be revisited
        # later without re-recording. Clipping is lossy and not always benign:
        # it is applied per-axis, so a step where only one axis exceeds the
        # cap saves a DIFFERENT direction than was actually moved, not just a
        # slower one.
        self.samples.append((self.obs.copy(), action, self.vel.astype(np.float32).copy()))

    def _start(self, _, response):
        if self.active:
            response.success = False; response.message = "episode already active"
            return response
        if self.obs is None or not self.obs_valid:
            response.success = False; response.message = "observation unavailable or invalid"
            return response
        self.samples = []
        self.vel = np.zeros(2)
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
        raw_vel = np.stack([s[2] for s in self.samples])
        # transitions: state -> next_state, so the last sample has no successor
        states, actions, next_states = obs[:-1], act[:-1], obs[1:]
        raw_actions_mps = raw_vel[:-1]
        absorbing = np.zeros(len(states), dtype=np.float32)
        absorbing[-1] = 1.0
        path = self.session / f"episode_{self.episode_id:04d}.npz"
        np.savez(path, states=states, actions=actions, next_states=next_states,
                 absorbing=absorbing, rewards=np.zeros(len(states), dtype=np.float32),
                 raw_actions_mps=raw_actions_mps)
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
