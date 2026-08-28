"""PS5 teleoperation front-end for the bulb-screwing pipeline.

Publishes normalized 5-D actions for franka_bulbscrew; it never talks to Servo
directly. Layout matches bulbscrew_mjx:

    action[0:3]  end-effector velocity x, y, z
    action[3]    wrist yaw rate -- the screwing DoF
    action[4]    gripper, -1 fully closed .. +1 fully open

Controller mapping (mirrors franka_pusht's, extended for the extra DoF):

    L1 (4)          dead-man; nothing moves unless held
    left stick      x / y translation
    R1 (5) / L2 (2) up / down (z)
    right stick X   wrist yaw -- screw in and out
    R2 (7) / Square close / open the gripper (latched, not momentary)
    PS (10)         home
    Cross (0)       start episode (homes first)
    Circle (1)      stop episode (saves, then homes)

The gripper is latched rather than proportional because a bulb needs a settled
grip: a stick axis would jitter the commanded width every tick and franka_gripper
would keep re-issuing goals. Latching also means releasing the dead-man does not
drop the bulb -- safety_node keeps publishing the last width.
"""

import math
import threading
import time

import rclpy
try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        Constraints,
        JointConstraint,
        MotionPlanRequest,
        PlanningOptions,
    )
    from moveit_msgs.srv import ServoCommandType
except ImportError:  # allows controller/data-pipeline testing without MoveIt
    MoveGroup = None
    ServoCommandType = None
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool, Trigger


JOY_TOPIC = "/joy"
ACTION_TOPIC = "/bulbscrew/teleop_action"
DEADMAN_TOPIC = "/bulbscrew/deadman"
PUBLISH_RATE = 20.0

ENABLE_BUTTON = 4         # L1
HOME_BUTTON = 10          # PS
START_COLLECT_BUTTON = 0  # Cross
STOP_COLLECT_BUTTON = 1   # Circle
UP_BUTTON = 5             # R1
DOWN_BUTTON = 2           # L2 (digital)
CLOSE_BUTTON = 7          # R2 (digital)
OPEN_BUTTON = 3           # Square
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = 3
# Sign of each axis in the seat frame. Flip if the arm moves opposite the stick.
SIGN_X = -1.0
SIGN_Y = -1.0
SIGN_YAW = -1.0
DEADZONE = 0.05
Z_SPEED = 0.6             # fraction of max_linear_speed for the up/down buttons
JOY_TIMEOUT_S = 0.15

PLANNING_GROUP = "fr3_arm"
# fr3_hand_tcp = [0.671, -0.032, 0.250] in fr3_link0, tool exactly vertical
# (0.00 deg tilt), minimum joint-limit margin 78.7 deg -- far clear of Servo's
# joint_limit_margin (0.10 rad = 5.7 deg), inside which it halts on almost any
# motion. Solved via /compute_ik on 2026-08-28.
#
# Deliberately NEUTRAL rather than poised at the bulb: it sits at the midpoint
# between the bulb start and the socket seat, 25.8 cm from one and 27.3 cm from
# the other, so every episode begins with the same reach-then-carry structure
# the sim has. A home already at the grasp would also imply a precision the
# reset does not have, since the bulb is placed by hand and varies slightly.
#
# Note this was NOT chosen to match bulbscrew_mjx's QHOME. This task is being
# set up real-first, so the sim's QHOME gets set FROM this value instead --
# see SIM_ALIGNMENT.md. (Selecting for closeness to the sim's shipped posture
# would have been the wrong metric: its tool sits at [0.30, 0, 0.03], low and
# close, which suits neither this socket nor this bulb.)
#
# Must stay equal to servo_ik_node's nullspace_target.
HOME_JOINTS = {
    "fr3_joint1": 0.201812,
    "fr3_joint2": 0.461781,
    "fr3_joint3": -0.293619,
    "fr3_joint4": -1.651913,
    "fr3_joint5": 0.149214,
    "fr3_joint6": 2.091641,
    "fr3_joint7": -0.853753,
}
JOINT_TOL = 0.01
VEL_SCALE = 0.2
ACC_SCALE = 0.2
MOVE_ACTION = "/move_action"
PAUSE_SERVO = "/servo_node/pause_servo"
START_COLLECT_SRV = "/bulbscrew/start_episode"
STOP_COLLECT_SRV = "/bulbscrew/finish_episode"
SERVO_SETTLE_S = 0.2
SWITCH_COMMAND_TYPE_SRV = "/servo_node/switch_command_type"


class BulbScrewTeleop(Node):
    def __init__(self):
        super().__init__("bulbscrew_ps5_teleop")
        self._cbg = ReentrantCallbackGroup()
        self._joy = None
        self._joy_time = None
        self._prev = {}
        self._busy = False
        self.declare_parameter("grip_open", True)
        self._grip_open = bool(self.get_parameter("grip_open").value)
        self._action_pub = self.create_publisher(Float32MultiArray, ACTION_TOPIC, 10)
        self._deadman_pub = self.create_publisher(Bool, DEADMAN_TOPIC, 10)
        self.create_subscription(Joy, JOY_TOPIC, self._on_joy, 10, callback_group=self._cbg)
        self.create_timer(1.0 / PUBLISH_RATE, self._on_timer, callback_group=self._cbg)
        self._servo_pause = self.create_client(SetBool, PAUSE_SERVO, callback_group=self._cbg)
        self._collect_start = self.create_client(Trigger, START_COLLECT_SRV, callback_group=self._cbg)
        self._collect_stop = self.create_client(Trigger, STOP_COLLECT_SRV, callback_group=self._cbg)
        self._move = (ActionClient(self, MoveGroup, MOVE_ACTION, callback_group=self._cbg)
                      if MoveGroup is not None else None)
        if self._move is None:
            self.get_logger().warning("moveit_msgs unavailable: Home button is disabled")
        # servo_ik_node owns the Servo command mode (JOINT_JOG); the two must
        # not fight over it, so this is opt-out.
        self.declare_parameter("set_servo_command_type", False)
        self._switch = (self.create_client(ServoCommandType, SWITCH_COMMAND_TYPE_SRV,
                                           callback_group=self._cbg)
                        if ServoCommandType is not None
                        and self.get_parameter("set_servo_command_type").value else None)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joy(self, msg):
        self._joy = msg
        self._joy_time = self._now()
        self._edge(msg, HOME_BUTTON, self._on_home)
        self._edge(msg, START_COLLECT_BUTTON, self._on_start_collect)
        self._edge(msg, STOP_COLLECT_BUTTON, self._on_stop_collect)
        self._edge(msg, CLOSE_BUTTON, lambda: setattr(self, "_grip_open", False))
        self._edge(msg, OPEN_BUTTON, lambda: setattr(self, "_grip_open", True))

    def _edge(self, msg, index, callback):
        if index >= len(msg.buttons):
            return
        pressed = bool(msg.buttons[index])
        if pressed and not self._prev.get(index, False):
            callback()
        self._prev[index] = pressed

    def _held(self, joy, index):
        return index < len(joy.buttons) and bool(joy.buttons[index])

    def _on_timer(self):
        joy = self._joy
        fresh = (joy is not None and self._joy_time is not None
                 and self._now() - self._joy_time <= JOY_TIMEOUT_S)
        enabled = (fresh and not self._busy and ENABLE_BUTTON < len(joy.buttons)
                   and bool(joy.buttons[ENABLE_BUTTON]))
        action = [0.0, 0.0, 0.0, 0.0, 1.0 if self._grip_open else -1.0]
        if enabled and max(AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X) < len(joy.axes):
            sx = SIGN_X * float(joy.axes[AXIS_LEFT_Y])
            sy = SIGN_Y * float(joy.axes[AXIS_LEFT_X])
            if math.hypot(sx, sy) > DEADZONE:
                action[0], action[1] = sx, sy
            action[2] = Z_SPEED * ((1.0 if self._held(joy, UP_BUTTON) else 0.0)
                                   - (1.0 if self._held(joy, DOWN_BUTTON) else 0.0))
            yaw = SIGN_YAW * float(joy.axes[AXIS_RIGHT_X])
            if abs(yaw) > DEADZONE:
                action[3] = yaw
        self._deadman_pub.publish(Bool(data=enabled))
        self._action_pub.publish(Float32MultiArray(data=action))

    def _on_home(self):
        if self._move is None or self._busy:
            return
        threading.Thread(target=self._home_sequence, daemon=True).start()

    def _home_sequence(self):
        self._busy = True
        try:
            ok = self._home_and_settle()
            self.get_logger().info("Home reached" if ok else "Home failed")
        finally:
            self._busy = False

    def _home_and_settle(self):
        """Pause Servo, drive to HOME_JOINTS, resume Servo. Returns success."""
        self._publish_stop_action()
        paused = self._call_sync(self._servo_pause, SetBool.Request(data=True))
        if not (paused and paused.success):
            self.get_logger().error(
                "Failed to pause Servo; not homing (Servo would fight MoveGroup's execution)")
            return False
        time.sleep(SERVO_SETTLE_S)
        ok = self._send_home_goal()
        resumed = self._call_sync(self._servo_pause, SetBool.Request(data=False))
        if not (resumed and resumed.success):
            self.get_logger().error("Failed to resume Servo after homing")
        self._publish_stop_action()
        return ok

    def _on_start_collect(self):
        if self._busy:
            return
        threading.Thread(target=self._start_collect_sequence, daemon=True).start()

    def _start_collect_sequence(self):
        # Home first so every episode starts from the same pose and the transit
        # motion is never recorded.
        self._busy = True
        try:
            ok = self._home_and_settle()
            self.get_logger().info("Home reached before episode" if ok else
                                   "Home failed before episode; not starting")
            if ok:
                self._call_sync(self._collect_start)
        finally:
            self._busy = False

    def _on_stop_collect(self):
        if self._busy:
            return
        threading.Thread(target=self._stop_collect_sequence, daemon=True).start()

    def _stop_collect_sequence(self):
        # Finish the episode before homing, so the return motion is not recorded
        # and disturbing the scene during it cannot contaminate the data.
        self._call_sync(self._collect_stop)
        self._busy = True
        try:
            ok = self._home_and_settle()
            self.get_logger().info("Home reached after episode" if ok else "Home failed after episode")
        finally:
            self._busy = False

    def _publish_stop_action(self):
        self._deadman_pub.publish(Bool(data=False))
        self._action_pub.publish(Float32MultiArray(
            data=[0.0, 0.0, 0.0, 0.0, 1.0 if self._grip_open else -1.0]))

    def _build_home_goal(self):
        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = VEL_SCALE
        request.max_acceleration_scaling_factor = ACC_SCALE
        constraints = Constraints()
        for name, position in HOME_JOINTS.items():
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = JOINT_TOL
            constraint.tolerance_below = JOINT_TOL
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints.append(constraints)
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        return goal

    def _send_home_goal(self):
        if self._move is None or not self._move.wait_for_server(timeout_sec=5.0):
            return False
        future = self._move.send_goal_async(self._build_home_goal())
        if not self._await(future, 10.0):
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MoveGroup rejected the home goal -- is move_group running?")
            return False
        result_future = handle.get_result_async()
        if not self._await(result_future, 30.0):
            return False
        result = result_future.result()
        return bool(result and result.result.error_code.val == 1)

    def _await(self, future, timeout):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return False
            time.sleep(0.01)
        return True

    def _call_sync(self, client, request=None):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(request if request is not None else Trigger.Request())
        return future.result() if self._await(future, 5.0) else None


def main(args=None):
    rclpy.init(args=args)
    node = BulbScrewTeleop()
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
