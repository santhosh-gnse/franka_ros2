"""PS5 teleoperation front-end for the PushT safety pipeline.

This preserves the controller mapping and homing logic from the original
``ps5/joy_to_twist.py``, but it never publishes directly to MoveIt Servo.
Instead it publishes normalized goal-frame actions for ``franka_pusht``.
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
except ImportError:  # Allows controller/data-pipeline testing without MoveIt.
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
ACTION_TOPIC = "/pusht/teleop_action"
DEADMAN_TOPIC = "/pusht/deadman"
PUBLISH_RATE = 20.0

# Existing, robot-tested PS5 mapping.
ENABLE_BUTTON = 4
HOME_BUTTON = 10
START_COLLECT_BUTTON = 0
STOP_COLLECT_BUTTON = 1
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
# Sign of each stick axis in the goal frame. ROS joy reports stick-up and
# stick-left as +1; flip a sign here if the arm moves opposite to the stick.
SIGN_X = -1.0   # stick up/down   -> action[0]
SIGN_Y = -1.0  # stick left/right -> action[1]  (inverted)
AXIS_R2 = 5
DEADZONE = 0.05
TRIGGER_FLOOR = 0.02
JOY_TIMEOUT_S = 0.15

PLANNING_GROUP = "fr3_arm"
# fr3_pusher_tcp = [0.40, -0.10, 0.045] in fr3_link0, tool exactly vertical
# (0.00 deg tilt), 23.9 deg of joint-limit margin, 10 mm of sphere clearance
# above the 0.020 table. Chosen 2026-08-20 to put arm_qpos near the
# distribution the policy was trained on, at sim's own pushing height.
#
# arm_qpos/arm_qvel are 14 of the 24 observation dims and the policy is very
# sensitive to them -- it effectively memorised sim's joint-space trajectories.
# Measuring the angle between the policy's action and the direction from the
# pusher to the block, on the real observation:
#     previous home ......... 67.3 deg off
#     sim's QHOME ........... 27.4 deg off   (unsafe, see below)
#     this pose .............. 3.1 deg off
# For comparison, physically repositioning the goal and block to sim's exact
# layout scores 3.7 deg -- so this pose is as good, with nothing moved.
# Magnitude is a misleading metric here (a large action in the wrong direction
# is worse than a small correct one); this was selected on direction.
#
# Note pusht_mjx's QHOME itself is NOT usable on the real robot, for two
# independent reasons: its joint 4 sits 5.5 deg from the limit, inside Servo's
# joint_limit_margin of 0.10 rad (5.7 deg), so Servo would halt with "close to a
# joint bound"; and its tip is at z=0.03, at or below the real table surface,
# because sim's ground plane is at z=0 while this table sits 0.020 above the
# robot base (measured). QHOME is only a null-space attractor in sim anyway -- the arm
# never actually rests there, it balances against the Z_HOLD servo.
#
# Joint 7 matters far more than its physical effect suggests: it only spins the
# round, coaxial pusher about its own axis (/compute_fk gives an identical tip
# position for wildly different values), yet on a real observation swapping only
# joint 7 to sim's value moved the action from 0.153 to 0.675. It stays put
# during an episode because commanding zero angular velocity makes Servo hold
# orientation, which pins it.
HOME_JOINTS = {
    "fr3_joint1": 0.342282,
    "fr3_joint2": 0.240131,
    "fr3_joint3": -0.502155,
    "fr3_joint4": -2.659841,
    "fr3_joint5": 0.407356,
    "fr3_joint6": 2.848474,
    "fr3_joint7": -2.110221,
}
JOINT_TOL = 0.01
VEL_SCALE = 0.2
ACC_SCALE = 0.2
MOVE_ACTION = "/move_action"
# This build of moveit_servo has no start_servo/stop_servo Trigger services --
# only a single pause_servo SetBool (true = pause, false = resume).
PAUSE_SERVO = "/servo_node/pause_servo"
START_COLLECT_SRV = "/pusht/start_episode"
STOP_COLLECT_SRV = "/pusht/finish_episode"
SERVO_SETTLE_S = 0.2
# Servo ignores Cartesian twist commands (silently -- no error/warning) until
# switched into this command mode. It resets to some other mode every time
# servo_node restarts, so this must be re-sent on every teleop-node startup.
SWITCH_COMMAND_TYPE_SRV = "/servo_node/switch_command_type"
SERVO_COMMAND_TYPE_TWIST = 1


class PushTTeleop(Node):
    def __init__(self):
        super().__init__("pusht_ps5_teleop")
        self._cbg = ReentrantCallbackGroup()
        self._joy = None
        self._joy_time = None
        self._prev = {}
        self._busy = False
        self._action_pub = self.create_publisher(Float32MultiArray, ACTION_TOPIC, 10)
        self._deadman_pub = self.create_publisher(Bool, DEADMAN_TOPIC, 10)
        self.create_subscription(Joy, JOY_TOPIC, self._on_joy, 10,
                                 callback_group=self._cbg)
        self.create_timer(1.0 / PUBLISH_RATE, self._on_timer,
                          callback_group=self._cbg)
        self._servo_pause = self.create_client(SetBool, PAUSE_SERVO, callback_group=self._cbg)
        self._collect_start = self.create_client(Trigger, START_COLLECT_SRV, callback_group=self._cbg)
        self._collect_stop = self.create_client(Trigger, STOP_COLLECT_SRV, callback_group=self._cbg)
        self._move = (ActionClient(self, MoveGroup, MOVE_ACTION, callback_group=self._cbg)
                      if MoveGroup is not None else None)
        if self._move is None:
            self.get_logger().warning("moveit_msgs unavailable: Home button is disabled")
        self._switch_command_type = (
            self.create_client(ServoCommandType, SWITCH_COMMAND_TYPE_SRV, callback_group=self._cbg)
            if ServoCommandType is not None else None)
        if self._switch_command_type is not None:
            threading.Thread(target=self._enable_cartesian_servo, daemon=True).start()
        else:
            self.get_logger().warning("moveit_msgs unavailable: cannot auto-enable Cartesian Servo")

    def _enable_cartesian_servo(self):
        result = self._call_sync(self._switch_command_type,
                                 ServoCommandType.Request(command_type=SERVO_COMMAND_TYPE_TWIST))
        if result and result.success:
            self.get_logger().info("Servo switched to Cartesian twist command mode")
        else:
            self.get_logger().error(
                "Failed to switch Servo to Cartesian twist command mode -- "
                "teleop/policy motion will silently do nothing until this succeeds")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joy(self, msg):
        self._joy = msg
        self._joy_time = self._now()
        self._edge(msg, HOME_BUTTON, self._on_home)
        self._edge(msg, START_COLLECT_BUTTON, self._on_start_collect)
        self._edge(msg, STOP_COLLECT_BUTTON, self._on_stop_collect)

    def _edge(self, msg, index, callback):
        if index >= len(msg.buttons):
            return
        pressed = bool(msg.buttons[index])
        if pressed and not self._prev.get(index, False):
            callback()
        self._prev[index] = pressed

    def _on_timer(self):
        joy = self._joy
        fresh = joy is not None and self._joy_time is not None and self._now() - self._joy_time <= JOY_TIMEOUT_S
        enabled = fresh and not self._busy and ENABLE_BUTTON < len(joy.buttons) and bool(joy.buttons[ENABLE_BUTTON])
        action = [0.0, 0.0]
        if enabled and max(AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_R2) < len(joy.axes):
            sx = SIGN_X * float(joy.axes[AXIS_LEFT_Y])
            sy = SIGN_Y * float(joy.axes[AXIS_LEFT_X])
            norm = math.hypot(sx, sy)
            trigger = max(0.0, min(1.0, (1.0 - joy.axes[AXIS_R2]) / 2.0))
            if norm > DEADZONE and trigger > TRIGGER_FLOOR:
                action = [trigger * sx / norm, trigger * sy / norm]
        self._deadman_pub.publish(Bool(data=enabled))
        self._action_pub.publish(Float32MultiArray(data=action))

    def _on_home(self):
        if self._move is None:
            self.get_logger().error("Cannot home: moveit_msgs is not installed")
            return
        if self._busy:
            return
        threading.Thread(target=self._home_sequence, daemon=True).start()

    def _home_sequence(self):
        self._busy = True
        try:
            self._call_sync(self._collect_stop)
            ok = self._home_and_settle()
            self.get_logger().info("Home reached" if ok else "Home failed")
        finally:
            self._busy = False

    def _home_and_settle(self):
        """Pause Servo, drive to HOME_JOINTS, then resume Servo. Returns success."""
        self._publish_stop_action()
        paused = self._call_sync(self._servo_pause, SetBool.Request(data=True))
        if not (paused and paused.success):
            self.get_logger().error("Failed to pause Servo; not homing (Servo would fight MoveGroup's execution)")
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
        # Home before recording so every episode starts from the same fixed
        # pose (mirrors the sim env's reset()) and the transit motion itself
        # is never part of the recorded trajectory.
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
        # Finish the episode first, then home -- the return-to-home motion
        # itself must not be recorded as part of the episode either.
        self._call_sync(self._collect_stop)
        self._busy = True
        try:
            ok = self._home_and_settle()
            self.get_logger().info("Home reached after episode" if ok else "Home failed after episode")
        finally:
            self._busy = False

    def _publish_stop_action(self):
        self._deadman_pub.publish(Bool(data=False))
        self._action_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

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

    def _call_async(self, client):
        if not client.service_is_ready():
            self.get_logger().warning(f"{client.srv_name} unavailable")
            return
        client.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)
    node = PushTTeleop()
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
