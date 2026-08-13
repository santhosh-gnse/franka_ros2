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
except ImportError:  # Allows controller/data-pipeline testing without MoveIt.
    MoveGroup = None
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger


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
AXIS_R2 = 5
DEADZONE = 0.05
TRIGGER_FLOOR = 0.02
JOY_TIMEOUT_S = 0.15

PLANNING_GROUP = "fr3_arm"
HOME_JOINTS = {
    "fr3_joint1": 0.51199203,
    "fr3_joint2": 0.1014329,
    "fr3_joint3": 0.0,
    "fr3_joint4": -2.356,
    "fr3_joint5": 0.0,
    "fr3_joint6": 1.571,
    "fr3_joint7": 0.785,
}
JOINT_TOL = 0.01
VEL_SCALE = 0.2
ACC_SCALE = 0.2
MOVE_ACTION = "/move_action"
START_SERVO = "/servo_node/start_servo"
STOP_SERVO = "/servo_node/stop_servo"
START_COLLECT_SRV = "/pusht/start_episode"
STOP_COLLECT_SRV = "/pusht/finish_episode"
SERVO_SETTLE_S = 0.2


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
        self._servo_start = self.create_client(Trigger, START_SERVO, callback_group=self._cbg)
        self._servo_stop = self.create_client(Trigger, STOP_SERVO, callback_group=self._cbg)
        self._collect_start = self.create_client(Trigger, START_COLLECT_SRV, callback_group=self._cbg)
        self._collect_stop = self.create_client(Trigger, STOP_COLLECT_SRV, callback_group=self._cbg)
        self._move = (ActionClient(self, MoveGroup, MOVE_ACTION, callback_group=self._cbg)
                      if MoveGroup is not None else None)
        if self._move is None:
            self.get_logger().warning("moveit_msgs unavailable: Home button is disabled")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joy(self, msg):
        self._joy = msg
        self._joy_time = self._now()
        self._edge(msg, HOME_BUTTON, self._on_home)
        self._edge(msg, START_COLLECT_BUTTON, lambda: self._call_async(self._collect_start))
        self._edge(msg, STOP_COLLECT_BUTTON, lambda: self._call_async(self._collect_stop))

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
            sx = float(joy.axes[AXIS_LEFT_Y])
            sy = float(joy.axes[AXIS_LEFT_X])
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
            self._publish_stop_action()
            self._call_sync(self._servo_stop)
            time.sleep(SERVO_SETTLE_S)
            ok = self._send_home_goal()
            self.get_logger().info("Home reached" if ok else "Home failed")
        finally:
            self._call_sync(self._servo_start)
            self._publish_stop_action()
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

    def _call_sync(self, client):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(Trigger.Request())
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
