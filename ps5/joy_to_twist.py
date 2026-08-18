import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from std_srvs.srv import Trigger, SetBool
from moveit_msgs.srv import ServoCommandType

from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import TwistStamped, Pose
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint, PlanningOptions,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
# --- fill these in ---
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
JOY_TOPIC = "/joy"
FRAME_ID = "fr3_link0"
PUBLISH_RATE = 100.0

# button indices — VERIFY with `ros2 topic echo /joy`
ENABLE_BUTTON = 4              # deadman (L1)
HOME_BUTTON = 10
START_COLLECT_BUTTON = 0
STOP_COLLECT_BUTTON = 1

# axis indices — VERIFY
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_R2 = 5                    # rests at +1, goes to -1 when pulled

DEADZONE = 0.05
MAX_LINEAR = 1.0
SPEED_SCALE = 0.1

PLANNING_GROUP = "fr3_arm"
# HOME_JOINTS = {
#     "fr3_joint1": 0.51199203,
#     "fr3_joint2": 0.1014329,
#     "fr3_joint3": 0.0,
#     "fr3_joint4": -2.356,
#     "fr3_joint5": 0.0,
#     "fr3_joint6": 1.571,
#     "fr3_joint7": 0.785,
# }

HOME_JOINTS = {
    "fr3_joint1": 0.23,
    "fr3_joint2": 0.21,
    "fr3_joint3": -0.21,
    "fr3_joint4": -2.67,
    "fr3_joint5": 0.03,
    "fr3_joint6": 2.84,
    "fr3_joint7": -0.04,
}
JOINT_TOL = 0.01
VEL_SCALE = 0.2               # home move velocity scaling (be conservative on hardware)
ACC_SCALE = 0.2

EE_LINK = "fr3_pusher_tcp"    # matches ee_id:=custom_pusher_ee
POS_TOL = 0.01               # position sphere radius (m)
ORI_TOL = 0.05               # orientation tolerance per axis (rad)
TRIGGER_FLOOR = 0.02   # R2 must be pulled past this before any motion

MOVE_ACTION = "/move_action"
START_SERVO = "/servo_node/start_servo"
STOP_SERVO = "/servo_node/stop_servo"
START_COLLECT_SRV = "/data_collection/start"
STOP_COLLECT_SRV = "/data_collection/stop"

SERVO_SETTLE_S = 0.2


class JoyToTwist(Node):
    def __init__(self):
        super().__init__("joy_to_twist")
        self._cbg = ReentrantCallbackGroup()

        self._joy = None
        self._prev = {}
        self._busy = False
        self._collecting = False

        self.create_subscription(Joy, JOY_TOPIC, self._on_joy, 10,
                                 callback_group=self._cbg)
        self.pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self.create_timer(1.0 / PUBLISH_RATE, self._on_timer,
                          callback_group=self._cbg)

        self._servo_start = self.create_client(Trigger, START_SERVO,
                                               callback_group=self._cbg)
        self._servo_stop = self.create_client(Trigger, STOP_SERVO,
                                              callback_group=self._cbg)
        self._collect_start = self.create_client(Trigger, START_COLLECT_SRV,
                                                 callback_group=self._cbg)
        self._collect_stop = self.create_client(Trigger, STOP_COLLECT_SRV,
                                                callback_group=self._cbg)

        self._move = ActionClient(self, MoveGroup, MOVE_ACTION,
                                  callback_group=self._cbg)
        self.get_logger().info("MoveGroup action client created")

    # ---------- input ----------
    def _on_joy(self, msg):
        self._joy = msg
        self._edge(msg, HOME_BUTTON, self._on_home)
        self._edge(msg, START_COLLECT_BUTTON, self._on_start_collect)
        self._edge(msg, STOP_COLLECT_BUTTON, self._on_stop_collect)

    def _edge(self, msg, idx, cb):
        if idx >= len(msg.buttons):
            return
        pressed = bool(msg.buttons[idx])
        if pressed and not self._prev.get(idx, False):
            cb()
        self._prev[idx] = pressed

    # ---------- home ----------
    def _on_home(self):
        if self._busy:
            self.get_logger().warn("Busy; ignoring home")
            return
        threading.Thread(target=self._home_sequence, daemon=True).start()
    def _home_pose(self):
        pos = (0.4, 0.0, 0.039)
        quat = (0.0, 1.0, 0.0, 0.0)   # [w, x, y, z]
        return pos, quat

    def _home_sequence(self):
        self._busy = True
        try:
            # a) always stop collection (idempotent on data-node side)
            self.get_logger().info("Home: stopping collection")
            self._call_sync(self._collect_stop)
            self._collecting = False

            # b) stop the robot: zero twist, then stop Servo
            self._publish_zero()
            self.get_logger().info("Home: stopping Servo")
            self._call_sync(self._servo_stop)
            time.sleep(SERVO_SETTLE_S)

            # c) plan + execute to home via /move_action
            # self.get_logger().info("Home: sending MoveGroup goal")
            # ok = self._send_home_goal()
            pos, quat = self._home_pose()
            self.get_logger().info(
                f"Home: pose goal x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}")
            ok = self._send_pose_goal(pos, quat)
            self.get_logger().info("Home: reached" if ok else "Home: FAILED")
        finally:
            self._call_sync(self._servo_start)
            self._publish_zero()
            self._busy = False
            self.get_logger().info("Home: done, teleop idle")
    
    def _send_pose_goal(self, pos, quat):
        if not self._move.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{MOVE_ACTION} server unavailable")
            return False
        goal = self._build_pose_goal(pos, quat)
        send_fut = self._move.send_goal_async(goal)
        if not self._await(send_fut, timeout=10.0):
            return False
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("Pose goal rejected")
            return False
        res_fut = gh.get_result_async()
        if not self._await(res_fut, timeout=30.0):
            return False
        result = res_fut.result()
        return bool(result and result.result.error_code.val == 1)

    def _build_pose_goal(self, pos, quat):
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE

        # position: small sphere around target point
        pc = PositionConstraint()
        pc.header.frame_id = FRAME_ID
        pc.link_name = EE_LINK
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [POS_TOL]
        bv = BoundingVolume()
        bv.primitives.append(sphere)
        p = Pose()
        p.position.x = float(pos[0])
        p.position.y = float(pos[1])
        p.position.z = float(pos[2])
        p.orientation.w = 1.0            # region pose; identity is fine
        bv.primitive_poses.append(p)
        pc.constraint_region = bv

        # orientation: quat is [w, x, y, z]; ROS msg is x, y, z, w
        oc = OrientationConstraint()
        oc.header.frame_id = FRAME_ID
        oc.link_name = EE_LINK
        oc.orientation.w = float(quat[0])
        oc.orientation.x = float(quat[1])
        oc.orientation.y = float(quat[2])
        oc.orientation.z = float(quat[3])
        oc.absolute_x_axis_tolerance = ORI_TOL
        oc.absolute_y_axis_tolerance = ORI_TOL
        oc.absolute_z_axis_tolerance = ORI_TOL
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        return goal

    def _build_home_goal(self):
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE

        jcs = []
        for name, pos in HOME_JOINTS.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = JOINT_TOL
            jc.tolerance_below = JOINT_TOL
            jc.weight = 1.0
            jcs.append(jc)
        req.goal_constraints.append(Constraints(joint_constraints=jcs))

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False   # plan AND execute
        return goal

    def _send_home_goal(self):
        if not self._move.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{MOVE_ACTION} server unavailable")
            return False

        send_fut = self._move.send_goal_async(self._build_home_goal())
        if not self._await(send_fut, timeout=10.0):
            return False
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("Home goal rejected")
            return False

        res_fut = gh.get_result_async()
        if not self._await(res_fut, timeout=30.0):
            return False
        result = res_fut.result()
        # error_code.val == 1 is SUCCESS
        return bool(result and result.result.error_code.val == 1)

    def _await(self, fut, timeout):
        """Block worker thread on a future; executor spins it elsewhere."""
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                self.get_logger().error("Future timed out")
                return False
            time.sleep(0.01)
        return True

    # ---------- collection ----------
    def _on_start_collect(self):
        self.get_logger().info("Start collection")
        if self._call_async(self._collect_start):
            self._collecting = True

    def _on_stop_collect(self):
        self.get_logger().info("Stop collection")
        if self._call_async(self._collect_stop):
            self._collecting = False

    # ---------- helpers ----------
    def _publish_zero(self):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = FRAME_ID
        self.pub.publish(ts)

    def _call_sync(self, cli, timeout=2.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"{cli.srv_name} unavailable")
            return None
        fut = cli.call_async(Trigger.Request())
        self._await(fut, timeout=5.0)
        return fut.result()

    def _call_async(self, cli):
        if not cli.service_is_ready():
            self.get_logger().warn(f"{cli.srv_name} not available yet")
            return False
        cli.call_async(Trigger.Request())
        return True

    # ---------- twist ----------
    def _on_timer(self):
        joy = self._joy
        if joy is None or self._busy:
            return
        if ENABLE_BUTTON is not None and (
            ENABLE_BUTTON >= len(joy.buttons) or not joy.buttons[ENABLE_BUTTON]
        ):
            self._publish_zero()
            return

        sx = joy.axes[AXIS_LEFT_Y]
        sy = joy.axes[AXIS_LEFT_X]
        norm = math.hypot(sx, sy)

        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = FRAME_ID
        ts.twist.linear.z = 0.0
        ts.twist.angular.x = 0.0
        ts.twist.angular.y = 0.0
        ts.twist.angular.z = 0.0
        raw = max(0.0, min(1.0, (1.0 - joy.axes[AXIS_R2]) / 2.0))
        if norm > DEADZONE and raw > TRIGGER_FLOOR:
            mag = raw * SPEED_SCALE
            dx, dy = sx / norm, sy / norm
            ts.twist.linear.x = MAX_LINEAR * mag * dx
            ts.twist.linear.y = MAX_LINEAR * mag * dy
        else:
            ts.twist.linear.x = 0.0
            ts.twist.linear.y = 0.0
        self.pub.publish(ts)


def main():
    rclpy.init()
    node = JoyToTwist()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
