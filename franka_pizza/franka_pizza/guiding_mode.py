"""Switch the arm between hand-guiding and normal control.

Kinesthetic teaching needs the arm held against gravity but otherwise free, so
it can be pushed around by hand. That is what
GravityCompensationExampleController does -- and crucially it runs with FCI ON,
unlike Desk's guiding mode, where the controllers go inactive and a fallback
joint_state_publisher serves frozen defaults, making it useless for recording.

Height and orientation are NOT locked here, even though the task ultimately
requires both fixed -- see kinesthetic_recorder_node.py's module docstring for
why: an earlier attempt at actively locking them during teaching (via
CartesianImpedanceExampleController, x/y soft, z/roll/pitch/yaw stiff) hit
sustained oscillation the moment the tool touched the board, a rigid-on-rigid
contact instability, not a tuning mistake. The operator's own hand keeps
height/orientation close enough during a real demonstration; enforcing the
lock belongs on a deployed policy's actions (a future safety_node/servo_ik_node,
not built yet), not on a human's.

The gravity controller and fr3_arm_controller both claim the arm's effort
interfaces, so exactly one may be active. This flips between them in one call.

    ros2 run franka_pizza guiding_mode --ros-args -p enable:=true    # guide
    ros2 run franka_pizza guiding_mode --ros-args -p enable:=false   # normal

Servo is paused while guiding, so it cannot fight the hand, and resumed on the
way back. Homing and teleop only work in normal mode.
"""

import sys

import rclpy
from controller_manager_msgs.srv import (ConfigureController, ListControllers,
                                         LoadController, SwitchController)
from rclpy.node import Node
from std_srvs.srv import SetBool

ARM = "fr3_arm_controller"
GRAVITY = "gravity_compensation_example_controller"


class GuidingMode(Node):
    def __init__(self):
        super().__init__("pizza_guiding_mode")
        self.declare_parameter("enable", True)
        self.declare_parameter("switch_service", "/controller_manager/switch_controller")
        self.declare_parameter("pause_servo_service", "/servo_node/pause_servo")
        self.declare_parameter("list_service", "/controller_manager/list_controllers")
        self.declare_parameter("load_service", "/controller_manager/load_controller")
        self.declare_parameter("configure_service", "/controller_manager/configure_controller")

    def _call(self, cls, service, request, timeout=10.0):
        client = self.create_client(cls, service)
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def _ensure_loaded(self):
        """Load and configure the gravity controller if it is not ready.

        Declaring it in fr3_ros_controllers.yaml only makes the type spawnable;
        controller_manager still has to load and configure it before it can be
        activated. Doing that here means guiding works on a fresh stack instead
        of failing with an unhelpful "switch failed".
        """
        listing = self._call(ListControllers, self.get_parameter("list_service").value,
                             ListControllers.Request())
        if listing is None:
            self.get_logger().error("controller_manager list service unavailable")
            return False
        state = next((c.state for c in listing.controller if c.name == GRAVITY), None)
        if state is None:
            self.get_logger().info(f"loading {GRAVITY}")
            r = self._call(LoadController, self.get_parameter("load_service").value,
                           LoadController.Request(name=GRAVITY))
            if r is None or not r.ok:
                self.get_logger().error(f"could not load {GRAVITY}")
                return False
            state = "unconfigured"
        if state == "unconfigured":
            self.get_logger().info(f"configuring {GRAVITY}")
            r = self._call(ConfigureController, self.get_parameter("configure_service").value,
                           ConfigureController.Request(name=GRAVITY))
            if r is None or not r.ok:
                self.get_logger().error(f"could not configure {GRAVITY}")
                return False
        return True

    def run(self):
        enable = bool(self.get_parameter("enable").value)
        if enable and not self._ensure_loaded():
            return 1
        # Pause Servo first when entering guiding: a live Servo would keep
        # streaming commands and fight the hand.
        self._pause_servo(enable)
        client = self.create_client(SwitchController,
                                    self.get_parameter("switch_service").value)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("controller_manager switch service unavailable")
            return 1
        request = SwitchController.Request()
        request.activate_controllers = [GRAVITY] if enable else [ARM]
        request.deactivate_controllers = [ARM] if enable else [GRAVITY]
        # STRICT: refuse rather than half-switch, which would leave the arm with
        # no controller holding it.
        request.strictness = SwitchController.Request.STRICT
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not result.ok:
            self.get_logger().error(
                f"switch failed; the arm is still under {ARM if enable else GRAVITY}")
            return 1
        if not enable:
            self._pause_servo(False)
        self.get_logger().info(
            "GUIDING MODE: the arm is free to move by hand. Height/orientation are NOT "
            "locked (see module docstring). Recording still works (FCI is on). Run with "
            "enable:=false before homing or teleop."
            if enable else
            "NORMAL MODE: fr3_arm_controller active, Servo resumed.")
        return 0

    def _pause_servo(self, pause):
        client = self.create_client(SetBool, self.get_parameter("pause_servo_service").value)
        if not client.wait_for_service(timeout_sec=2.0):
            return
        future = client.call_async(SetBool.Request(data=pause))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = GuidingMode()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)
