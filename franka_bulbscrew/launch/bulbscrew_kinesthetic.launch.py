import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Hand-guided demonstration collection.

    Deliberately omits safety_node, servo_ik_node and ps5_teleop_node: nothing
    should command the arm while it is being moved by hand. The gamepad is still
    launched, but only gripper_node consumes it -- the gripper is the one thing
    the operator cannot do by hand while holding the robot.
    """
    share = get_package_share_directory("franka_bulbscrew")
    parameters = [os.path.join(share, "config", "bulbscrew_common.yaml"),
                  os.path.join(share, "config", "bulbscrew_robot.yaml")]
    return LaunchDescription([
        Node(package="franka_bulbscrew", executable="optitrack_bridge_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbscrew", executable="observation_node",
             parameters=parameters, output="screen"),
        Node(package="joy", executable="joy_node", output="screen"),
        Node(package="franka_bulbscrew", executable="gripper_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbscrew", executable="collision_behavior_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbscrew", executable="kinesthetic_recorder_node",
             parameters=parameters, output="screen"),
    ])
