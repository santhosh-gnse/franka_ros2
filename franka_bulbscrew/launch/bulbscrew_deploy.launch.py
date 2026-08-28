import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("franka_bulbscrew")
    parameters = [os.path.join(share, "config", "bulbscrew_common.yaml"),
                  os.path.join(share, "config", "bulbscrew_robot.yaml")]
    return LaunchDescription([
        Node(package="franka_bulbscrew", executable="optitrack_bridge_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbscrew", executable="observation_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbscrew", executable="policy_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbscrew", executable="safety_node",
             parameters=parameters + [{"command_source": "policy"}], output="screen"),
        Node(package="franka_bulbscrew", executable="servo_ik_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbscrew", executable="gripper_node",
             parameters=parameters, output="screen"),
    ])
