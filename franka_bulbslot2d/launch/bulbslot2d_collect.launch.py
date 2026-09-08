import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("franka_bulbslot2d")
    parameters = [os.path.join(share, "config", "bulbslot2d_common.yaml"),
                  os.path.join(share, "config", "bulbslot2d_robot.yaml")]
    return LaunchDescription([
        Node(package="franka_bulbslot2d", executable="optitrack_bridge_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbslot2d", executable="observation_node",
             parameters=parameters, output="screen"),
        Node(package="joy", executable="joy_node", output="screen"),
        Node(package="franka_bulbslot2d", executable="ps5_teleop_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbslot2d", executable="safety_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbslot2d", executable="servo_ik_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbslot2d", executable="collision_behavior_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbslot2d", executable="gripper_node",
             parameters=parameters, output="screen"),
        Node(package="franka_bulbslot2d", executable="data_collection_node",
             parameters=parameters, output="screen"),
    ])
