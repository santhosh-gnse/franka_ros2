import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("franka_pusht")
    parameters = [os.path.join(share, "config", "pusht_common.yaml"),
                  os.path.join(share, "config", "pusht_robot.yaml")]
    return LaunchDescription([
        Node(package="franka_pusht", executable="optitrack_bridge_node", parameters=parameters, output="screen"),
        Node(package="franka_pusht", executable="observation_node", parameters=parameters, output="screen"),
        Node(package="joy", executable="joy_node", output="screen"),
        Node(package="franka_pusht", executable="ps5_teleop_node", parameters=parameters + [{"set_servo_command_type": False}], output="screen"),
        Node(package="franka_pusht", executable="safety_node", parameters=parameters, output="screen"),
        Node(package="franka_pusht", executable="servo_ik_node", parameters=parameters, output="screen"),
        Node(package="franka_pusht", executable="data_collection_node", parameters=parameters, output="screen"),
    ])
