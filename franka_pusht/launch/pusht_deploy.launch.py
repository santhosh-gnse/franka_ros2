import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("franka_pusht")
    common = os.path.join(share, "config", "pusht_common.yaml")
    robot = os.path.join(share, "config", "pusht_robot.yaml")
    parameters = [common, robot]
    return LaunchDescription([
        Node(package="franka_pusht", executable="optitrack_bridge_node", parameters=parameters, output="screen"),
        Node(package="franka_pusht", executable="observation_node", parameters=parameters, output="screen"),
        Node(package="franka_pusht", executable="policy_node", parameters=parameters, output="screen"),
        Node(package="franka_pusht", executable="safety_node", parameters=parameters + [{"command_source": "policy"}], output="screen"),
    ])

