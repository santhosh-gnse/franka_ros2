import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Hand-guided demonstration collection.

    No optitrack_bridge_node and no tracked rigid body: unlike PushT and
    BulbScrew, nothing external is sensed at runtime. OptiTrack is used once,
    offline, to measure home_position (see NEED_TO_CONFIGURE.md) and plays no
    part in this pipeline -- observation_node builds the observation from
    /joint_states and forward kinematics alone.

    No gripper_node either: the tool has no actuation.
    """
    share = get_package_share_directory("franka_pizza")
    parameters = [os.path.join(share, "config", "pizza_common.yaml"),
                  os.path.join(share, "config", "pizza_robot.yaml")]
    return LaunchDescription([
        Node(package="franka_pizza", executable="observation_node",
             parameters=parameters, output="screen"),
        Node(package="joy", executable="joy_node", output="screen"),
        Node(package="franka_pizza", executable="collision_behavior_node",
             parameters=parameters, output="screen"),
        Node(package="franka_pizza", executable="kinesthetic_recorder_node",
             parameters=parameters, output="screen"),
    ])
