import os
from glob import glob

from setuptools import find_packages, setup


package_name = "franka_bulbscrew"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "NEED_TO_CONFIGURE.md",
                                   "SIM_ALIGNMENT.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models"), glob("models/*.npz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Santhosh G S",
    maintainer_email="santhoshgs013@gmail.com",
    description="FR3 bulb-screwing observation, teleoperation, recording, safety, and deployment",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "optitrack_bridge_node = franka_bulbscrew.optitrack_bridge_node:main",
            "observation_node = franka_bulbscrew.observation_node:main",
            "ps5_teleop_node = franka_bulbscrew.ps5_teleop_node:main",
            "safety_node = franka_bulbscrew.safety_node:main",
            "servo_ik_node = franka_bulbscrew.servo_ik_node:main",
            "gripper_node = franka_bulbscrew.gripper_node:main",
            "collision_behavior_node = franka_bulbscrew.collision_behavior_node:main",
            "guiding_mode = franka_bulbscrew.guiding_mode:main",
            "kinesthetic_recorder_node = franka_bulbscrew.kinesthetic_recorder_node:main",
            "data_collection_node = franka_bulbscrew.data_collection_node:main",
            "policy_node = franka_bulbscrew.policy_node:main",
            "extract_success_trajectories = franka_bulbscrew.extract_success_trajectories:main",
        ],
    },
)
