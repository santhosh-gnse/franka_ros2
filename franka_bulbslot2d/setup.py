import os
from glob import glob

from setuptools import find_packages, setup


package_name = "franka_bulbslot2d"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Santhosh G S",
    maintainer_email="santhoshgs013@gmail.com",
    description="FR3 bulb-slotting reduced to a 2-D vertical plane, teleoperated",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "optitrack_bridge_node = franka_bulbslot2d.optitrack_bridge_node:main",
            "observation_node = franka_bulbslot2d.observation_node:main",
            "ps5_teleop_node = franka_bulbslot2d.ps5_teleop_node:main",
            "safety_node = franka_bulbslot2d.safety_node:main",
            "servo_ik_node = franka_bulbslot2d.servo_ik_node:main",
            "gripper_node = franka_bulbslot2d.gripper_node:main",
            "collision_behavior_node = franka_bulbslot2d.collision_behavior_node:main",
            "data_collection_node = franka_bulbslot2d.data_collection_node:main",
        ],
    },
)
