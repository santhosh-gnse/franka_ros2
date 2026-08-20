import os
from glob import glob

from setuptools import find_packages, setup


package_name = "franka_pusht"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "NEED_TO_CONFIGURE.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models"), glob("models/*.npz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="franka",
    maintainer_email="joao@robot-learning.de",
    description="FR3 PushT observation, teleoperation, recording, safety, and deployment",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "observation_node = franka_pusht.observation_node:main",
            "teleop_node = franka_pusht.teleop_node:main",
            "ps5_teleop_node = franka_pusht.ps5_teleop_node:main",
            "safety_node = franka_pusht.safety_node:main",
            "servo_ik_node = franka_pusht.servo_ik_node:main",
            "data_collection_node = franka_pusht.data_collection_node:main",
            "policy_node = franka_pusht.policy_node:main",
            "mock_optitrack_node = franka_pusht.mock_optitrack_node:main",
            "mock_joint_state_node = franka_pusht.mock_joint_state_node:main",
            "optitrack_bridge_node = franka_pusht.optitrack_bridge_node:main",
        ],
    },
)
