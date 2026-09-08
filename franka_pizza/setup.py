import os
from glob import glob

from setuptools import find_packages, setup


package_name = "franka_pizza"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "NEED_TO_CONFIGURE.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="franka",
    maintainer_email="joao@robot-learning.de",
    description="FR3 pizza sauce spreading: kinesthetic demonstration collection",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "observation_node = franka_pizza.observation_node:main",
            "guiding_mode = franka_pizza.guiding_mode:main",
            "collision_behavior_node = franka_pizza.collision_behavior_node:main",
            "kinesthetic_recorder_node = franka_pizza.kinesthetic_recorder_node:main",
        ],
    },
)
