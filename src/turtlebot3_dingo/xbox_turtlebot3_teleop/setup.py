import os
from glob import glob

from setuptools import setup

package_name = "xbox_turtlebot3_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@example.com",
    description="Xbox controller teleop node for TurtleBot3",
    license="Apache License 2.0",
    tests_require=["pytest"],
    test_suite="test",
    entry_points={
        "console_scripts": [
            "xbox_teleop = xbox_turtlebot3_teleop.xbox_teleop_node:main"
        ],
    },
)
