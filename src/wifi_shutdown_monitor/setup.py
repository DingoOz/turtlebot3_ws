import os
from glob import glob

from setuptools import setup

package_name = "wifi_shutdown_monitor"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@example.com",
    description="WiFi monitor for TurtleBot3 shutdown",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wifi_monitor = wifi_shutdown_monitor.wifi_monitor_node:main",
        ],
    },
    python_requires=">=3.6",  # ROS 2 Humble requires Python 3.6 or newer
)
