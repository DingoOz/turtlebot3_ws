from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xbox_turtlebot3_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/test', glob('test/**/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.20.0',  # Specify minimum version
        'rclpy',
    ],
    setup_requires=[
        'numpy>=1.20.0',  # Add setup_requires for numpy
    ],
    zip_safe=True,
    maintainer='dingo',
    maintainer_email='your_email@example.com',
    description='Xbox controller teleop node for TurtleBot3',
    license='Apache License 2.0',
    tests_require=[
        'pytest',
        'pytest-cov',
        'pytest-mock',
        'pytest-asyncio',
        'numpy>=1.20.0',  # Add numpy to test requirements
    ],
    entry_points={
        'console_scripts': [
            'xbox_teleop = xbox_turtlebot3_teleop.xbox_teleop_node:main'
        ],
    },
    python_requires='>=3.8',
)
