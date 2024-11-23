from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xbox_turtlebot3_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package XML
        ('share/' + package_name, ['package.xml']),
        # Launch files if you have any
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Test files
        ('share/' + package_name + '/test', glob('test/**/*.py')),
        # Config files if you have any
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
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
    ],
    entry_points={
        'console_scripts': [
            'xbox_teleop = xbox_turtlebot3_teleop.xbox_teleop_node:main'
        ],
    },
    python_requires='>=3.8',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: OS Independent',
    ],
)
