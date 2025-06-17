# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 Humble workspace for a custom TurtleBot3 setup with SLAMTEC RPLidar A1, OAK-D Lite camera, and additional custom packages. The workspace contains both C++ and Python ROS2 packages for robotics applications.

## Commands

### Build and Development
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build entire workspace
colcon build --symlink-install

# Build with release optimizations
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build specific package 
colcon build --packages-select <package_name> --symlink-install

# Source workspace after building
source install/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Testing
```bash
# Run all tests
colcon test --event-handlers console_direct+

# Run tests for specific package
colcon test --packages-select <package_name> --event-handlers console_direct+

# Run Python tests with coverage
python -m pytest test/ --cov --cov-report=term-missing

# View test results
colcon test-result --verbose
```

### Linting and Code Quality
```bash
# C++ packages use ament linting
colcon test --packages-select <cpp_package> --event-handlers console_direct+

# Python packages - run in package directory
python -m pytest --cov --cov-report=term-missing
ruff check .  # if available
```

### Robot Operations
```bash
# Launch robot (main configuration)
ros2 launch turtlebot3_dingo dingo.robot.launch.py

# Launch standard TurtleBot3
ros2 launch turtlebot3_bringup robot.launch.py

# Xbox controller teleoperation
ros2 launch xbox_turtlebot3_teleop xbox_teleop.launch.py

# SLAM with SLAM Toolbox
ros2 launch slam_toolbox_tb3 slam_toolbox_launch.py

# Navigation
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

### Development Dependencies
Required system packages that must be installed before building:
```bash
sudo apt-get install libv4l-dev wireless-tools ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-depthai-ros ros-humble-cv-bridge
```

Python dependencies:
```bash
pip3 install numpy depthai
```

Note: Remove system setuptools if present (`pip3 uninstall setuptools`) as it conflicts with ROS2 build system.

## Architecture

### Core Robot Setup
- **Main Launch**: `turtlebot3_bringup/launch/dingo.robot.launch.py` - Primary robot bringup that launches all essential components
- **LiDAR**: SLAMTEC RPLidar A1 (configured via LDS_MODEL=A1 environment variable)
- **Camera**: OAK-D Lite depth camera via depthai_ros_driver
- **Robot Base**: TurtleBot3 Burger model with OpenCR board on /dev/ttyACM0
- **Domain**: Uses ROS_DOMAIN_ID=30 for network isolation

### Custom Packages

**turtlebot3_dingo/**: Fork of official TurtleBot3 packages with custom modifications
- Modified launch files for RPLidar A1 support
- Custom parameter configurations for burger model
- Xbox controller teleop with deadman switch safety

**dashboard_turtlebot3_cpp/**: Qt5-based C++ dashboard application
- Dependencies: rclcpp, geometry_msgs, Qt5, OpenCV, cv_bridge

**oakd_lite_turtlebot3/**: OAK-D Lite camera integration
- Launches depthai_ros_driver with 1080p RGB, depth, and pointcloud output
- Configured for 30fps with synchronized outputs

**slam_toolbox_tb3/**: SLAM configuration
- Custom slam_toolbox parameters in config/slam_toolbox_params.yaml
- Launch files for SLAM with RViz visualization

**wifi_shutdown_monitor/**: Safety system
- Monitors WiFi connectivity and initiates shutdown on prolonged disconnection
- Automatically included in main robot launch

**xbox_turtlebot3_teleop/**: Xbox controller teleop
- Right trigger deadman switch for safety
- Mathematical velocity calculation with configurable max speeds
- Direction reversal logic for intuitive control
- Python package with numpy dependency

**sllidar_ros2/**: SLAMTEC RPLidar driver supporting A1/A2/A3/C1/S1/S2/S3/T1 models

**opensw_ros/**: Open driver for SlamTec M2M1 Mapper devices

### Environment Variables
- `TURTLEBOT3_MODEL=burger` (default)
- `LDS_MODEL=A1` (for RPLidar A1)
- `ROS_DOMAIN_ID=30` (network isolation)

### Hardware Configuration
- OpenCR board: /dev/ttyACM0
- RPLidar A1: /dev/ttyUSB0
- OAK-D Lite: Requires udev rules for proper permissions (see README.md)

## Testing Strategy

The workspace uses pytest for Python packages and gtest for C++ packages. Tests are automatically run in CI with the following pattern:
- Install numpy dependency explicitly for test environment
- Run `colcon test --event-handlers console_direct+`
- Check results with `colcon test-result --verbose`

## Systemd Service Deployment

The workspace includes `start_turtlebot3.sh` for automatic robot startup. This script:
- Sources ROS2 and workspace environments
- Launches the main robot bringup
- Logs output to `/home/ubuntu/turtlebot3_ws/launch.log`
- Can be deployed as systemd service for boot-time startup

## Important Files
- `dingo.robot.launch.py` - Main robot bringup (use this instead of standard robot.launch.py)
- `start_turtlebot3.sh` - Systemd service script for automatic startup
- `dingo_turtlebot3_commands.sh` - Interactive command menu for operations
- CI configuration in `.github/workflows/ros2-ci.yml`

## Development Notes
- Always source the workspace before running commands: `source install/setup.bash`
- Use `--symlink-install` for faster development iteration
- All custom launch files set ROS_DOMAIN_ID=30 for consistent networking
- The main robot launch includes WiFi monitoring as a safety feature
- Xbox teleop requires joy node to be running separately
- Camera and LiDAR can be launched independently for testing
- SLAM toolbox is configured specifically for the burger model with RPLidar A1
- Python packages require numpy dependency for testing
- LIDAR driver supports multiple SLAMTEC models - check hardware compatibility
- Dashboard application requires Qt5 development libraries
