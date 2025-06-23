# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 Jazzy workspace for a custom TurtleBot3 setup with SLAMTEC RPLidar A1, OAK-D Lite camera, and additional custom packages. The workspace contains both C++ and Python ROS2 packages for robotics applications.

## Essential Commands

### Build and Setup
```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build entire workspace
colcon build --symlink-install

# Build with release optimizations
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace after building
source install/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Testing
```bash
# Run all tests
colcon test --event-handlers console_direct+

# View test results
colcon test-result --verbose

# Run specific package tests
colcon test --packages-select <package_name> --event-handlers console_direct+

# Run Python linting/formatting (if available)
python3 -m flake8 src/
python3 -m black src/
python3 -m isort src/
```

### Development Dependencies
Required system packages that must be installed before building:
```bash
sudo apt-get install libv4l-dev wireless-tools ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-depthai-ros ros-jazzy-cv-bridge

# Additional Qt5 dependencies for webcam viewer
sudo apt-get install libqt5-core libqt5-gui libqt5-widgets

# OCR service dependencies
sudo apt-get install tesseract-ocr tesseract-ocr-eng libtesseract-dev libleptonica-dev libopencv-dev

# Additional ROS2 packages for full functionality
sudo apt-get install ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard
```

Python dependencies:
```bash
pip3 install numpy depthai
```

**Important:** Remove system setuptools if present (`pip3 uninstall setuptools`) as it conflicts with ROS2 build system.

## Architecture Overview

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

**turtlebot3_webcam/**: USB camera integration
- V4L2 camera support with image streaming
- Dependencies: cv_bridge, image_transport, libv4l-dev

**qt5_webcam_viewer/**: Qt5-based camera viewer
- Real-time camera feed display using Qt5 widgets
- Dependencies: Qt5 core, gui, widgets libraries

**ocr_camera_service/**: OCR-based camera capture service
- Service-triggered 5-second video capture with OCR text extraction
- C++ implementation using Tesseract OCR and OpenCV
- Automatic video/image saving with timestamp organization
- Dependencies: libtesseract-dev, libleptonica-dev, libopencv-dev

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

## Common Launch Commands

### Individual Component Testing
```bash
# Test camera only
ros2 launch turtlebot3_webcam camera_viewer.launch.py

# Test OAK-D Lite camera
ros2 launch oakd_lite_turtlebot3 oakd_lite.launch.py

# Test Xbox controller (requires joy node)
ros2 run joy joy_node
ros2 launch xbox_turtlebot3_teleop xbox_teleop.launch.py

# Test SLAM with visualization
ros2 launch slam_toolbox_tb3 slam_and_rviz_launch.py

# Test OCR camera service
ros2 launch ocr_camera_service ocr_camera.launch.py camera_topic:=/webcam/image_raw

# Launch navigation
ros2 launch turtlebot3_navigation2 dingo.navigation2.launch.py map:=$HOME/map.yaml
```

### Full Robot Operations
```bash
# Main robot bringup (all sensors and base)
ros2 launch turtlebot3_bringup dingo.robot.launch.py

# Robot with navigation
ros2 launch oakd_lite_turtlebot3 turtlebot3_oakd_nav2.launch.py

# Teleop control
ros2 run turtlebot3_teleop teleop_keyboard
```

## Development Notes

- All custom launch files set ROS_DOMAIN_ID=30 for consistent networking
- The main robot launch includes WiFi monitoring as a safety feature
- Xbox teleop requires joy node to be running separately
- Camera and LiDAR can be launched independently for testing
- SLAM toolbox is configured specifically for the burger model with RPLidar A1
- Use `dingo_turtlebot3_commands.sh` script for interactive command selection during development