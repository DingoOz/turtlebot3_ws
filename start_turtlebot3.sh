#! /bin/bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/turtlebot3_ws/install/setup.bash

#Launch the Turtlebot3 bringup
exec ros2 launch turtlebot3_bringup dingo.robot.launch.py  >> /home/ubuntu/turtlebot3_ws/launch.log 2>&1

StandardOutput=journal
StandardErr=journal
