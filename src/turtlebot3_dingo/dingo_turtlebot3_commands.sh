#!/bin/bash


# Define the commands
commands=(
    "export ROS_DOMAIN_ID=30"
    "export TURTLEBOT3_MODEL=burger"
    "ros2 launch turtlebot3_bringup dingo.robot.launch.py use_sim_time:=False"
    "ros2 run turtlebot3_teleop teleop_keyboard"
    "ros2 launch turtlebot3_cartographer cartographer.launch.py"
    "ros2 run nav2_map_server map_saver_cli -f ~/map"
    "ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml"
    "ros2 launch nav2_bringup navigation_launch.py"
    "ros2 launch slam_toolbox online_async_launch.py"
    "watch ros2 topic list"
)

echo "***This script needs to be run as 'source dingo_turtlebots_commands,sh***"

# Display menu and get user choice
echo "Select a command to run:"
for i in "${!commands[@]}"; do
    echo "$((i+1))) ${commands[$i]}"
done

read -p "Enter your choice (1-${#commands[@]}): " choice

if [[ "$choice" -ge 1 && "$choice" -le "${#commands[@]}" ]]; then
    selected_command="${commands[$((choice-1))]}"
    echo "Running command: $selected_command"

    # Check if the selected command is an export
    if [[ $selected_command == export* ]]; then
        eval "$selected_command"
    else
        eval "$selected_command"
    fi
else
    echo "Invalid choice. Exiting."
    exit 1
fi
