# Custom TurtleBot3 with SLAMTEC Lidar

This workspace contains a custom setup for TurtleBot3 with SLAMTEC Lidar.

## Setup
0. sudo apt install libv4l-dev ros-humble-cv-bridge
1. Clone this repository   
2. Source ROS setup file: `source /opt/ros/humble/setup.bash`
3. Build the workspace: `colcon build --symlink-install`


To clone this repository with all its submodules, use:

git clone --recursive https://github.com/yourusername/your-repo-name.git

# Building

Make sure that you do not have setuptools installed on your machine, or this will fail to build
uninstall via:
```
pip3 uninstall setuptools
```

canera requires: sudo apt-get install libv4l-dev

wifi_shutdown_monitor requires: wireless-tools

slamtool_box is required: sudo apt install ros-humble-slam-toolbox

sudo apt install ros-humble-nav2-bringup

pip3 install depthai

sudo apt install ros-humble-depthai-ros

<<<<<<< HEAD
##Setting up the systemd service
There is a bash script in the root directory called "start_turtblebot3.sh". If you would like to 
create a systemd service from this file that will automatically run on startup you can follow these 
steps. This will result in the robot being ready as soon as it is booted and without and need to ssh 
into it or similar.

1 - Edit the start_turtlebot3.sh file and change the username and directories if needed
2 - Make sure the script is executable
3 - Create a service file:
	3.1 - sudo nano /etc/systemd/system/turtlebot3.service
	3.2 - add the following

```
[Unit]
Description=TurtleBot3 Bringup Service
After=network.target

[Service]
Type=simple
User=username
WorkingDirectory=/home/username
ExecStart=/home/ubuntu/turtblebot3_ws/start_turtlebot3.sh
Restart=on-failure
Environment=ROS_DOMAIN_ID=30  # Optional, depending on your setup

[Install]
WantedBy=multi-user.target
```

4 - Reload the daemon
```
sudo systemctl daemon-reload
```

5 - Enable the service to start on boot
```
sudo systemctl enable turtlebot3.service
```

6 - Start the service (or reboot)
```
sudo systemctl start turtlebot3.service
```



=======
sudo apt install ros-humble-nav2-bringup
>>>>>>> 11fc59686f1ca332a4f79d2f55ec442ae1d5b980
## Usage
[Add usage instructions here]

## Updating TurtleBot3 Fork
To update your TurtleBot3 fork with the latest changes from the original repository:

1. Add the original repo as a remote:
   `git remote add upstream https://github.com/ROBOTIS-GIT/turtlebot3.git`
2. Fetch the updates:
   `git fetch upstream`
3. Merge the updates into your ros2 branch:
   `git checkout ros2`
   `git merge upstream/ros2`
4. Push the updates to your fork:
   `git push origin ros2`

