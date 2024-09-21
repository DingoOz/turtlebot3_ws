# Custom TurtleBot3 with SLAMTEC Lidar

This workspace contains a custom setup for TurtleBot3 with SLAMTEC Lidar, using a forked version of the TurtleBot3 repository.

## Setup
1. Clone this repository
2. Build the workspace: `colcon build --symlink-install`
3. Source the setup file: `source install/setup.bash`

To clone this repository with all its submodules, use:

git clone --recursive https://github.com/yourusername/your-repo-name.git


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

