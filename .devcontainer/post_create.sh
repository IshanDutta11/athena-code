#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"

sudo rosdep update
sudo rosdep install -r --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y

colcon build --symlink-install
