#!/bin/bash

# Build ROS Nodes
colcon build --symlink-install

# Source ROS 2 setup
source install/setup.bash

# Launch Gazebo using ROS 2 launch
ros2 launch robot_spawn_pkg gazebo_launcher.launch.py