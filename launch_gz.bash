#!/bin/bash

clear

# Build ROS Nodes & Terminate if there are errors
if ! colcon build --symlink-install; then
    echo "Error: Build failed. Exiting..."
    exit 1
fi

# Source ROS 2 setup
source install/setup.bash

# Launch Gazebo using ROS 2 launch
ros2 launch robot_spawn_pkg spawn_ttbot_pose.launch.py &

# Launch RQT_Network to visualize all nodes, services and topics
ros2 run ros_network_viz ros_network_viz