#!/bin/bash

# Add ROS 2 Foxy repository
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Foxy and dependencies
sudo apt update
sudo apt install -y ros-foxy-desktop

# Install additional Gazebo-related packages
sudo apt install -y ros-foxy-gazebo-dev ros-foxy-gazebo-msgs ros-foxy-gazebo-plugins ros-foxy-gazebo-ros ros-foxy-gazebo-ros-pkgs

# Source ROS 2 setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Installation completed!"
