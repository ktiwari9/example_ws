#!/bin/bash

# Function to check if a package is installed
is_package_installed() {
    dpkg -s "$1" &> /dev/null
}

# Add ROS 2 Foxy repository
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Foxy and dependencies
sudo apt update
if ! is_package_installed "ros-foxy-desktop"; then
    sudo apt install -y ros-foxy-desktop
fi

# Install additional Gazebo-related packages
packages=("ros-foxy-gazebo-dev" "ros-foxy-gazebo-msgs" "ros-foxy-gazebo-plugins" "ros-foxy-gazebo-ros" "ros-foxy-gazebo-ros-pkgs")
for package in "${packages[@]}"; do
    if ! is_package_installed "$package"; then
        sudo apt install -y "$package"
    fi
done

# Install Networkx and pip
if ! is_package_installed "python3-pip"; then
    sudo apt-get -y install python3-pip
fi
pip3 install PyQt5 networkx

# Check and install scipy
if ! is_package_installed "python3-scipy"; then
    sudo apt-get -y install python3-scipy
fi

# Check and install TF library
packages=("ros-foxy-turtle-tf2-py" "ros-foxy-tf2-tools" "ros-foxy-tf-transformations")
for package in "${packages[@]}"; do
    if ! is_package_installed "$package"; then
        sudo apt install -y "$package"
    fi
sudo pip3 install transforms3d # installation by hand required
done

# Source ROS 2 setup.bash
if ! grep -q "source /opt/ros/foxy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    source ~/.bashrc
fi

echo "Installation completed!"
