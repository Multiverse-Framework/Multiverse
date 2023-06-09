#!/usr/bin/env sh

# Update package lists
apt update

# Setup your sources.list
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package lists
apt update

# Install ROS
apt install ros-noetic-desktop-full

# Install python3-pip
apt install -y python3-pip

# Install python3-dev
apt install -y python3-dev

# Install python3-catkin-tools
apt install -y python3-catkin-tools

# Install python3-rosdep
apt install -y python3-rosdep

# Install libglfw3
apt install -y libglfw3

# Install libglfw3-dev
apt install -y libglfw3-dev

# Install doxygen
apt install -y doxygen

# Install additional packages
apt install -y libgl1-mesa-dev
apt install -y libglu1-mesa-dev
apt install -y libxt-dev

# Install PySide6
pip3 install pyside6

# Install PyOpenGL
pip3 install pyopengl

# Install MuJoCo
pip3 install mujoco-py

# Install wheel
pip3 install wheel

# Install Cython
pip3 install cython

# Install owlready2
pip3 install owlready2