#!/usr/bin/env sh

# Update package lists
apt update

# Install python3-pip
apt install -y python3-pip

# Install python3-dev
apt install -y python3-dev

# Install python3-catkin-tools
apt install -y python3-catkin-tools

# Install libglfw3
apt install -y libglfw3

# Install libglfw3-dev
apt install -y libglfw3-dev

# Install doxygen
apt install -y doxygen

# Install git
apt install -y git

# Install additional packages
apt install -y libgl1-mesa-dev
apt install -y libglu1-mesa-dev
apt install -y libxt-dev