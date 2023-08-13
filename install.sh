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
apt install -y ros-noetic-desktop-full

# Install python3-pip
apt install -y python3-pip

# Install python3.8
update-alternatives --remove-all python3
apt install -y python3.8-dev
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# Install catkin-tools
apt install -y python3-catkin-tools

# Install rosdep
apt install -y python3-rosdep

# Install glfw3
apt install -y libglfw3
apt install -y libglfw3-dev

# Install doxygen
apt install -y doxygen

# Install additional packages for MuJoCo
apt install -y libgl1-mesa-dev
apt install -y libglu1-mesa-dev
apt install -y libxt-dev

# Install packages for creating shared library
apt install -y clang
apt install -y libc++-dev
apt install -y libc++abi-dev

# Install and link gcc-11
update-alternatives --remove-all gcc
apt install gcc-11
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 1

# Install additional packages for blender
apt install -y build-essential git subversion cmake libx11-dev libxxf86vm-dev libxcursor-dev libxi-dev libxrandr-dev libxinerama-dev libegl-dev
apt install -y libwayland-dev wayland-protocols libxkbcommon-dev libdbus-1-dev linux-libc-dev

# Install pybind11
apt install -y pybind11-dev

# Install PySide6
pip3 install pyside6

# Install PyOpenGL
pip3 install pyopengl

# Install wheel
pip3 install wheel

# Install Cython
pip3 install cython

# Install owlready2
pip3 install owlready2

# Install jinja2
pip3 install jinja2

# Install pybind11
pip3 install pybind11