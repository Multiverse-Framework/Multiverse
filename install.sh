#!/usr/bin/env sh

# Update package lists
apt-get update

# Setup your sources.list
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt-get/sources.list.d/ros-latest.list'
apt-get install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-get-key add -

# Update package lists
apt-get update

# Install ROS
apt-get install -y ros-noetic-desktop-full

# Install python3-pip
apt-get install -y python3-pip

# Install python3.8
apt-get install -y python3.8-dev
update-alternatives --remove-all python3
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 100

# Install catkin-tools
apt-get install -y python3-catkin-tools

# Install rosdep
apt-get install -y python3-rosdep
rosdep init

# Install glfw3
apt-get install -y libglfw3
apt-get install -y libglfw3-dev

# Install doxygen
apt-get install -y doxygen

# Install additional packages for MuJoCo
apt-get install -y libgl1-mesa-dev
apt-get install -y libglu1-mesa-dev
apt-get install -y libxt-dev

# Install and link clang-11 for creating shared library
apt-get install -y clang-11
apt-get install -y libc++-11-dev libc++abi-11-dev
update-alternatives --remove-all clang++
update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang-11 100

# Install and link gcc-11
apt-get install gcc-11
update-alternatives --remove-all gcc
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100

# Install additional packages for blender
apt-get install -y build-essential git subversion cmake libx11-dev libxxf86vm-dev libxcursor-dev libxi-dev libxrandr-dev libxinerama-dev libegl-dev
apt-get install -y libwayland-dev wayland-protocols libxkbcommon-dev libdbus-1-dev linux-libc-dev

# Install pybind11
apt-get install -y pybind11-dev

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