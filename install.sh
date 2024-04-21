#!/usr/bin/env sh

# Update package lists
sudo apt-get update && sudo apt-get upgrade -y

# Install presiqisite for ubuntu
sudo apt-get install -y software-properties-common curl python-is-python3

# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo add-apt-repository ppa:ubuntu-toolchain-r/test

sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo add-apt-repository universe

# Update package lists
sudo apt-get update

# Install python3.8
sudo apt-get install -y python3.8-dev python3.8-venv
sudo update-alternatives --remove-all python3
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 100

# Install ROS1
sudo apt-get install -y ros-noetic-desktop-full
sudo apt-get install -y ros-noetic-xacro ros-noetic-rviz ros-noetic-joint-trajectory-controller ros-noetic-rqt-robot-steering ros-noetic-rqt-joint-trajectory-controller ros-noetic-joint-state-controller ros-noetic-joint-state-publisher-gui ros-noetic-effort-controllers ros-noetic-gripper-action-controller ros-noetic-dwa-local-planner ros-noetic-cob-gazebo-objects ros-noetic-map-server ros-noetic-move-base
sudo apt-get install -y python3-catkin-tools

# Install ROS2
sudo apt-get install -y ros-foxy-desktop
sudo apt-get install -y ros-dev-tools

# Install python3-pip
sudo apt-get install -y python3-pip

# Install rosdep
sudo apt-get install -y python3-rosdep
sudo rosdep init
sudo rosdep fix-permissions
rosdep update

# Install glfw3
sudo apt-get install -y libglfw3
sudo apt-get install -y libglfw3-dev

# Install jsoncpp
sudo apt-get install -y libjsoncpp-dev

# Install zmqpp
sudo apt-get install -y libzmqpp-dev

# Install additional packages for USD
sudo apt-get install -y libxcb-cursor0

# Install additional packages for MuJoCo
sudo apt-get install -y libgl1-mesa-dev libglu1-mesa-dev libxt-dev

# Install and link clang-11 for creating shared library
sudo apt-get install -y clang-11 libc++-11-dev libstdc++-11-dev libc++abi-11-dev llvm-11-dev
sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/x86_64-linux-gnu/libstdc++.so
sudo update-alternatives --remove-all clang++
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang-11 100

# Install and link gcc-11
sudo apt-get install -y gcc-11
sudo update-alternatives --remove-all gcc
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100

# Install additional packages for blender
sudo apt-get install -y build-essential git subversion cmake libx11-dev libxxf86vm-dev libxcursor-dev libxi-dev libxrandr-dev libxinerama-dev libegl-dev
sudo apt-get install -y libwayland-dev wayland-protocols libxkbcommon-dev libdbus-1-dev linux-libc-dev libepoxy-dev

# Install pybind11
sudo apt-get install -y pybind11-dev

# Install jupyter-notebook
sudo apt-get install -y jupyter-notebook

# Upgrade pip
pip install --upgrade pip build

# Install additional packages for USD and multiverse_knowledge
pip install pyside6 pyopengl wheel cython owlready2 markupsafe==2.0.1 jinja2 pybind11

# Install additional packages for multiverse_parser
pip install urdf_parser_py

# Install MuJoCo
pip install mujoco==3.1.4

# Install additional packages for Jupyter Notebook
pip install panel jupyter-server bash_kernel
python3 -m bash_kernel.install