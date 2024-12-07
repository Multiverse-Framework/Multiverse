#!/bin/bash

# Update package lists
# sudo apt-get update && sudo apt-get upgrade -y

# Install presiqisite for ubuntu
sudo apt-get install -y software-properties-common curl python-is-python3

# Install python3-pip
sudo apt-get install -y python3-pip

UBUNTU_VERSION=$(lsb_release -rs)

if [ $UBUNTU_VERSION = "20.04" ]; then
    # Setup your sources.list
    sudo apt-get install software-properties-common
    sudo add-apt-repository ppa:deadsnakes/ppa

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test

    sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    sudo add-apt-repository universe

    # Update package lists
    sudo apt-get update && sudo apt-get upgrade -y

    # Install python3.10
    sudo apt-get install -y python3.10-dev python3.10-venv

    # Install ROS1
    sudo apt-get install -y ros-noetic-desktop-full
    sudo apt-get install -y ros-noetic-xacro ros-noetic-rviz ros-noetic-joint-trajectory-controller ros-noetic-rqt-robot-steering ros-noetic-rqt-joint-trajectory-controller ros-noetic-joint-state-controller ros-noetic-joint-state-publisher-gui ros-noetic-effort-controllers ros-noetic-gripper-action-controller ros-noetic-dwa-local-planner ros-noetic-cob-gazebo-objects ros-noetic-map-server ros-noetic-move-base
    sudo apt-get install -y python3-catkin-tools

    # Install ROS2
    sudo apt-get install -y ros-foxy-desktop
    sudo apt-get install -y ros-foxy-joint-state-publisher-gui
    sudo apt-get install -y ros-dev-tools

    # Install rosdep
    sudo apt-get install -y python3-rosdep
    sudo rosdep init
    sudo rosdep fix-permissions
    rosdep update
elif [ $UBUNTU_VERSION = "24.04" ]; then
    # Setup your sources.list
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Update package lists
    sudo apt-get update && sudo apt-get upgrade -y
    
    # Install ROS2
    sudo apt-get install -y ros-jazzy-desktop
    sudo apt-get install -y ros-dev-tools

    # Install rosdep
    sudo apt-get install -y python3-rosdep
    sudo rosdep init
    sudo rosdep fix-permissions
    rosdep update
fi

# Install glfw3
sudo apt-get install -y libglfw3
sudo apt-get install -y libglfw3-dev

# Install jsoncpp
sudo apt-get install -y libjsoncpp-dev

# Install zmqpp
sudo apt-get install -y libzmq3-dev

# Install boost
sudo apt-get install -y libboost-dev libboost-filesystem-dev

# Install tinyxml2
sudo apt-get install -y libtinyxml2-dev

# Install additional packages for USD
sudo apt-get install -y libxcb-cursor0

# Install additional packages for MuJoCo
sudo apt-get install -y libgl1-mesa-dev libglu1-mesa-dev libxt-dev

if [ $UBUNTU_VERSION = "20.04" ]; then
    # Install and link clang-11 for creating shared library
    sudo apt-get install -y clang-11 llvm-11-dev libc++-11-dev libc++abi-11-dev libstdc++-11-dev 
    sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/x86_64-linux-gnu/libstdc++.so
    sudo update-alternatives --remove-all clang++
    sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang-11 100

    # Install and link gcc-11
    sudo apt-get install -y gcc-11
    sudo update-alternatives --remove-all gcc
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100

    # Install and link gcc+11
    sudo apt-get install -y g++-11
    sudo update-alternatives --remove-all g++
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100

    # Upgrade pip
    pip install -U pip

    # Setup virtual environment
    pip install virtualenvwrapper
elif [ $UBUNTU_VERSION = "24.04" ]; then
    # Install and link clang-17 for creating shared library
    sudo apt-get install -y clang-17 llvm-17-dev libc++-17-dev libc++abi-17-dev libstdc++-14-dev 
    sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/x86_64-linux-gnu/libstdc++.so
    sudo update-alternatives --remove-all clang++
    sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang-17 100

    # Setup virtual environment
    python3 -m pip install virtualenvwrapper --break-system-packages
fi

# Install additional packages for blender
sudo apt-get install -y build-essential git git-lfs subversion cmake libx11-dev libxxf86vm-dev libxcursor-dev libxi-dev libxrandr-dev libxinerama-dev libegl-dev
sudo apt-get install -y libwayland-dev wayland-protocols libxkbcommon-dev libdbus-1-dev linux-libc-dev

# Install pybind11
sudo apt-get install -y pybind11-dev

# Install jupyter-notebook
sudo apt-get install -y jupyter-notebook

# Setup virtual environment
for virtualenvwrapper in $(which virtualenvwrapper.sh) /usr/share/virtualenvwrapper/virtualenvwrapper.sh /usr/local/bin/virtualenvwrapper.sh /home/$USER/.local/bin/virtualenvwrapper.sh; do
    if [ -f $virtualenvwrapper ]; then
        . $virtualenvwrapper
        mkvirtualenv --system-site-packages multiverse -p python3.10

        # Instlal build
        pip install -U pip build

        # Install additional packages for USD and multiverse_knowledge
        pip install pyside6 pyopengl wheel cython owlready2 markupsafe==2.0.1 jinja2 pybind11 inflection

        # Install additional packages for multiverse_parser
        pip install urdf_parser_py

        # Install MuJoCo
        pip install mujoco==3.2.6

        # Install additional packages for Jupyter Notebook
        pip install panel jupyter-server bash_kernel
        python3 -m bash_kernel.install
        break
    fi
done
if [ ! -f $virtualenvwrapper ]; then
    echo "virtualenvwrapper.sh not found"
fi