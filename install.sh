#!/bin/bash

CURRENT_DIR=$PWD

cd $(dirname $0)

MULTIVERSE_DIR=$PWD/multiverse

INSTALL_ROS=true
INSTALL_PARSER=true
INSTALL_MUJOCO=true
INSTALL_ISAACLAB=true
INSTALL_KNOWLEDGE=true

while [ -n "$1" ]; do
    case "$1" in
        --minimal) echo "--minimal option passed"
            shift 1
            INSTALL_ROS=false
            INSTALL_PARSER=false
            INSTALL_KNOWLEDGE=false
            INSTALL_ISAACLAB=false
            break
        ;;
        --excludes) echo -n "--excludes option passed"
            shift 1
            if [ "$#" -eq 0 ]; then
                echo ""
                INSTALL_ROS=false
                INSTALL_PARSER=false
                INSTALL_MUJOCO=false
                INSTALL_ISAACLAB=false
                INSTALL_KNOWLEDGE=false
            else
                echo -n ", with value:"
                for module in "$@"; do
                    echo -n " $module"
                    shift 1
                    if [ "$module" = "ros" ]; then
                        INSTALL_ROS=false
                    elif [ "$module" = "parser" ]; then
                        INSTALL_PARSER=false
                    elif [ "$module" = "mujoco" ]; then
                        INSTALL_MUJOCO=false
                    elif [ "$module" = "isaaclab" ]; then
                        INSTALL_ISAACLAB=false
                    elif [ "$module" = "knowledge" ]; then
                        INSTALL_KNOWLEDGE=false
                    fi
                done
                echo ""
            fi
        ;;
        *) echo "Option $1 not recognized"
            shift 1
        ;;
    esac
done

# Update package lists
sudo apt-get update && sudo apt-get upgrade -y

# Install presiqisite for ubuntu
sudo apt-get install -y software-properties-common curl python-is-python3 python3-pip
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y python3.10-dev python3.10-venv

UBUNTU_VERSION=$(lsb_release -rs)

if [ $INSTALL_ROS = true ]; then
    if [ $UBUNTU_VERSION = "20.04" ]; then
        # Setup your sources.list
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

        PYTHON_EXECUTABLE=python3.8
        for virtualenvwrapper in $(which virtualenvwrapper.sh) /usr/share/virtualenvwrapper/virtualenvwrapper.sh /usr/local/bin/virtualenvwrapper.sh /home/$USER/.local/bin/virtualenvwrapper.sh; do
            if [ -f $virtualenvwrapper ]; then
                break
            fi
        done
        if [ ! -f $virtualenvwrapper ]; then
            echo "virtualenvwrapper.sh not found"
        else
            . $virtualenvwrapper
            mkvirtualenv --system-site-packages multiverse3.8 -p $PYTHON_EXECUTABLE
            $PYTHON_EXECUTABLE -m pip install -U pip build setuptools packaging distro
        fi
    elif [ $UBUNTU_VERSION = "22.04" ]; then
        # Setup your sources.list
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

        # Update package lists
        sudo apt-get update && sudo apt-get upgrade -y

        # Install ROS2
        sudo apt-get install -y ros-humble-desktop
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

        PYTHON_EXECUTABLE=python3.12
        for virtualenvwrapper in $(which virtualenvwrapper.sh) /usr/share/virtualenvwrapper/virtualenvwrapper.sh /usr/local/bin/virtualenvwrapper.sh /home/$USER/.local/bin/virtualenvwrapper.sh; do
            if [ -f $virtualenvwrapper ]; then
                break
            fi
        done
        if [ ! -f $virtualenvwrapper ]; then
            echo "virtualenvwrapper.sh not found"
        else
            . $virtualenvwrapper
            mkvirtualenv --system-site-packages multiverse3.12 -p $PYTHON_EXECUTABLE
            $PYTHON_EXECUTABLE -m pip install -U pip build setuptools packaging distro
            $PYTHON_EXECUTABLE -m pip install -U urdf_parser_py
        fi
    fi
fi

# Install jsoncpp
sudo apt-get install -y libjsoncpp-dev

# Install zmqpp
sudo apt-get install -y libzmq3-dev

# Install boost
sudo apt-get install -y libboost-dev libboost-filesystem-dev

# Install pybind11
sudo apt-get install -y pybind11-dev

if [ $UBUNTU_VERSION = "20.04" ]; then
    # Install and link clang-11 for creating shared library
    sudo apt-get install -y clang-11 llvm-11-dev libc++-11-dev libc++abi-11-dev libstdc++-11-dev 
    sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/x86_64-linux-gnu/libstdc++.so

    # Install and link gcc-11
    sudo apt-get install -y gcc-11

    # Install and link gcc+11
    sudo apt-get install -y g++-11

    # Upgrade pip
    pip install -U pip

    # Setup virtual environment
    pip install virtualenvwrapper
elif [ $UBUNTU_VERSION = "22.04" ]; then
    # Install and link clang-14 for creating shared library
    sudo apt-get install -y clang-14 llvm-14-dev libc++-14-dev libc++abi-14-dev libstdc++-12-dev 
    sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/x86_64-linux-gnu/libstdc++.so

    # Setup virtual environment
    python3 -m pip install virtualenvwrapper
elif [ $UBUNTU_VERSION = "24.04" ]; then
    # Install and link clang-17 for creating shared library
    sudo apt-get install -y clang-17 llvm-17-dev libc++-17-dev libc++abi-17-dev libstdc++-14-dev 
    sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/x86_64-linux-gnu/libstdc++.so

    # Setup virtual environment
    python3 -m pip install virtualenvwrapper --break-system-packages
fi

PYTHON_EXECUTABLE=python3.10

# Setup virtual environment
for virtualenvwrapper in $(which virtualenvwrapper.sh) /usr/share/virtualenvwrapper/virtualenvwrapper.sh /usr/local/bin/virtualenvwrapper.sh /home/$USER/.local/bin/virtualenvwrapper.sh; do
    if [ -f $virtualenvwrapper ]; then
        break
    fi
done
if [ ! -f $virtualenvwrapper ]; then
    echo "virtualenvwrapper.sh not found"
else
    . $virtualenvwrapper
    mkvirtualenv --system-site-packages multiverse -p $PYTHON_EXECUTABLE

    # Install and upgrade pip build setuptools packaging distro
    $PYTHON_EXECUTABLE -m pip install -U pip build setuptools packaging distro

    if [ $INSTALL_PARSER = true ]; then
        git submodule update --init multiverse/modules/multiverse_parser
        
        # Install additional packages for blender
        sudo apt-get install -y build-essential git git-lfs subversion cmake libx11-dev libxxf86vm-dev libxcursor-dev libxi-dev libxrandr-dev libxinerama-dev libegl-dev libwayland-dev wayland-protocols libxkbcommon-dev libdbus-1-dev linux-libc-dev
        
        # Install additional packages for USD
        sudo apt-get install -y libxcb-cursor0
        $PYTHON_EXECUTABLE -m pip install -r $MULTIVERSE_DIR/modules/multiverse_parser/requirements.txt
    fi

    if [ $INSTALL_KNOWLEDGE = true ]; then
        git submodule update --init multiverse/modules/multiverse_knowledge
        (cd $MULTIVERSE_DIR/modules/multiverse_knowledge; git submodule update --init ease_lexical_resources)

        # Install additional packages for multiverse_knowledge
        $PYTHON_EXECUTABLE -m pip install -r $MULTIVERSE_DIR/modules/multiverse_knowledge/requirements.txt
    fi

    if [ $INSTALL_MUJOCO = true ]; then
        # Install additional packages for MuJoCo
        sudo apt-get install -y libgl1-mesa-dev libglu1-mesa-dev libxt-dev

        # Install glfw3
        sudo apt-get install -y libglfw3
        sudo apt-get install -y libglfw3-dev
        $PYTHON_EXECUTABLE -m pip install -r $MULTIVERSE_DIR/modules/multiverse_connectors/src/multiverse_simulators/src/mujoco_connector/requirements.txt
    fi

    if [ $INSTALL_ISAACLAB = true ]; then
        $PYTHON_EXECUTABLE -m pip install -r $MULTIVERSE_DIR/modules/multiverse_connectors/src/multiverse_simulators/src/isaac_sim_connector/requirements.txt
    fi
fi