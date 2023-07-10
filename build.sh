#!/usr/bin/env sh

cd $(dirname $0)

# Update the submodules to make sure everything is there
git submodule update --init

# Specify the folder path to create
BUILD_PATH="build"

USD_BUILD_PATH="$BUILD_PATH/USD"

# Check if the folder already exists
if [ ! -d "$USD_BUILD_PATH" ]; then
    # Create the folder if it doesn't exist
    mkdir -p "$USD_BUILD_PATH"
    echo "Folder created: $USD_BUILD_PATH"
else
    echo "Folder already exists: $USD_BUILD_PATH"
fi

python3 src/USD/build_scripts/build_usd.py $USD_BUILD_PATH

PYBIND11_BUILD_PATH="$BUILD_PATH/pybind11"

# Check if the folder already exists
if [ ! -d "$PYBIND11_BUILD_PATH" ]; then
    # Create the folder if it doesn't exist
    mkdir -p "$PYBIND11_BUILD_PATH"
    echo "Folder created: $PYBIND11_BUILD_PATH"
else
    echo "Folder already exists: $PYBIND11_BUILD_PATH"
fi

(cd $PYBIND11_BUILD_PATH; cmake ../../src/pybind11 -DCMAKE_INSTALL_PREFIX=$BUILD_PATH; make install)

# Build the workspace
# rosdep init
# (cd multiverse_ws; . /opt/ros/noetic/setup.sh; catkin build)
