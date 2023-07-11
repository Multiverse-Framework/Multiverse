#!/usr/bin/env sh

# Update the submodules to make sure everything is there
git submodule update --init

SRC_PATH="$(dirname $0)/src"

# Specify the folder path to create
BUILD_PATH="$(dirname $0)/build"

# Build USD

# Check if the folder already exists
USD_BUILD_PATH="$BUILD_PATH/USD"
USD_SRC_PATH="$SRC_PATH/USD"
if [ ! -d "$USD_BUILD_PATH" ]; then
    # Create the folder if it doesn't exist
    mkdir -p "$USD_BUILD_PATH"
    echo "Folder created: $USD_BUILD_PATH"
else
    echo "Folder already exists: $USD_BUILD_PATH"
fi

(python3 $USD_SRC_PATH/build_scripts/build_usd.py $USD_BUILD_PATH)

# Build the workspace
rosdep init
(cd multiverse_ws; . /opt/ros/noetic/setup.sh; catkin build)
