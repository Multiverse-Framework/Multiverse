#!/usr/bin/env sh

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

# Build the workspace
# cd universim_ws && . /opt/ros/$ROS_DISTRO/setup.sh && catkin build
