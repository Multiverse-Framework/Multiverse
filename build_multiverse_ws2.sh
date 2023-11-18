#!/usr/bin/env sh

# Build the workspace
cd $(dirname $0)/multiverse_ws2
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
. /opt/ros/foxy/setup.sh
colcon build
