#!/usr/bin/env sh

# Build the workspace
cd $(dirname $0)/multiverse_ws2
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
. /opt/ros/foxy/setup.sh
colcon build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
