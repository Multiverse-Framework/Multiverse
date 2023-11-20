#!/usr/bin/env sh

# Build the workspace
cd $(dirname $0)/multiverse_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
. /opt/ros/noetic/setup.sh
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
