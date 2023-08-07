#!/usr/bin/env sh

# Build the workspace
rosdep init
(cd $(dirname $0)/multiverse_ws; rosdep update; rosdep install --from-paths src --ignore-src -r -y; . /opt/ros/noetic/setup.sh; catkin build)
