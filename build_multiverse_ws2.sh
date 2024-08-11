#!/usr/bin/env sh

for distro in foxy jazzy; do
    if [ -f "/opt/ros/$distro/setup.sh" ]; then
        ROS2_DISTRO=$distro
        break
    fi
done

if [ -z "$ROS2_DISTRO" ]; then
    echo "No ROS2 distro found"
    exit 1
fi

# Build the workspace
cd $(dirname $0)/multiverse_ws2
rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS2_DISTRO
. /opt/ros/$ROS2_DISTRO/setup.sh
colcon build --symlink-install
