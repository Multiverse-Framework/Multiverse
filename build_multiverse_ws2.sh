#!/usr/bin/env sh

for distro in foxy humble jazzy; do
    if [ -f "/opt/ros/$distro/setup.sh" ]; then
        ROS2_DISTRO=$distro
        break
    fi
done

if [ -z "$ROS2_DISTRO" ]; then
    echo "No ROS2 distro found"
    exit 1
fi

cd $(dirname $0)

MULTIVERSE_DIR=$PWD/multiverse

# Build the workspace
cd $(dirname $0)/multiverse_ws2
rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS2_DISTRO
. /opt/ros/$ROS2_DISTRO/setup.sh

if [ "$ROS2_DISTRO" = "foxy" ]; then
    colcon build
elif [ "$ROS2_DISTRO" = "jazzy" ]; then
    colcon build --symlink-install
    ln -sf ${MULTIVERSE_DIR}/lib/libstdc++/libmultiverse_client_json.so ${MULTIVERSE_DIR}/../multiverse_ws2/install/multiverse_control/lib/libmultiverse_client_json.so
fi