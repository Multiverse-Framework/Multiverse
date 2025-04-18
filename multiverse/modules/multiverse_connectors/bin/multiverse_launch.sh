#!/bin/bash

handle_sigint() {
    echo "Caught SIGINT (Ctrl+C), exiting..."
    sleep 2
    exit 1
}

trap handle_sigint SIGINT

NO_MULTIVERSE_SERVER=false
KILL_EVERYTHING_FROM_START=true
COMMAND_PATTERNS=("multiverse_socket_node" "multiverse_ros_run" "rviz/rviz" "rviz2/rviz2" "multiverse_control_node" "isaac-sim-4.2.0/kit/python/bin/python3" "mujoco" "multiverse_server")

for arg in "$@"; do
    if [ "$arg" == "--no-multiverse-server" ]; then
        NO_MULTIVERSE_SERVER=true
        COMMAND_PATTERNS=("multiverse_socket_node" "multiverse_ros_run" "rviz/rviz" "rviz2/rviz2" "multiverse_control_node" "isaac-sim-4.2.0/kit/python/bin/python3" "mujoco")
    fi
    if [ "$arg" == "--keep-everything" ]; then
        KILL_EVERYTHING_FROM_START=false
    fi
done

if [ "$KILL_EVERYTHING_FROM_START" = true ]; then
    echo "Killing all processes..."
    for COMMAND_PATTERN in "${COMMAND_PATTERNS[@]}"; do
        # Get the PIDs of all matching processes
        PIDS=$(pgrep -a -f "$COMMAND_PATTERN" | grep "$COMMAND_PATTERN" | awk '{print $1}')
        
        if [ -n "$PIDS" ]; then
            # Use kill to terminate the processes
            for PID in $PIDS; do
                if kill -0 "$PID" 2>/dev/null; then
                    kill -9 "$PID"
                    # Check if kill was successful
                    if kill -0 "$PID" 2>/dev/null; then
                        echo "Process $PID terminated."
                    else
                        echo "Failed to terminate process $PID. It may require stronger measures."
                    fi
                fi
            done
        fi
    done
fi

MULTIVERSE_PATH=$(dirname "$(dirname "$0")")

# Check if an argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 path/to/your/project.muv"
    MUV_FILE=$MULTIVERSE_PATH/resources/muv/table_with_bowling.muv
else
    # File name of the MUV file
    MUV_FILE=$1
    if [[ "$MUV_FILE" != /* ]]; then
        MUV_FILE="$(pwd)/$MUV_FILE"
    fi
fi

# Check if the file exists
if [ ! -f "$MUV_FILE" ]; then
    echo "Error: File $MUV_FILE not found."
    exit 1
fi

for virtualenvwrapper in $(which virtualenvwrapper.sh) /usr/share/virtualenvwrapper/virtualenvwrapper.sh /usr/local/bin/virtualenvwrapper.sh /home/$USER/.local/bin/virtualenvwrapper.sh; do
    if [ -f $virtualenvwrapper ]; then
        . $virtualenvwrapper
        break
    fi
done
if [ ! -f $virtualenvwrapper ]; then
    echo "virtualenvwrapper.sh not found"
    exit 1
fi

workon multiverse

PYTHON_EXECUTABLE=python3.10

if [ "$NO_MULTIVERSE_SERVER" = true ]; then
    echo "Skipping the Multiverse server..."
else
    echo "Launching the Multiverse server..."
    ($PYTHON_EXECUTABLE "$MULTIVERSE_PATH"/modules/multiverse_connectors/scripts/launch_multiverse_server.py --muv_file="$MUV_FILE")
fi
($PYTHON_EXECUTABLE "$MULTIVERSE_PATH"/modules/multiverse_connectors/scripts/launch_simulators.py --muv_file="$MUV_FILE")

for distro in noetic; do
    if [ -f "/opt/ros/$distro/setup.sh" ]; then
        ROS_DISTRO=$distro
        break
    fi
done

MULTIVERSE_WS_PATH="$MULTIVERSE_PATH"/../multiverse_ws/devel/setup.bash
if [ "$ROS_DISTRO" ] && [ -f "$MULTIVERSE_WS_PATH" ]; then
    (source $MULTIVERSE_WS_PATH && $PYTHON_EXECUTABLE "$MULTIVERSE_PATH"/modules/multiverse_connectors/scripts/launch_ros.py --muv_file="$MUV_FILE")
fi

for distro in foxy humble jazzy; do
    if [ -f "/opt/ros/$distro/setup.sh" ]; then
        ROS2_DISTRO=$distro
        break
    fi
done

MULTIVERSE_WS2_PATH="$MULTIVERSE_PATH"/../multiverse_ws2/install/local_setup.bash
if [ "$ROS2_DISTRO" ] && [ -f "$MULTIVERSE_WS2_PATH" ]; then
    if [ $ROS2_DISTRO = "foxy" ]; then
        workon multiverse3.8
        PYTHON_EXECUTABLE=python3.8 # ROS 2 Foxy only supports Python 3.8
    elif [ $ROS2_DISTRO = "jazzy" ]; then
        workon multiverse3.12
        PYTHON_EXECUTABLE=python3.12 # ROS 2 Jazzy only supports Python 3.12
    fi
    (source /opt/ros/$ROS2_DISTRO/setup.bash && source "$MULTIVERSE_WS2_PATH" && $PYTHON_EXECUTABLE "$MULTIVERSE_PATH"/modules/multiverse_connectors/scripts/launch_ros.py --muv_file="$MUV_FILE")
fi

# Your script's main logic here
echo "[multiverse_launch] Running... Press Ctrl+C to exit"
while true; do
    sleep 1
done