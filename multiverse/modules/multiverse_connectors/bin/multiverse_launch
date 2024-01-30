#!/bin/bash

handle_sigint() {
    echo "Caught SIGINT (Ctrl+C), exiting..."
    sleep 2
    exit 1
}

trap handle_sigint SIGINT

COMMAND_PATTERNS=("multiverse_socket_node" "rviz" "multiverse_control_node" "mujoco" "multiverse_server")

for COMMAND_PATTERN in "${COMMAND_PATTERNS[@]}"; do
    # Get the PIDs of all matching processes
    PIDS=$(pgrep -a -f "$COMMAND_PATTERN" | grep "$COMMAND_PATTERN" | awk '{print $1}')
    
    if [ -n "$PIDS" ]; then
        # Use kill to terminate the processes
        for PID in $PIDS; do
            if kill -0 "$PID" 2>/dev/null; then
                kill -9 $PID
                # Check if kill was successful
                if kill -0 $PID 2>/dev/null; then
                    echo "Process $PID terminated."
                else
                    echo "Failed to terminate process $PID. It may require stronger measures."
                fi
            fi
        done
    fi
done

MULTIVERSE_PATH=$(dirname $(dirname "$0"))

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

(python3 $MULTIVERSE_PATH/modules/multiverse_connectors/scripts/launch_multiverse_server.py --muv_file=$MUV_FILE)
(python3 $MULTIVERSE_PATH/modules/multiverse_connectors/scripts/launch_simulators.py --muv_file=$MUV_FILE)
(source $MULTIVERSE_PATH/../multiverse_ws/devel/setup.bash && python3 $MULTIVERSE_PATH/modules/multiverse_connectors/scripts/launch_ros.py --muv_file=$MUV_FILE)
(source /opt/ros/foxy/setup.bash && source $MULTIVERSE_PATH/../multiverse_ws2/install/local_setup.bash && python3 $MULTIVERSE_PATH/modules/multiverse_connectors/scripts/launch_ros.py --muv_file=$MUV_FILE)

# Your script's main logic here
echo "[multiverse_launch] Running... Press Ctrl+C to exit"
while true; do
    sleep 1
done