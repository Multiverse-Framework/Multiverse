#!/bin/bash

handle_sigint() {
    echo "Caught SIGINT (Ctrl+C), exiting..."
    sleep 2
    exit 1
}

trap handle_sigint SIGINT

COMMAND_PATTERNS=("rviz")

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

SCENE_FILE=$1
if [[ "$SCENE_FILE" != /* ]]; then
    SCENE_FILE="$(pwd)/$SCENE_FILE"
fi

# Check if the file exists
if [ ! -f "$SCENE_FILE" ]; then
    echo "Error: File $SCENE_FILE not found."
    exit 1
fi

(source /opt/ros/noetic/setup.bash && source $MULTIVERSE_PATH/../multiverse_ws/devel/setup.bash && python3 $MULTIVERSE_PATH/modules/multiverse_connectors/scripts/view_multiverse.py --scene_file=$SCENE_FILE)
# (source /opt/ros/foxy/setup.bash && source $MULTIVERSE_PATH/../multiverse_ws2/install/local_setup.bash && python3 $MULTIVERSE_PATH/modules/multiverse_connectors/scripts/view_multiverse.py --scene_file=$SCENE_FILE)

# Your script's main logic here
echo "[multiverse_view] Running... Press Ctrl+C to exit"
while true; do
    sleep 1
done