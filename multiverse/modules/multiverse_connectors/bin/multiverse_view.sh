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

# Split the input string into an array of variables
read -ra vars <<< "$@"

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

# Print each variable
for var in "${vars[@]}"; do
    echo "$var"
    SCENE_FILE=$var
    if [[ "$SCENE_FILE" != /* ]]; then
        SCENE_FILE="$(pwd)/$SCENE_FILE"
    fi

    # Check if the file exists
    if [ ! -f "$SCENE_FILE" ]; then
        echo "Error: File $SCENE_FILE not found."
        exit 1
    fi

    # Check if ROS2 or ROS1 exists
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        workon multiverse3.8
        source /opt/ros/foxy/setup.bash && source $MULTIVERSE_PATH/../multiverse_ws2/install/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        workon multiverse
        source /opt/ros/humble/setup.bash && source $MULTIVERSE_PATH/../multiverse_ws2/install/setup.bash
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        workon multiverse3.12
        source /opt/ros/jazzy/setup.bash && source $MULTIVERSE_PATH/../multiverse_ws2/install/setup.bash
    else
        echo "Warning: ROS not found."
    fi

    python3 $MULTIVERSE_PATH/modules/multiverse_connectors/scripts/view_multiverse.py --scene_file=$SCENE_FILE
done

# Your script's main logic here
echo "[multiverse_view] Running... Press Ctrl+C to exit"
while true; do
    sleep 1
done