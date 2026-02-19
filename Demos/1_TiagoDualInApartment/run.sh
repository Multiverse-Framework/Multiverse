#!/bin/bash

SESH="teleop_iai_tiago"

# If session exists, attach
if tmux has-session -t "$SESH" 2>/dev/null; then
  tmux attach -t "$SESH"
  exit 0
fi

cd "$(dirname "$0")" || exit 1

if [ ! -d "multiverse" ]; then
  echo "Environment folder not found. Creating it now..."
  python3 -m venv multiverse
  . multiverse/bin/activate
  python -m pip install -U pip
  pip install -r requirements.txt
  (cd ../.. && pip install -r requirements.txt)
fi

cd ../..

MUJOCO_VERSION=3.5.0
if [ ! -d "$PWD/Demos/1_TiagoDualInApartment/mujoco-${MUJOCO_VERSION}" ]; then
  echo "MuJoCo not found. Downloading MuJoCo ${MUJOCO_VERSION}..."
  wget -qO- https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz | tar -xz -C $PWD/Demos/1_TiagoDualInApartment/
  cp -f ./MultiverseConnector/mujoco_connector/mujoco-${MUJOCO_VERSION}/*.so ./Demos/1_TiagoDualInApartment/mujoco-${MUJOCO_VERSION}/bin/mujoco_plugin/
fi

ROSPKG_PATH="$PWD"/MultiverseConnector/ros_connector/ros_ws/multiverse_ws2/install/setup.bash
if [ ! -f "${ROSPKG_PATH}" ]; then
  echo "ROS package not found. Building it..."
  source /opt/ros/jazzy/setup.bash
  cd "$PWD"/MultiverseConnector/ros_connector/ros_ws/multiverse_ws2 || exit
  colcon build --symlink-install
  cd ../../../../
fi

tmux new-session -d -s "$SESH" -n server

tmux set-option -t "$SESH" -g mouse on
tmux set-option -t "$SESH" -g history-limit 200000

# --- 3 columns (make 3 panes horizontally) ---
tmux split-window -t "$SESH":0 -h
tmux split-window -t "$SESH":0 -h

mapfile -t COLS < <(tmux list-panes -t "$SESH":0 -F '#{pane_id}' | head -n 3)

# --- for each column, split twice vertically to make 3 rows ---
for col in "${COLS[@]}"; do
  tmux select-pane -t "$col"
  tmux split-window -t "$col" -v             # add 2nd row
  tmux select-pane -t "$col"
  tmux split-window -t "$col" -v             # add 3rd row
done

tmux select-layout -t "$SESH":0 tiled

# Pane 0 - Multiverse Server
tmux send-keys -t "$SESH":0.0 \
"./MultiverseServer/bin/multiverse_server_cpp --transport zmq --bind tcp://127.0.0.1:7000" C-m

# Pane 1 - MuJoCo
tmux send-keys -t "$SESH":0.1 \
"source ./Demos/1_TiagoDualInApartment/multiverse/bin/activate
python ./MultiverseUtilities/multiverse_initializing.py --data_path=./Demos/1_TiagoDualInApartment/config/multiverse.yaml
export MUJOCO_VERSION=3.5.0
./Demos/1_TiagoDualInApartment/mujoco-\${MUJOCO_VERSION}/bin/simulate ./Demos/1_TiagoDualInApartment/assets/mjcf/scene_position.xml" C-m

# Pane 2 - robot_state_publisher
tmux send-keys -t "$SESH":0.2 \
"source /opt/ros/jazzy/setup.bash
source ${ROSPKG_PATH}
ros2 run robot_state_publisher robot_state_publisher --ros-args --remap /robot_description:=/robot_description -p robot_description:=\"\$(xacro ./Demos/1_TiagoDualInApartment/assets/urdf/iai_tiago.urdf)\" -r tf:=/tf" C-m

# Pane 3 - ros2_control_node
tmux send-keys -t "$SESH":0.3 \
"source /opt/ros/jazzy/setup.bash
source ${ROSPKG_PATH}
ros2 run controller_manager ros2_control_node --ros-args --remap /robot_description:=/robot_description --params-file './Demos/1_TiagoDualInApartment/config/ros2_control.yaml'
" C-m

# Pane 4 - spawn controllers + rviz2
tmux send-keys -t "$SESH":0.4 \
"source /opt/ros/jazzy/setup.bash
source ${ROSPKG_PATH}
ros2 run controller_manager spawner joint_state_broadcaster upper_body_position_controller --param-file ./Demos/1_TiagoDualInApartment/config/ros2_control.yaml
cp ./Demos/1_TiagoDualInApartment/assets/urdf/iai_tiago.urdf /tmp/iai_tiago.urdf
sed -i 's|file://\([^/]\)|file://'"'"'$PWD'"'"'/./Demos/1_TiagoDualInApartment/assets/urdf/\1|g' /tmp/iai_tiago.urdf
ros2 run rviz2 rviz2 --display-config ./Demos/1_TiagoDualInApartment/config/rviz2.rviz" C-m

# Pane 5 - vr_teleop_action_server
tmux send-keys -t "$SESH":0.5 \
"source /opt/ros/jazzy/setup.bash
source ${ROSPKG_PATH}
cd ./Demos/1_TiagoDualInApartment
ros2 run vr_teleop_action vr_teleop_action_server --ros-args --params-file ./config/vr_teleop.yaml" C-m

# Pane 6 - 
tmux send-keys -t "$SESH":0.6 \
"source /opt/ros/jazzy/setup.bash
source ${ROSPKG_PATH}
ros2 action send_goal /teleop vr_teleop_interfaces/action/Teleop \"timeout: {sec: -1}\"" C-m

# Pane 7 - 
tmux send-keys -t "$SESH":0.7 \
"" C-m

# Pane 8 - 
tmux send-keys -t "$SESH":0.8 \
"" C-m

tmux select-pane -t "$SESH":0.0
tmux attach -t "$SESH"
