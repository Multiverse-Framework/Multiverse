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

MUJOCO_VERSION=3.4.0
if [ ! -d "$PWD/Demos/1_TiagoDualInApartment/mujoco-${MUJOCO_VERSION}" ]; then
  echo "MuJoCo not found. Downloading MuJoCo ${MUJOCO_VERSION}..."
  wget -qO- https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz | tar -xz -C $PWD/Demos/1_TiagoDualInApartment/
  cp -f ./MultiverseConnector/mujoco_connector/mujoco-${MUJOCO_VERSION}/*.so ./Demos/1_TiagoDualInApartment/mujoco-${MUJOCO_VERSION}/bin/mujoco_plugin/
fi

source /opt/ros/noetic/setup.bash
ROSPKG_PATH="$PWD"/MultiverseConnector/ros_connector/ros_ws/multiverse_ws/devel/setup.bash
if [ ! -f "${ROSPKG_PATH}" ]; then
  echo "ROS package not found. Building it..."
  cd "$PWD"/MultiverseConnector/ros_connector/ros_ws/multiverse_ws || exit
  catkin build
  cd ../../
  pip install -e .
  cd ../../
fi

rosparam set /robot_description "$(xacro ./Demos/1_TiagoDualInApartment/assets/urdf/iai_tiago.urdf)"

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
"./MultiverseServer/bin/multiverse_server_cpp --transport zmq --bind tcp://127.0.0.1:7000 --transport tcp --bind 192.168.102.35:8000" C-m

# Pane 1 - MuJoCo
tmux send-keys -t "$SESH":0.1 \
"source ./Demos/1_TiagoDualInApartment/multiverse/bin/activate
python ./MultiverseUtilities/multiverse_initializing.py --data_path=./Demos/1_TiagoDualInApartment/config/multiverse.yaml
export MUJOCO_VERSION=3.4.0
./Demos/1_TiagoDualInApartment/mujoco-\${MUJOCO_VERSION}/bin/simulate ./Demos/1_TiagoDualInApartment/assets/mjcf/scene_position_with_multiverse.xml" C-m

# Pane 2 - robot_state_publisher
tmux send-keys -t "$SESH":0.2 \
"source /opt/ros/noetic/setup.bash
source \"${ROSPKG_PATH}\"
rosrun robot_state_publisher robot_state_publisher tf:=/tf" C-m

# Pane 3 - multiverse_control_node
JSON_CONFIG="{\\\"host\\\":\\\"tcp://127.0.0.1\\\",\\\"server_port\\\":7000,\\\"client_port\\\":7601,\\\"meta_data\\\":{\\\"world_name\\\":\\\"world\\\",\\\"length_unit\\\":\\\"m\\\",\\\"angle_unit\\\":\\\"rad\\\",\\\"mass_unit\\\":\\\"kg\\\",\\\"time_unit\\\":\\\"s\\\",\\\"handedness\\\":\\\"rhs\\\"},\\\"controller_manager\\\":{\\\"robot\\\":\\\"iai_tiago\\\",\\\"robot_description\\\":\\\"/robot_description\\\",\\\"actuators\\\":{\\\"torso_lift_joint_position\\\":\\\"torso_lift_joint\\\",\\\"arm_left_1_joint_position\\\":\\\"arm_left_1_joint\\\",\\\"arm_left_2_joint_position\\\":\\\"arm_left_2_joint\\\",\\\"arm_left_3_joint_position\\\":\\\"arm_left_3_joint\\\",\\\"arm_left_4_joint_position\\\":\\\"arm_left_4_joint\\\",\\\"arm_left_5_joint_position\\\":\\\"arm_left_5_joint\\\",\\\"arm_left_6_joint_position\\\":\\\"arm_left_6_joint\\\",\\\"arm_left_7_joint_position\\\":\\\"arm_left_7_joint\\\",\\\"arm_right_1_joint_position\\\":\\\"arm_right_1_joint\\\",\\\"arm_right_2_joint_position\\\":\\\"arm_right_2_joint\\\",\\\"arm_right_3_joint_position\\\":\\\"arm_right_3_joint\\\",\\\"arm_right_4_joint_position\\\":\\\"arm_right_4_joint\\\",\\\"arm_right_5_joint_position\\\":\\\"arm_right_5_joint\\\",\\\"arm_right_6_joint_position\\\":\\\"arm_right_6_joint\\\",\\\"arm_right_7_joint_position\\\":\\\"arm_right_7_joint\\\",\\\"head_1_joint_position\\\":\\\"head_1_joint\\\",\\\"head_2_joint_position\\\":\\\"head_2_joint\\\"},\\\"init_joint_state\\\":{\\\"arm_left_1_joint\\\":0.27,\\\"arm_left_2_joint\\\":-1.07,\\\"arm_left_3_joint\\\":1.5,\\\"arm_left_4_joint\\\":1.96,\\\"arm_left_5_joint\\\":-2.0,\\\"arm_left_6_joint\\\":1.2,\\\"arm_left_7_joint\\\":0.5,\\\"arm_right_1_joint\\\":0.27,\\\"arm_right_2_joint\\\":-1.07,\\\"arm_right_3_joint\\\":1.5,\\\"arm_right_4_joint\\\":1.96,\\\"arm_right_5_joint\\\":-2.0,\\\"arm_right_6_joint\\\":1.2,\\\"arm_right_7_joint\\\":0.5}}}"

tmux send-keys -t "$SESH":0.3 "
source /opt/ros/noetic/setup.bash
source ${ROSPKG_PATH}
rosrun multiverse_control multiverse_control_node robot_description:=/robot_description \"$JSON_CONFIG\" " C-m

# Pane 4 - spawn controllers + rviz2
tmux send-keys -t "$SESH":0.4 \
"source /opt/ros/noetic/setup.bash
source ${ROSPKG_PATH}
rosparam load ./Demos/1_TiagoDualInApartment/config/ros_control.yaml
rosrun controller_manager spawner joint_state_controller upper_body_position_controller" C-m

# Pane 5 - vr_teleop_action_server
tmux send-keys -t "$SESH":0.5 \
"source /opt/ros/noetic/setup.bash
source ${ROSPKG_PATH}
rosparam load ./Demos/1_TiagoDualInApartment/config/vr_teleop_noetic.yaml
cd ./Demos/1_TiagoDualInApartment
rosrun vr_teleop_action vr_teleop_action_server_node __name:=vr_teleop_action_server" C-m

# Pane 6 - run vr_teleop_action_client
tmux send-keys -t "$SESH":0.6 \
"source /opt/ros/noetic/setup.bash
source ${ROSPKG_PATH}" C-m

# Pane 7 - run joint_state_subscriber
tmux send-keys -t "$SESH":0.7 \
"
# source ./Demos/1_TiagoDualInApartment/multiverse/bin/activate
# source /opt/ros/jazzy/setup.bash
# source ${ROSPKG_PATH}
# multiverse_ros_connector --subscribers=\\\"{'joint_state': [{'meta_data': {'world_name': 'world', 'length_unit': 'm', 'angle_unit': 'rad', 'mass_unit': 'kg', 'time_unit': 's', 'handedness': 'rhs'}, 'port': 7300, 'topic': '/joint_states', 'rate': 60, 'joint_types': {'torso_lift_joint': 'prismatic'}}]}\\\"
" C-m

# Pane 8 - 
tmux send-keys -t "$SESH":0.8 \
"" C-m

tmux select-pane -t "$SESH":0.0
tmux attach -t "$SESH"
