#!/bin/bash

SESH="wan_teleop_tiago_dual_server"

tmux has-session -t "$SESH" 2>/dev/null

if [ $? -eq 0 ]; then
  tmux attach -t "$SESH"
  exit 0
fi

cd $(dirname $0) || exit

if [ ! -d "multiverse" ]; then
  echo "Environment folder not found. Creating it now..."
  python3 -m venv multiverse
  . multiverse/bin/activate
  python -m pip install -U pip
  # Use -f to ensure we find the requirements file relative to the script location
  pip install -r requirements.txt
  (cd ../.. && pip install -r requirements.txt)
fi

# Check if CLOUDFLARE_TUNNEL_TOKEN is set
# if [ -z "$CLOUDFLARE_TUNNEL_TOKEN" ]; then
#   echo "Error: CLOUDFLARE_TUNNEL_TOKEN environment variable is not set."
#   echo ""
#   echo "To fix this:"
#   echo "1. Edit .env and add your Cloudflare tunnel token"
#   echo "2. Load the environment variables:  source .env"
#   echo "3. Run this script again"
#   echo ""
#   echo "Alternatively, you can set the token directly:"
#   echo "  export CLOUDFLARE_TUNNEL_TOKEN='your_token_here'"
#   echo ""
#   exit 1
# fi

# Create new detached tmux session
tmux new-session -d -s "$SESH" -n cloudflare

############################################
# Window 0 — Cloudflare Tunnel
############################################
# tmux send-keys -t "$SESH":0 "
# docker run --rm --network host \
#   -v \"$(pwd)/config.json:/etc/cloudflared/config.json\" \
#   cloudflare/cloudflared:latest \
#   tunnel --no-autoupdate --config /etc/cloudflared/config.json run --token \"$CLOUDFLARE_TUNNEL_TOKEN\"
# " C-m

cd ../..

############################################
# Window 1 — Multiverse Server
############################################
tmux new-window -t "$SESH":1 -n server
tmux send-keys -t "$SESH":1 "
./MultiverseServer/bin/multiverse_server_cpp --transport zmq --bind tcp://127.0.0.1:7000
" C-m

############################################
# Window 2 — Initialize + Start MuJoCo (Teleop Scene)
############################################
tmux new-window -t "$SESH":2 -n teleop
tmux send-keys -t "$SESH":2 "
. ./Demos/1_TiagoDualInApartment/multiverse/bin/activate
python ./MultiverseUtilities/multiverse_initializing.py --data_path=./Demos/1_TiagoDualInApartment/tiago_dual_init.yaml
export MUJOCO_VERSION=3.4.0
./Demos/1_TiagoDualInApartment/mujoco-\${MUJOCO_VERSION}/bin/simulate ./Demos/1_TiagoDualInApartment/assets/tiago_dual_teleop.xml
" C-m

############################################
# Window 3 — Start Digital Twin (Simulated Robot)
############################################
tmux new-window -t "$SESH":3 -n mujoco
tmux send-keys -t "$SESH":3 "
export MUJOCO_VERSION=3.4.0
./Demos/1_TiagoDualInApartment/mujoco-\${MUJOCO_VERSION}/bin/simulate ./Demos/1_TiagoDualInApartment/assets/scene_position.xml
" C-m

############################################
# Window 4 — Start robot_state_publisher
############################################
tmux new-window -t "$SESH":4 -n robot_state_publisher
tmux send-keys -t "$SESH":4 "
source /opt/ros/jazzy/setup.bash
source ./MultiverseConnector/ros_connector/ros_ws/multiverse_ws2/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args --remap /robot_description:=/robot_description -p robot_description:=\"\$(xacro ./Demos/1_TiagoDualInApartment/assets/tiago_dual_position.urdf | sed \"s|file://\\./|file://\$PWD/Demos/1_TiagoDualInApartment/assets/|g\")\" -r tf:=/tf
" C-m

############################################
# Window 5 — Start ros2_control_node
############################################
tmux new-window -t "$SESH":5 -n ros2_control_node
tmux send-keys -t "$SESH":5 "
source /opt/ros/jazzy/setup.bash
source ./MultiverseConnector/ros_connector/ros_ws/multiverse_ws2/install/setup.bash
ros2 run controller_manager ros2_control_node --ros-args --remap /robot_description:=/robot_description --params-file './Demos/1_TiagoDualInApartment/config/ros2_control.yaml'
" C-m

############################################
# Window 6 — Start spawner and rviz2
############################################
tmux new-window -t "$SESH":6 -n spawner
tmux send-keys -t "$SESH":6 "
source /opt/ros/jazzy/setup.bash
source ./MultiverseConnector/ros_connector/ros_ws/multiverse_ws2/install/setup.bash
ros2 run controller_manager spawner joint_state_broadcaster arm_position_controller torso_joint_trajectory_controller head_position_controller left_gripper_controller right_gripper_controller --param-file './Demos/1_TiagoDualInApartment/config/ros2_control.yaml'
ros2 run rviz2 rviz2 --display-config './Demos/1_TiagoDualInApartment/config/rviz2.rviz'
" C-m

############################################
# Window 7 — Run Finger Commands
############################################
tmux new-window -t "$SESH":7 -n fingers_command
tmux send-keys -t "$SESH":7 "
. ./Demos/1_TiagoDualInApartment/multiverse/bin/activate
python ./Demos/1_TiagoDualInApartment/fingers_command.py
" C-m

############################################
# Window 8 — Start Multiverse Smoothing
############################################
tmux new-window -t "$SESH":8 -n smoothing_1
tmux send-keys -t "$SESH":8 "
. ./Demos/1_TiagoDualInApartment/multiverse/bin/activate
python ./MultiverseUtilities/multiverse_smoothing.py \\
  --world=world \\
  --simulation_name=multiverse_smoother_1 \\
  --port=5001 \\
  --object_names=teleop_head_1_joint,teleop_head_2_joint,teleop_arm_left_1_joint,teleop_arm_left_2_joint,teleop_arm_left_3_joint,teleop_arm_left_4_joint,teleop_arm_left_5_joint,teleop_arm_left_6_joint,teleop_arm_left_7_joint,teleop_arm_right_1_joint,teleop_arm_right_2_joint,teleop_arm_right_3_joint,teleop_arm_right_4_joint,teleop_arm_right_5_joint,teleop_arm_right_6_joint,teleop_arm_right_7_joint \\
  --attribute_names=joint_angular_position \\
  --map=teleop_,mujoco_
" C-m

############################################
# Window 9 — Start Multiverse Smoothing
############################################
tmux new-window -t "$SESH":9 -n smoothing_2
tmux send-keys -t "$SESH":9 "
. ./Demos/1_TiagoDualInApartment/multiverse/bin/activate
python ./MultiverseUtilities/multiverse_smoothing.py \\
  --world=world \\
  --simulation_name=multiverse_smoother_2 \\
  --port=5002 \\
  --object_names=teleop_torso_lift_joint,teleop_gripper_left_left_finger_joint,teleop_gripper_left_right_finger_joint,teleop_gripper_right_left_finger_joint,teleop_gripper_right_right_finger_joint \\
  --attribute_names=joint_linear_position \\
  --map=teleop_,mujoco_
" C-m

############################################
# Window 10 — Upper Body Command
############################################
tmux new-window -t "$SESH":10 -n upper_body_command
tmux send-keys -t "$SESH":10 "
source /opt/ros/jazzy/setup.bash
. ./Demos/1_TiagoDualInApartment/multiverse/bin/activate
sleep 5
python ./Demos/1_TiagoDualInApartment/upper_body_command.py --port=4500
" C-m

############################################
# Attach to session
############################################
tmux select-window -t "$SESH":10
tmux attach -t "$SESH"
