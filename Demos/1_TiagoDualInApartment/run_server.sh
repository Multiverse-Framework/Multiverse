#!/bin/bash

SESH="wan_teleop_vr_h3_1_server"

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
tmux new-window -t "$SESH":2 -n mujoco
tmux send-keys -t "$SESH":2 "
. ./Demos/1_TiagoDualInApartment/multiverse/bin/activate
python ./MultiverseUtilities/multiverse_initializing.py \
  --data_path=./Demos/1_TiagoDualInApartment/tiago_dual_init.yaml
export MUJOCO_VERSION=3.4.0
./Demos/1_TiagoDualInApartment/mujoco-\${MUJOCO_VERSION}/bin/simulate \
  ./Demos/1_TiagoDualInApartment/assets/tiago_dual_teleop.xml
" C-m

############################################
# Window 3 — Start ros2_control_node
############################################
tmux new-window -t "$SESH":3 -n ros2_control_node
tmux send-keys -t "$SESH":3 "
source /opt/ros/jazzy/setup.bash
source ./MultiverseConnector/ros_connector/ros_ws/multiverse_ws2/install/setup.bash
ros2 run controller_manager ros2_control_node --ros-args --params-file './Demos/1_TiagoDualInApartment/config/ros2_control.yaml'
" C-m

############################################
# Attach to session
############################################
tmux select-window -t "$SESH":0
tmux attach -t "$SESH"
