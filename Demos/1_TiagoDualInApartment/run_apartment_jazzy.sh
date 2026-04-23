#!/usr/bin/env bash
set -euo pipefail

# --------------------------
# Config
# --------------------------
SESH="teleop_iai_tiago"
MUJOCO_VERSION="${MUJOCO_VERSION:-3.5.0}"

DEMO_DIR="Demos/1_TiagoDualInApartment"
VENV_DIR="${DEMO_DIR}/multiverse"
MUJOCO_DIR="${DEMO_DIR}/mujoco-${MUJOCO_VERSION}"
URDF_ROS2="${DEMO_DIR}/assets/urdf/iai_tiago_with_ros2_control_velocity.urdf"
MJCF_SCENE="${DEMO_DIR}/assets/mjcf/iai_tiago_velocity_in_apartment_with_multiverse.xml"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

COLCON_WS="MultiverseConnector/ros_connector/ros_ws/multiverse_ws2"
ROSPKG_SETUP="${COLCON_WS}/install/setup.bash"

REQ_LOCAL="${DEMO_DIR}/requirements.txt"
REQ_ROOT="requirements.txt"

# --------------------------
# Helpers
# --------------------------
log()  { echo -e "\n\033[1;32m[+] $*\033[0m"; }
warn() { echo -e "\n\033[1;33m[!] $*\033[0m"; }
die()  { echo -e "\n\033[1;31m[✗] $*\033[0m" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }

tmux_send() {
  local target="$1"; shift
  tmux send-keys -t "$target" "$*" C-m
}

ros_source() {
  local overlay="${1:-}"
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if [[ -n "$overlay" ]]; then
    # shellcheck disable=SC1090
    source "$overlay"
  fi
  set -u
}

if tmux has-session -t "$SESH" 2>/dev/null; then
  exec tmux attach -t "$SESH"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
cd ../..

need_cmd tmux
need_cmd python3
need_cmd wget
need_cmd tar

[[ -f "$ROS_SETUP" ]] || die "ROS setup not found: $ROS_SETUP"

if [[ ! -d "$VENV_DIR" ]]; then
  log "Creating venv: $VENV_DIR"
  mkdir -p "$(dirname "$VENV_DIR")"
  python3 -m venv "$VENV_DIR"
  # shellcheck disable=SC1090
  source "$VENV_DIR/bin/activate"
  python -m pip install -U pip
  pip install -r "$REQ_LOCAL"
  pip install -r "$REQ_ROOT"
  pip install -e "MultiverseConnector/ros_connector"
else
  # shellcheck disable=SC1090
  source "$VENV_DIR/bin/activate"
fi

if [[ ! -d "$MUJOCO_DIR" ]]; then
  log "Downloading MuJoCo ${MUJOCO_VERSION} -> ${MUJOCO_DIR}"
  mkdir -p "$DEMO_DIR"
  wget -qO- "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" \
    | tar -xz -C "$DEMO_DIR"
fi

mkdir -p "${MUJOCO_DIR}/bin/mujoco_plugin"
if compgen -G "MultiverseConnector/mujoco_connector/mujoco-${MUJOCO_VERSION}/*.so" >/dev/null; then
  cp -f "MultiverseConnector/mujoco_connector/mujoco-${MUJOCO_VERSION}/"*.so "${MUJOCO_DIR}/bin/mujoco_plugin/" || true
else
  warn "No plugin .so found at MultiverseConnector/mujoco_connector/mujoco-${MUJOCO_VERSION}/*.so"
fi

ros_source ""

if [[ ! -f "$ROSPKG_SETUP" ]]; then
  log "Building colcon workspace: ${COLCON_WS}"
  need_cmd colcon
  pushd "$COLCON_WS" >/dev/null
  colcon build --symlink-install
  popd >/dev/null
fi

ros_source "$ROSPKG_SETUP"

log "Starting tmux session: ${SESH}"
tmux new-session -d -s "$SESH" -n main
tmux set-option -t "$SESH" -g mouse on
tmux set-option -t "$SESH" -g history-limit 200000

tmux split-window -t "$SESH":0 -h
tmux split-window -t "$SESH":0 -h

mapfile -t COLS < <(tmux list-panes -t "$SESH":0 -F '#{pane_id}' | head -n 3)

for col in "${COLS[@]}"; do
  tmux split-window -t "$col" -v
  tmux split-window -t "$col" -v
done

tmux select-layout -t "$SESH":0 tiled

tmux_send "$SESH":0.0 \
"
./MultiverseServer/bin/multiverse_server_cpp --transport zmq --bind tcp://127.0.0.1:7000 --transport tcp --bind $(ip route get 1 | awk '{print $7; exit}'):8000
"

tmux_send "$SESH":0.1 \
"
source '${VENV_DIR}/bin/activate'
python ./MultiverseUtilities/multiverse_initializing.py --data_path=./${DEMO_DIR}/config/multiverse.yaml
export MUJOCO_VERSION='${MUJOCO_VERSION}'
./${MUJOCO_DIR}/bin/simulate ./${MJCF_SCENE}
"

tmux_send "$SESH":0.2 \
"
set +u
source '${ROS_SETUP}'
source '${ROSPKG_SETUP}'
set -u
ros2 run robot_state_publisher robot_state_publisher --ros-args --remap /robot_description:=/robot_description -p robot_description:=\"\$(xacro ./${URDF_ROS2} | sed 's|file://|file://${PWD}/Demos/1_TiagoDualInApartment/assets/urdf/|g')\" -r tf:=/tf
"

tmux_send "$SESH":0.3 \
"
set +u
source '${ROS_SETUP}'
source '${ROSPKG_SETUP}'
set -u
ros2 run controller_manager ros2_control_node --ros-args --remap /robot_description:=/robot_description --params-file './${DEMO_DIR}/config/ros2_control.yaml'
"

tmux_send "$SESH":0.4 \
"
set +u
source '${ROS_SETUP}'
source '${ROSPKG_SETUP}'
set -u
ros2 run controller_manager spawner joint_state_broadcaster upper_body_velocity_controller --param-file ./${DEMO_DIR}/config/ros2_control.yaml
ros2 run rviz2 rviz2 --display-config ./${DEMO_DIR}/config/rviz2.rviz
"

tmux_send "$SESH":0.5 \
"
source '${VENV_DIR}/bin/activate'
set +u
source '${ROS_SETUP}'
source '${ROSPKG_SETUP}'
set -u
# multiverse_ros_connector --publishers=\"{'tf': [{'meta_data': {'world_name': 'world', 'length_unit': 'm', 'angle_unit': 'rad', 'mass_unit': 'kg', 'time_unit': 's', 'handedness': 'rhs'}, 'port': 7305, 'topic': '/tf', 'rate': 60, 'root_frame_id': 'map'}]}\"  --subscribers=\"{}\"
"

tmux_send "$SESH":0.6 \
"
set +u
source '${ROS_SETUP}'
source '${ROSPKG_SETUP}'
set -u
# ros2 action send_goal /teleop vr_teleop_interfaces/action/Teleop \"timeout: {sec: -1}\"
"

tmux_send "$SESH":0.7 \
"
source '${VENV_DIR}/bin/activate'
set +u
source '${ROS_SETUP}'
source '${ROSPKG_SETUP}'
set -u
multiverse_ros_connector --publishers=\"{'tf': [{'meta_data': {'world_name': 'world', 'length_unit': 'm', 'angle_unit': 'rad', 'mass_unit': 'kg', 'time_unit': 's', 'handedness': 'rhs'}, 'port': 7305, 'topic': '/tf', 'rate': 60, 'root_frame_id': 'map'}], 'odom': [{'meta_data': {'world_name': 'world', 'length_unit': 'm', 'angle_unit': 'rad', 'mass_unit': 'kg', 'time_unit': 's', 'handedness': 'rhs'}, 'port': 7310, 'odom_topic': '/odom', 'tf_topic': '/tf', 'body': 'base_footprint', 'rate': 60}]}\" --subscribers=\"{'joint_state':[{'meta_data':{'world_name':'world','length_unit':'m','angle_unit':'rad','mass_unit':'kg','time_unit':'s','handedness':'rhs'},'port':7300,'topic':'/joint_states','rate':60,'joint_types':{'torso_lift_joint':'prismatic'}}], 'cmd_vel':[{'meta_data':{'world_name':'world','length_unit':'m','angle_unit':'rad','mass_unit':'kg','time_unit':'s','handedness':'rhs'},'port':7320,'topic':'/cmd_vel','body':'base_footprint'}], 'data':[{'meta_data':{'world_name':'world','length_unit':'m','angle_unit':'rad','mass_unit':'kg','time_unit':'s','handedness':'rhs'},'port':7325,'topic':'/gripper_left','msg_type':'std_msgs/Float64','send':{'gripper_right_fingers_actuator':['scalar']}}, {'meta_data':{'world_name':'world','length_unit':'m','angle_unit':'rad','mass_unit':'kg','time_unit':'s','handedness':'rhs'},'port':7330,'topic':'/gripper_right','msg_type':'std_msgs/Float64','send':{'gripper_left_fingers_actuator':['scalar']}}]}\"
"

tmux_send "$SESH":0.8 \
"
echo 'Pane 8: (placeholder)'
bash
"

tmux select-pane -t "$SESH":0.0
exec tmux attach -t "$SESH"