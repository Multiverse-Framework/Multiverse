#!/usr/bin/env bash
set -euo pipefail

# --------------------------
# Config
# --------------------------
SESH="teleop_iai_tiago"
MUJOCO_VERSION="${MUJOCO_VERSION:-3.4.0}"

DEMO_DIR="Demos/1_TiagoDualInApartment"
VENV_DIR="${DEMO_DIR}/multiverse"
MUJOCO_DIR="${DEMO_DIR}/mujoco-${MUJOCO_VERSION}"
URDF_XACRO="${DEMO_DIR}/assets/urdf/iai_tiago.urdf"

ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

CATKIN_WS="MultiverseConnector/ros_connector/ros_ws/multiverse_ws"
ROSPKG_SETUP="${CATKIN_WS}/devel/setup.bash"

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
cd "$SCRIPT_DIR" || die "Cannot cd to script dir"
cd ../.. || die "Cannot cd to repo root"

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
  log "Building catkin workspace: ${CATKIN_WS}"
  need_cmd catkin
  pushd "$CATKIN_WS" >/dev/null
  catkin build
  popd >/dev/null
fi

ros_source "$ROSPKG_SETUP"

log "Setting /robot_description from xacro: ${URDF_XACRO}"
rosparam set /robot_description "$(xacro "$URDF_XACRO")"

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
./MultiverseServer/bin/multiverse_server_cpp --transport zmq --bind tcp://127.0.0.1:7000 --transport tcp --bind 192.168.102.35:8000
"

tmux_send "$SESH":0.1 \
"
source '${VENV_DIR}/bin/activate'
python ./MultiverseUtilities/multiverse_initializing.py --data_path=./${DEMO_DIR}/config/multiverse.yaml
"

tmux_send "$SESH":0.2 \
"
"

tmux_send "$SESH":0.3 \
"
"

tmux_send "$SESH":0.4 \
"
"

tmux_send "$SESH":0.5 \
"
set +u
source '${ROS_SETUP}'
source '${ROSPKG_SETUP}'
set -u
rosparam load ./${DEMO_DIR}/config/vr_teleop_noetic.yaml
cd ./${DEMO_DIR}
rosrun vr_teleop_action vr_teleop_action_server_node __name:=vr_teleop_action_server
"

tmux_send "$SESH":0.6 \
"
set +u
source '${ROS_SETUP}'
source '${ROSPKG_SETUP}'
set -u
rostopic pub /vr_teleop_action_server/goal vr_teleop_msgs/TeleopActionGoal -1 \"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  timeout:
    secs: -1
    nsecs: 0
\"
"

tmux_send "$SESH":0.7 \
"
source '${VENV_DIR}/bin/activate'
set +u
source '${ROS_SETUP}'
source '${ROSPKG_SETUP}'
set -u
multiverse_ros_connector --subscribers=\"{'joint_state':[{'meta_data':{'world_name':'world','length_unit':'m','angle_unit':'rad','mass_unit':'kg','time_unit':'s','handedness':'rhs'},'port':7300,'topic':'/joint_states','rate':60,'joint_types':{'torso_lift_joint':'prismatic'}}]}\"
"

tmux_send "$SESH":0.8 \
"
echo 'Pane 8: (placeholder)'
bash
"

tmux select-pane -t "$SESH":0.0
exec tmux attach -t "$SESH"