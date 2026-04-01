#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/multiverse"
UNITREE_DIR="${SCRIPT_DIR}/unitree"
INSTALL_DIR="${UNITREE_DIR}/install"
mkdir -p "$INSTALL_DIR"/bin

cd "$UNITREE_DIR"

# -------------------------------
# Helpers
# -------------------------------
log()  { echo -e "\n\033[1;32m[+] $*\033[0m"; }
warn() { echo -e "\n\033[1;33m[!] $*\033[0m"; }
die()  { echo -e "\n\033[1;31m[✗] $*\033[0m" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }

need_cmd python3
need_cmd wget
need_cmd tar

# 0. Create venv
if [[ ! -d "$VENV_DIR" ]]; then
    log "Creating venv: $VENV_DIR"
    mkdir -p "$(dirname "$VENV_DIR")"
    python3 -m venv "$VENV_DIR"
    # shellcheck disable=SC1090
    source "$VENV_DIR/bin/activate"
    python -m pip install -U pip catkin_pkg empy==3.3.4 lark-parser
else
    # shellcheck disable=SC1090
    source "$VENV_DIR/bin/activate"
fi

# -------------------------------
# 1. Install unitree_sdk2
# -------------------------------

cd "$UNITREE_DIR"

if [[ -f "$INSTALL_DIR/unitree_sdk2/lib/libunitree_sdk2.a" ]]; then
    log "unitree_sdk2 already installed, skipping build"
else
    # Install required packages
    log "Installing packages for unitree_sdk2..."
    sudo apt update
    sudo apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev

    # Update the unitree_sdk2 repository
    log "Updating unitree_sdk2 repository..."
    git submodule update --init unitree_sdk2

    # Build and install unitree_sdk2
    log "Building and installing unitree_sdk2..."
    cd unitree_sdk2
    mkdir -p build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR/unitree_sdk2"
    make -j$(nproc)
    make install
fi

# -------------------------------
# 2. Install unitree_rl_mjlab
# -------------------------------

cd "$UNITREE_DIR"

if pip show unitree_rl_mjlab >/dev/null 2>&1; then
    log "unitree_rl_mjlab already installed in pip, skipping installation"
else
    # Install required packages
    log "Installing packages for unitree_rl_mjlab..."
    sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev

    # Update the unitree_rl_mjlab repository
    log "Updating unitree_rl_mjlab repository..."
    git submodule update --init unitree_rl_mjlab

    # Install unitree_rl_mjlab
    log "Installing unitree_rl_mjlab..."
    cd unitree_rl_mjlab
    pip install -e .
fi

cd "$UNITREE_DIR"

if [[ -f "$UNITREE_DIR/unitree_rl_mjlab/simulate/build/unitree_mujoco" ]]; then
    log "unitree_mujoco already built, skipping build"
else
    log "Building unitree_mujoco..."

    # Install required packages
    log "Installing packages for unitree_rl_mjlab..."
    sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev

    # Update the unitree_rl_mjlab repository
    log "Updating unitree_rl_mjlab repository..."
    git submodule update --init unitree_rl_mjlab

    cd unitree_rl_mjlab

    # Fix bug missing #include <cstdint> in https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/simulate/src/joystick/jstest.cc
    if ! grep -q "#include <cstdint>" simulate/src/joystick/jstest.cc; then
        log "Fixing missing #include <cstdint> in jstest.cc..."
        sed -i '3i#include <cstdint>' simulate/src/joystick/jstest.cc
    fi

    # Build and install unitree_rl_mjlab
    log "Building and installing unitree_rl_mjlab..."
    cd simulate
    mkdir -p build
    cd build
    cmake .. -DCMAKE_PREFIX_PATH="${INSTALL_DIR}"/unitree_sdk2
    make -j$(nproc)

    ln -s "${UNITREE_DIR}"/unitree_rl_mjlab/simulate/build/unitree_mujoco "${UNITREE_DIR}"/install/bin/unitree_mujoco
fi

cd "$UNITREE_DIR"

if [[ -f "$UNITREE_DIR/unitree_rl_mjlab/deploy/robots/g1/build/g1_ctrl" ]]; then
    log "g1_ctrl already built, skipping build"
else
    log "Building g1_ctrl..."

     # Install required packages
    log "Installing packages for unitree_rl_mjlab..."
    sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev

    # Update the unitree_rl_mjlab repository
    log "Updating unitree_rl_mjlab repository..."
    git submodule update --init unitree_rl_mjlab

    cd unitree_rl_mjlab
    
    log "Overwriting FSMState.h..."
    cp -f "$SCRIPT_DIR/unitree/FSMState.h" deploy/include/FSM/FSMState.h

    log "Overwriting keyboard.h..."
    cp -f "$SCRIPT_DIR/unitree/keyboard.h" deploy/include/isaaclab/devices/keyboard/keyboard.h

    log "Overwriting CMakelists.txt..."
    sed -i '/add_executable.*g1_ctrl/ s/main.cpp/& ${PROJECT_SOURCE_DIR}\/..\/..\/include\/FSM\/Twist.c ${PROJECT_SOURCE_DIR}\/..\/..\/include\/FSM\/Vector3.c/' deploy/robots/g1/CMakeLists.txt

    log "Disabling joystick input in config.yaml..."
    sed -i 's/use_joystick: 1/use_joystick: 0/' simulate/config.yaml

    log "Copying Twist.c, Twist.h, Vector3.c, and Vector3.h to deploy/include/FSM..."
    cp -f "$SCRIPT_DIR/dds/Twist.c" deploy/include/FSM/Twist.c
    cp -f "$SCRIPT_DIR/dds/Twist.h" deploy/include/FSM/Twist.h
    cp -f "$SCRIPT_DIR/dds/Vector3.c" deploy/include/FSM/Vector3.c
    cp -f "$SCRIPT_DIR/dds/Vector3.h" deploy/include/FSM/Vector3.h

    # Build and install deployment on g1
    log "Building and installing deployment on g1..."
    cd deploy/robots/g1
    mkdir -p build
    cd build
    cmake .. \
        -DCMAKE_PREFIX_PATH="${INSTALL_DIR}"/unitree_sdk2 \
        -DCMAKE_CXX_FLAGS="-I${INSTALL_DIR}/unitree_sdk2/include -I${INSTALL_DIR}/unitree_sdk2/include/ddscxx" \
        -DCMAKE_C_FLAGS="-I${INSTALL_DIR}/unitree_sdk2/include" \
        -DCMAKE_EXE_LINKER_FLAGS="-L${INSTALL_DIR}/unitree_sdk2/lib -Wl,-rpath,${INSTALL_DIR}/unitree_sdk2/lib" \
        -DCMAKE_SHARED_LINKER_FLAGS="-L${INSTALL_DIR}/unitree_sdk2/lib -Wl,-rpath,${INSTALL_DIR}/unitree_sdk2/lib"
    make -j$(nproc)

    ln -s "${UNITREE_DIR}"/unitree_rl_mjlab/deploy/robots/g1/build/g1_ctrl "${UNITREE_DIR}"/install/bin/g1_ctrl
fi

log "Setup complete! To run the demo, execute: ${SCRIPT_DIR}/run.sh"