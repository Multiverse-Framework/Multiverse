#!/bin/bash

ISAAC_SIM_DIR=/home/${USER}/.local/share/ov/pkg/isaac-sim-4.2.0

if [ ! -d "$ISAAC_SIM_DIR" ]; then
    echo "Isaac Sim is not installed. Please install Isaac Sim first."
    exit 0
fi

MULTIVERSE_PATH=$(dirname "$(dirname "$0")")

USD_PATH=$1
CONFIG_PARAMS=$2
MULTIVERSE_PARAMS=$3

$ISAAC_SIM_DIR/python.sh ${MULTIVERSE_PATH}/modules/multiverse_connectors/src/isaac_sim/src/isaac_sim/multiverse_isaac_sim_connector.py  --usd_path=${USD_PATH} --config_params=${CONFIG_PARAMS} --multiverse_params=${MULTIVERSE_PARAMS}