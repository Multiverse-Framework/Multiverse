#!/usr/bin/env sh

export PYTHONPATH=$PYTHONPATH:${PWD}/src/USD/build_scripts/install/lib/python
export PATH=$PATH:${PWD}/src/USD/build_scripts/install/bin
export PXR_PLUGINPATH_NAME=${PWD}/src/USD/build_scripts/install/share/usd/examples/plugin/usdOntology/resources