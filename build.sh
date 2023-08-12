#!/usr/bin/env sh

git submodule update --init

EXT_DIR="$(dirname $0)/multiverse/ext"

# Specify the folder path to create
BUILD_DIR="$(dirname $0)/multiverse/build"

# Build USD

# # Check if the folder already exists
# USD_BUILD_DIR=$BUILD_DIR/USD
# USD_EXT_DIR=$EXT_DIR/USD
# if [ ! -d "$USD_BUILD_DIR" ]; then
#     # Create the folder if it doesn't exist
#     mkdir -p "$USD_BUILD_DIR"
#     echo "Folder created: $USD_BUILD_DIR"
# else
#     echo "Folder already exists: $USD_BUILD_DIR"
# fi

# (python3 $USD_EXT_DIR/build_scripts/build_usd.py $USD_BUILD_DIR)

# # Build MuJoCo

# # Check if the folder already exists
# MUJOCO_BUILD_DIR=$BUILD_DIR/mujoco
# MUJOCO_EXT_DIR=$EXT_DIR/mujoco
# if [ ! -d "$MUJOCO_BUILD_DIR" ]; then
#     # Create the folder if it doesn't exist
#     mkdir -p "$MUJOCO_BUILD_DIR"
#     echo "Folder created: $MUJOCO_BUILD_DIR"
# else
#     echo "Folder already exists: $MUJOCO_BUILD_DIR"
# fi

# cmake -S $MUJOCO_EXT_DIR -B $MUJOCO_BUILD_DIR
# (cd $MUJOCO_BUILD_DIR && make)

# # Build blender

# # Check if the folder already exists
# BLENDER_BUILD_DIR=$BUILD_DIR/blender
# BLENDER_EXT_DIR=$EXT_DIR/blender-git
# if [ ! -d "$BLENDER_BUILD_DIR" ]; then
#     # Create the folder if it doesn't exist
#     mkdir -p "$BLENDER_BUILD_DIR"
#     echo "Folder created: $BLENDER_BUILD_DIR"
# else
#     echo "Folder already exists: $BLENDER_BUILD_DIR"
# fi

# if [ ! -d "$BLENDER_EXT_DIR/lib" ]; then
#     (cd $BLENDER_EXT_DIR; mkdir lib; cd lib; svn checkout https://svn.blender.org/svnroot/bf-blender/trunk/lib/linux_x86_64_glibc_228)
#     (cd $BLENDER_EXT_DIR/blender; make update)
# fi

# (cd $BLENDER_EXT_DIR/blender && make BUILD_DIR=../../../build/blender)
# (cd $BLENDER_BUILD_DIR/bin/3.6/python/bin;
# ./python3.10 -m pip install --upgrade pip;
# ./python3.10 -m pip install mujoco numpy-stl;
# ./python3.10 -m pip install bpy)

# Initialize rosdep

rosdep init