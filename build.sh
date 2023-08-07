#!/usr/bin/env sh

SRC_DIR="$(dirname $0)/src"

# Specify the folder path to create
BUILD_DIR="$(dirname $0)/build"

# Build USD

# Check if the folder already exists
USD_BUILD_DIR="$BUILD_DIR/USD"
USD_SRC_DIR="$SRC_DIR/USD"
if [ ! -d "$USD_BUILD_DIR" ]; then
    # Create the folder if it doesn't exist
    mkdir -p "$USD_BUILD_DIR"
    echo "Folder created: $USD_BUILD_DIR"
else
    echo "Folder already exists: $USD_BUILD_DIR"
fi

(python3 $USD_SRC_DIR/build_scripts/build_usd.py $USD_BUILD_DIR)

# Build MuJoCo

# Check if the folder already exists
MUJOCO_BUILD_DIR="$BUILD_DIR/mujoco"
MUJOCO_SRC_DIR="$SRC_DIR/mujoco"
if [ ! -d "$MUJOCO_BUILD_DIR" ]; then
    # Create the folder if it doesn't exist
    mkdir -p "$MUJOCO_BUILD_DIR"
    echo "Folder created: $MUJOCO_BUILD_DIR"
else
    echo "Folder already exists: $MUJOCO_BUILD_DIR"
fi

cmake -S $MUJOCO_SRC_DIR -B $MUJOCO_BUILD_DIR
(cd $MUJOCO_BUILD_DIR && make)

# Build blender

if [ ! -d "$SRC_DIR/blender-git/lib" ]; then
    (cd $SRC_DIR/blender-git; mkdir lib; cd lib; svn checkout https://svn.blender.org/svnroot/bf-blender/trunk/lib/linux_x86_64_glibc_228)
    (cd $SRC_DIR/blender-git/blender; make update)
fi

(cd $SRC_DIR/blender-git/blender && make BUILD_DIR=../../../build/blender/build_linux)
(cd $SRC_DIR/blender-git/blender && make bpy BUILD_DIR=../../../build/blender/build_linux_bpy)