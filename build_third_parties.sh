#!/usr/bin/env sh

CURRENT_DIR=$PWD

git submodule update --init

cd $(dirname $0)

MULTIVERSE_DIR=$PWD/multiverse

BIN_DIR=$MULTIVERSE_DIR/bin
if [ ! -d "$BIN_DIR" ]; then
    # Create the folder if it doesn't exist
    mkdir -p $BIN_DIR
fi

EXT_DIR=$MULTIVERSE_DIR/external

BUILD_DIR=$MULTIVERSE_DIR/build

SRC_DIR=$MULTIVERSE_DIR/src

INCLUDE_DIR=$MULTIVERSE_DIR/include

# Build blender

BLENDER_BUILD_DIR=$BUILD_DIR/blender
BLENDER_EXT_DIR=$EXT_DIR/blender-git
if [ ! -d "$BLENDER_BUILD_DIR" ]; then
    # Create the folder if it doesn't exist
    mkdir -p "$BLENDER_BUILD_DIR"
    echo "Folder created: $BLENDER_BUILD_DIR"
else
    echo "Folder already exists: $BLENDER_BUILD_DIR"
fi

if [ ! -d "$BLENDER_EXT_DIR/lib" ]; then
    (cd $BLENDER_EXT_DIR; mkdir lib; cd lib; svn checkout https://svn.blender.org/svnroot/bf-blender/trunk/lib/linux_x86_64_glibc_228)
    (cd $BLENDER_EXT_DIR/blender; make update)
fi

(cd $BLENDER_EXT_DIR/blender && make BUILD_DIR=../../../build/blender)
(cd $BLENDER_BUILD_DIR/bin/3.6/python/bin;
./python3.10 -m pip install --upgrade pip build;
./python3.10 -m pip install mujoco numpy-stl;
./python3.10 -m pip install bpy)
ln -sf $BLENDER_BUILD_DIR/bin/blender $BIN_DIR
ln -sf $BLENDER_BUILD_DIR/bin/3.6/python/bin/python3.10 $BIN_DIR

# Build USD

USD_BUILD_DIR=$BUILD_DIR/USD
USD_EXT_DIR=$EXT_DIR/USD
if [ ! -d "$USD_BUILD_DIR" ]; then
    # Create the folder if it doesn't exist
    mkdir -p "$USD_BUILD_DIR"
    echo "Folder created: $USD_BUILD_DIR"
else
    echo "Folder already exists: $USD_BUILD_DIR"
fi

python3 $USD_EXT_DIR/build_scripts/build_usd.py $USD_BUILD_DIR
ln -sf $USD_BUILD_DIR/bin/usdview $BIN_DIR
ln -sf $USD_BUILD_DIR/bin/usdGenSchema $BIN_DIR

# Build MuJoCo

MUJOCO_BUILD_DIR=$BUILD_DIR/mujoco
MUJOCO_EXT_DIR=$EXT_DIR/mujoco
if [ ! -d "$MUJOCO_BUILD_DIR" ]; then
    # Create the folder if it doesn't exist
    mkdir -p "$MUJOCO_BUILD_DIR"
    echo "Folder created: $MUJOCO_BUILD_DIR"
else
    echo "Folder already exists: $MUJOCO_BUILD_DIR"
fi

cmake -S $MUJOCO_EXT_DIR -B $MUJOCO_BUILD_DIR
(cd $MUJOCO_BUILD_DIR && make)
ln -sf $MUJOCO_BUILD_DIR/bin/simulate $BIN_DIR

RELOAD=false

PATH_TO_ADD="export PATH=$PATH:$BIN_DIR"
if ! grep -Fxq ":$BIN_DIR" ~/.bashrc; then
    echo "$PATH_TO_ADD" >> ~/.bashrc
    echo "Add $PATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH:$USD_BUILD_DIR/lib/python"
if ! grep -Fxq ":$USD_BUILD_DIR/lib/python" ~/.bashrc; then
    echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

cd $CURRENT_DIR

if [ "$RELOAD" = true ]; then
    exec bash # Reload ~/.bashrc
fi