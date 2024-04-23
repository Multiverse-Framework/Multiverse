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

(cd $BLENDER_EXT_DIR && make update)

$BLENDER_EXT_DIR/blender/build_files/utils/make_update.py --use-linux-libraries

FILE2=$BLENDER_EXT_DIR/blender/source/blender/blenfont/intern/blf_glyph.cc
if [ -f "$FILE2" ]; then
    sed -i \
        -e 's/g->bitmap\[i\] = blf_glyph_gamma(glyph->bitmap.buffer\[i\] \* scale);/g->bitmap[i] = blf_glyph_gamma(char(glyph->bitmap.buffer[i] * scale));/' \
        "$FILE2"
else
    echo "Error: File does not exist: $FILE2"
fi

FILE3=$BLENDER_EXT_DIR/blender/source/blender/blenfont/intern/blf_font.cc
if [ -f "$FILE3" ]; then
    sed -i \
        -e 's/metrics->descender = metrics->ascender - metrics->units_per_EM;/metrics->descender = short(metrics->ascender - metrics->units_per_EM);/' \
        "$FILE3"
else
    echo "Error: File does not exist: $FILE3"
fi

FILE4=$BLENDER_EXT_DIR/blender/source/blender/editors/space_view3d/view3d_navigate_walk.cc
if [ -f "$FILE4" ]; then
    sed -i \
        -e 's/direction += 1;/direction = short(direction + 1);/' \
        -e 's/direction -= 1;/direction = short(direction - 1);/' \
        "$FILE4"
else
    echo "Error: File does not exist: $FILE4"
fi

FILE5=$BLENDER_EXT_DIR/blender/source/blender/blenkernel/intern/bvhutils.cc
if [ -f "$FILE5" ]; then
    sed -i \
        -e 's/ = const_cast<const BMLoop \*(\*)\[3\]>(em->looptris);/; memcpy(corner_tris, em->looptris, sizeof(*em->looptris));/' \
        "$FILE5"
else
    echo "Error: File does not exist: $FILE5"
fi

FILE6=$BLENDER_EXT_DIR/blender/source/blender/blenkernel/intern/editmesh_tangent.cc
if [ -f "$FILE6" ]; then
    sed -i \
        -e 's/mesh2tangent->looptris = const_cast<const BMLoop \*(\*)\[3\]>(em->looptris);/memcpy(mesh2tangent->looptris, em->looptris, sizeof(*em->looptris));/' \
        "$FILE6"
else
    echo "Error: File does not exist: $FILE6"
fi

FILE7=$BLENDER_EXT_DIR/blender/source/blender/blenkernel/intern/editmesh_bvh.cc
if [ -f "$FILE7" ]; then
    sed -i \
        -e 's/bmcb_data.looptris = const_cast<const BMLoop \*(\*)\[3\]>(bmtree->looptris);/const BMLoop *tmp_looptris[3] = {nullptr, nullptr, nullptr}; bmcb_data.looptris = \&tmp_looptris; memcpy(bmcb_data.looptris, bmtree->looptris, sizeof(*bmtree->looptris));/' \
        -e 's/bmcb_data->looptris = const_cast<const BMLoop \*(\*)\[3\]>(bmtree->looptris);/const BMLoop *tmp_looptris[3] = {nullptr, nullptr, nullptr}; bmcb_data->looptris = \&tmp_looptris; memcpy(bmcb_data->looptris, bmtree->looptris, sizeof(*bmtree->looptris));/' \
        "$FILE7"
else
    echo "Error: File does not exist: $FILE7"
fi

(cd $BLENDER_BUILD_DIR && cmake -S ../../external/blender-git/blender -B . -Wno-deprecated -Wno-dev && make -j$(nproc) && make install)
(cd $BLENDER_BUILD_DIR/bin/4.1/python/bin;
    ./python3.11 -m pip install --upgrade pip build --no-warn-script-location;
    ./python3.11 -m pip install bpy Pillow --no-warn-script-location) # For blender
ln -sf $BLENDER_BUILD_DIR/bin/blender $BIN_DIR
ln -sf $BLENDER_BUILD_DIR/bin/4.1/python/bin/python3.11 $BIN_DIR

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
ln -sf $USD_BUILD_DIR/bin/usdcat $BIN_DIR

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

(cd $MUJOCO_BUILD_DIR && cmake $MUJOCO_EXT_DIR -DCMAKE_INSTALL_PREFIX=$MUJOCO_BUILD_DIR -Wno-deprecated -Wno-dev && cmake --build . && cmake --install .)
ln -sf $MUJOCO_BUILD_DIR/bin/simulate $BIN_DIR

RELOAD=false

if ! echo "$PATH" | grep -q "$BIN_DIR"; then
    PATH_TO_ADD="export PATH=$PATH:$BIN_DIR"
    echo "$PATH_TO_ADD" >> ~/.bashrc
    echo "Add $PATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

if ! echo "$PYTHONPATH" | grep -q "$USD_BUILD_DIR/lib/python"; then
    PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH:$USD_BUILD_DIR/lib/python"
    echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

cd $CURRENT_DIR

if [ "$RELOAD" = true ]; then
    exec bash # Reload ~/.bashrc
    rosdep update
fi
