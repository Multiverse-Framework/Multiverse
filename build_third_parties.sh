#!/usr/bin/env sh

CURRENT_DIR=$PWD

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

BUILD_BLENDER=true
BUILD_USD=true
BUILD_MUJOCO=true

while [ -n "$1" ]; do
    case "$1" in
        --excludes) echo -n "--excludes option passed"
            shift 1
            if [ "$#" -eq 0 ]; then
                echo ""
                BUILD_BLENDER=false
                BUILD_USD=false
                BUILD_MUJOCO=false
            else
                echo -n ", with value:"
                for module in "$@"; do
                    echo -n " $module"
                    shift 1
                    if [ "$module" = "blender" ]; then
                        BUILD_BLENDER=OFF
                    elif [ "$module" = "usd" ]; then
                        BUILD_BLENDER=OFF
                        BUILD_USD=OFF
                    elif [ "$module" = "mujoco" ]; then
                        BUILD_MUJOCO=OFF
                    fi
                done
                echo ""
            fi
        ;;
        *) echo "Option $1 not recognized"
            shift 1
        ;;
    esac
done

if [ $BUILD_BLENDER = true ]; then
    echo "Building Blender..."

    # Build blender

    BLENDER_BUILD_DIR=$BUILD_DIR/blender
    BLENDER_EXT_DIR=$EXT_DIR/blender-git

    git submodule update --init $BLENDER_EXT_DIR/blender

    if [ ! -d "$BLENDER_BUILD_DIR" ]; then
        # Create the folder if it doesn't exist
        mkdir -p "$BLENDER_BUILD_DIR"
        echo "Folder created: $BLENDER_BUILD_DIR"
    else
        echo "Folder already exists: $BLENDER_BUILD_DIR"
    fi

    (cd $BLENDER_EXT_DIR/blender && make update && ./build_files/utils/make_update.py --use-linux-libraries)
    (cd $BLENDER_BUILD_DIR && cmake -S ../../external/blender-git/blender -B . -Wno-deprecated -Wno-dev && make -j$(nproc) && make install)
    (cd $BLENDER_BUILD_DIR/bin/4.1/python/bin;
        ./python3.11 -m pip install --upgrade pip build --no-warn-script-location;
        ./python3.11 -m pip install bpy Pillow --no-warn-script-location) # For blender
    ln -sf $BLENDER_BUILD_DIR/bin/blender $BIN_DIR
    ln -sf $BLENDER_BUILD_DIR/bin/4.1/python/bin/python3.11 $BIN_DIR
fi

if [ $BUILD_USD = true ]; then
    echo "Building USD..."

    # Build USD

    USD_BUILD_DIR=$BUILD_DIR/USD
    USD_EXT_DIR=$EXT_DIR/USD

    git submodule update --init $USD_EXT_DIR

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
fi

if [ $BUILD_MUJOCO = true ]; then
    echo "Building MuJoCo..."

    # Build MuJoCo

    MUJOCO_BUILD_DIR=$BUILD_DIR/mujoco
    MUJOCO_EXT_DIR=$EXT_DIR/mujoco

    git submodule update --init $MUJOCO_EXT_DIR

    if [ ! -d "$MUJOCO_BUILD_DIR" ]; then
        # Create the folder if it doesn't exist
        mkdir -p "$MUJOCO_BUILD_DIR"
        echo "Folder created: $MUJOCO_BUILD_DIR"
    else
        echo "Folder already exists: $MUJOCO_BUILD_DIR"
    fi

    (cd $MUJOCO_BUILD_DIR && cmake $MUJOCO_EXT_DIR -DCMAKE_INSTALL_PREFIX=$MUJOCO_BUILD_DIR -Wno-deprecated -Wno-dev && cmake --build . && cmake --install .)
    ln -sf $MUJOCO_BUILD_DIR/bin/simulate $BIN_DIR
fi

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
