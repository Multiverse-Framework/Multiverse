#!/bin/bash

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

BUILD_CMAKE=true
BUILD_BLENDER=true
BUILD_USD=true
BUILD_MUJOCO=true
BUILD_ISAACLAB=true
BUILD_PYBIND11=true

while [ -n "$1" ]; do
    case "$1" in
    --excludes)
        echo -n "--excludes option passed"
        shift 1
        if [ "$#" -eq 0 ]; then
            echo ""
            BUILD_CMAKE=false
            BUILD_BLENDER=false
            BUILD_USD=false
            BUILD_MUJOCO=false
            BUILD_PYBIND11=false
            BUILD_ISAACLAB=false
        else
            echo -n ", with value:"
            for module in "$@"; do
                echo -n " $module"
                shift 1
                if [ "$module" = "cmake" ]; then
                    BUILD_CMAKE=false
                elif [ "$module" = "blender" ]; then
                    BUILD_BLENDER=false
                elif [ "$module" = "usd" ]; then
                    BUILD_USD=false
                elif [ "$module" = "mujoco" ]; then
                    BUILD_MUJOCO=false
                elif [ "$module" = "isaaclab" ]; then
                    BUILD_ISAACLAB=false
                elif [ "$module" = "pybind11" ]; then
                    BUILD_PYBIND11=false
                fi
            done
            echo ""
        fi
        ;;
    *)
        echo "Option $1 not recognized"
        shift 1
        ;;
    esac
done

UBUNTU_VERSION=$(lsb_release -rs)

PYTHON_EXECUTABLE=python3.10

if [ $BUILD_CMAKE = true ]; then
    echo "Building CMake..."

    # Build CMake

    CMAKE_BUILD_DIR=$BUILD_DIR/CMake

    if [ ! -d "$CMAKE_BUILD_DIR" ]; then
        # Create the folder if it doesn't exist
        mkdir -p "$CMAKE_BUILD_DIR"
        echo "Folder created: $CMAKE_BUILD_DIR"
    else
        echo "Folder already exists: $CMAKE_BUILD_DIR"
    fi

    CMAKE_TAR_FILE=cmake-4.0.0-linux-x86_64.tar.gz
    curl -L -o $EXT_DIR/$CMAKE_TAR_FILE https://github.com/Kitware/CMake/releases/download/v4.0.0/$CMAKE_TAR_FILE
    tar xf $EXT_DIR/$CMAKE_TAR_FILE -C $CMAKE_BUILD_DIR --strip-components=1
fi

CMAKE_EXECUTABLE=$BUILD_DIR/CMake/bin/cmake
if [ ! -f "$CMAKE_EXECUTABLE" ]; then
    CMAKE_EXECUTABLE=$(which cmake)
fi
if [ ! -f "$CMAKE_EXECUTABLE" ]; then
    echo "cmake does not exist."
    exit 1
fi
CMAKE_DIR=$(dirname $CMAKE_EXECUTABLE)

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

    for virtualenvwrapper in $(which virtualenvwrapper.sh) /usr/share/virtualenvwrapper/virtualenvwrapper.sh /usr/local/bin/virtualenvwrapper.sh /home/$USER/.local/bin/virtualenvwrapper.sh; do
        if [ -f $virtualenvwrapper ]; then
            break
        fi
    done
    if [ ! -f $virtualenvwrapper ]; then
        echo "virtualenvwrapper.sh not found"
    else
        . $virtualenvwrapper
        mkvirtualenv --system-site-packages multiverse -p $PYTHON_EXECUTABLE
        $PYTHON_EXECUTABLE -m pip install -U pip build setuptools packaging distro
        $PYTHON_EXECUTABLE -m pip install pyside6 pyopengl
        PATH=$CMAKE_DIR:$PATH $PYTHON_EXECUTABLE $USD_EXT_DIR/build_scripts/build_usd.py $USD_BUILD_DIR
        ln -sf $USD_BUILD_DIR/bin/usdview $BIN_DIR
        ln -sf $USD_BUILD_DIR/bin/usdGenSchema $BIN_DIR
        ln -sf $USD_BUILD_DIR/bin/usdcat $BIN_DIR
    fi
fi

if [ $BUILD_BLENDER = true ]; then
    echo "Building Blender..."

    FROM_SRC=false
    BLENDER_BUILD_DIR=$BUILD_DIR/blender

    if [ ! -d "$BLENDER_BUILD_DIR" ]; then
        # Create the folder if it doesn't exist
        mkdir -p "$BLENDER_BUILD_DIR"
        echo "Folder created: $BLENDER_BUILD_DIR"
    else
        echo "Folder already exists: $BLENDER_BUILD_DIR"
    fi

    if [ $FROM_SRC = true ]; then
        # Build blender

        BLENDER_EXT_DIR=$EXT_DIR/blender-git

        git submodule update --init $BLENDER_EXT_DIR/blender

        (
            cd $BLENDER_EXT_DIR/blender
            make update && ./build_files/utils/make_update.py --use-linux-libraries
        )
        (
            cd $BLENDER_BUILD_DIR
            $CMAKE_EXECUTABLE -S ../../external/blender-git/blender -B . -Wno-deprecated -Wno-dev && make -j$(nproc) && make install
        )
    else
        # Download blender

        BLENDER_TAR_FILE=blender-4.4.0-linux-x64.tar.xz
        curl -o $EXT_DIR/$BLENDER_TAR_FILE https://download.blender.org/release/Blender4.4/$BLENDER_TAR_FILE
        tar xf $EXT_DIR/$BLENDER_TAR_FILE -C $BLENDER_BUILD_DIR --strip-components=1
    fi

    (
        cd $BLENDER_BUILD_DIR/4.4/python/bin
        ./python3.11 -m pip install --upgrade pip build --no-warn-script-location
        ./python3.11 -m pip install bpy Pillow --no-warn-script-location
    ) # For blender
    ln -sf $BLENDER_BUILD_DIR/blender $BIN_DIR
    ln -sf $BLENDER_BUILD_DIR/4.4/python/bin/python3.11 $BIN_DIR
fi

if [ $BUILD_MUJOCO = true ]; then
    echo "Building MuJoCo..."

    FROM_SRC=false
    MUJOCO_BUILD_DIR=$BUILD_DIR/mujoco

    if [ ! -d "$MUJOCO_BUILD_DIR" ]; then
        # Create the folder if it doesn't exist
        mkdir -p "$MUJOCO_BUILD_DIR"
        echo "Folder created: $MUJOCO_BUILD_DIR"
    else
        echo "Folder already exists: $MUJOCO_BUILD_DIR"
    fi

    if [ $FROM_SRC = false ]; then
        # Build MuJoCo

        MUJOCO_EXT_DIR=$EXT_DIR/mujoco

        git submodule update --init $MUJOCO_EXT_DIR
        MUJOCO_PLUGIN_DIR=$MUJOCO_BUILD_DIR/bin/mujoco_plugin
        mkdir -p $MUJOCO_PLUGIN_DIR
        (
            cd $MUJOCO_BUILD_DIR
            $CMAKE_EXECUTABLE $MUJOCO_EXT_DIR -DCMAKE_INSTALL_PREFIX=$MUJOCO_BUILD_DIR -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -Wno-deprecated -Wno-dev && $CMAKE_EXECUTABLE --build . && $CMAKE_EXECUTABLE --install .
        )
    else
        # Download MuJoCo

        MUJOCO_TAR_FILE=mujoco-3.3.0-linux-x86_64.tar.gz
        curl -sL https://github.com/google-deepmind/mujoco/releases/download/3.3.0/$MUJOCO_TAR_FILE | tar zx -C $MUJOCO_BUILD_DIR --strip-components=1
    fi

    ln -sf $MUJOCO_BUILD_DIR/bin/simulate $BIN_DIR
fi

UBUNTU_VERSION=$(lsb_release -rs)
if [ $BUILD_ISAACLAB = true ] && [ $UBUNTU_VERSION != "20.04" ]; then
    echo "Building IsaacLab..."

    # Download IsaacLab

    ISAACLAB_EXT_DIR=$EXT_DIR/IsaacLab

    git submodule update --init $ISAACLAB_EXT_DIR

    for virtualenvwrapper in $(which virtualenvwrapper.sh) /usr/share/virtualenvwrapper/virtualenvwrapper.sh /usr/local/bin/virtualenvwrapper.sh /home/$USER/.local/bin/virtualenvwrapper.sh; do
        if [ -f $virtualenvwrapper ]; then
            break
        fi
    done
    if [ ! -f $virtualenvwrapper ]; then
        echo "virtualenvwrapper.sh not found"
    else
        . $virtualenvwrapper
        mkvirtualenv --system-site-packages multiverse -p $PYTHON_EXECUTABLE
        $PYTHON_EXECUTABLE -m pip install -U pip build setuptools packaging distro
        (
            cd $ISAACLAB_EXT_DIR
            ./isaaclab.sh --install
        )
    fi
fi

if [ $BUILD_PYBIND11 = true ]; then
    echo "Building pybind11..."

    # Build pybind11

    PYBIND11_BUILD_DIR=$BUILD_DIR/pybind11
    PYBIND11_EXT_DIR=$EXT_DIR/pybind11

    git submodule update --init $PYBIND11_EXT_DIR

    if [ ! -d "$PYBIND11_BUILD_DIR" ]; then
        # Create the folder if it doesn't exist
        mkdir -p "$PYBIND11_BUILD_DIR"
        echo "Folder created: $PYBIND11_BUILD_DIR"
    else
        echo "Folder already exists: $PYBIND11_BUILD_DIR"
    fi

    (
        cd $PYBIND11_BUILD_DIR
        $CMAKE_EXECUTABLE $PYBIND11_EXT_DIR -DCMAKE_INSTALL_PREFIX=$PYBIND11_BUILD_DIR -DDOWNLOAD_CATCH=ON -Wno-deprecated -Wno-dev && $CMAKE_EXECUTABLE --build . && sudo $CMAKE_EXECUTABLE --install .
    )
fi

RELOAD=false

if ! echo "$PATH" | grep -q "$BIN_DIR"; then
    PATH_TO_ADD="export PATH=$PATH:$BIN_DIR"
    echo "$PATH_TO_ADD" >>~/.bashrc
    echo "Add $PATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

if ! echo "$PYTHONPATH" | grep -q "$USD_BUILD_DIR/lib/python"; then
    PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH:$USD_BUILD_DIR/lib/python"
    echo "$PYTHONPATH_TO_ADD" >>~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

cd $CURRENT_DIR

if [ "$RELOAD" = true ]; then
    exec bash # Reload ~/.bashrc
    rosdep update
fi
