#!/usr/bin/env sh

# Check if MUJOCO_SRC_DIR or MUJOCO_BUILD_DIR exists
if [ -z "$MUJOCO_SRC_DIR" ] || [ -z "$MUJOCO_BUILD_DIR" ]; then
    echo "MUJOCO_SRC_DIR or MUJOCO_BUILD_DIR is unset." >&2
    exit 1
else
    echo "MUJOCO_SRC_DIR is set to: $MUJOCO_SRC_DIR"
    echo "MUJOCO_BUILD_DIR is set to: $MUJOCO_BUILD_DIR"

    MULTIVERSE_DIR="$PWD/../../../../../.."

    CMAKE_EXECUTABLE=$MULTIVERSE_DIR/build/CMake/bin/cmake
    if [ ! -f "$CMAKE_EXECUTABLE" ]; then
        CMAKE_EXECUTABLE=$(which cmake)
    fi
    if [ ! -f "$CMAKE_EXECUTABLE" ]; then
        echo "cmake does not exist."
        exit 1
    fi

    if [ -d "$MUJOCO_SRC_DIR" ]; then
        cp -r plugin/multiverse_connector $MUJOCO_SRC_DIR/plugin
        ln -sf $MULTIVERSE_DIR/src/multiverse_client/include/multiverse_client.h $MUJOCO_SRC_DIR/plugin/multiverse_connector
        ln -sf $MULTIVERSE_DIR/src/multiverse_client/include/multiverse_client_json.h $MUJOCO_SRC_DIR/plugin/multiverse_connector
        ln -sf $MULTIVERSE_DIR/lib/libstdc++/libmultiverse_client_json.so $MUJOCO_SRC_DIR/plugin/multiverse_connector
        ln -sf $MULTIVERSE_DIR/lib/libstdc++/libmultiverse_client.a $MUJOCO_SRC_DIR/plugin/multiverse_connector

        # Specify the file path
        MUJOCO_CMAKE_PATH="$MUJOCO_SRC_DIR/CMakeLists.txt"

        # Specify the line to add
        LINE_TO_ADD="add_subdirectory(plugin/multiverse_connector)"

        # Check if the line already exists in the file
        if ! grep -Fxq "$LINE_TO_ADD" "$MUJOCO_CMAKE_PATH"; then
            # Add the line to the file
            echo "\n$LINE_TO_ADD" >>$MUJOCO_CMAKE_PATH
        fi

        MUJOCO_PLUGIN_DIR=$MUJOCO_BUILD_DIR/bin/mujoco_plugin
        mkdir -p $MUJOCO_PLUGIN_DIR
        (
            cd $MUJOCO_BUILD_DIR
            echo "Building MuJoCo plugin using CMake: $CMAKE_EXECUTABLE"
            $CMAKE_EXECUTABLE $MUJOCO_SRC_DIR -DCMAKE_INSTALL_PREFIX=$MUJOCO_BUILD_DIR -DCMAKE_POLICY_VERSION_MINIMUM="3.5" -Wno-deprecated -Wno-dev && $CMAKE_EXECUTABLE --build . && $CMAKE_EXECUTABLE --install .
            cp $MUJOCO_BUILD_DIR/lib/libmultiverse_connector.so $MUJOCO_PLUGIN_DIR
        )
    else
        mkdir -p $MUJOCO_BUILD_DIR
        MUJOCO_TAR_FILE=mujoco-3.3.0-linux-x86_64.tar.gz
        curl -sL https://github.com/google-deepmind/mujoco/releases/download/3.3.0/$MUJOCO_TAR_FILE | tar zx -C $MUJOCO_BUILD_DIR --strip-components=1
        cp -r mujoco_plugin $MUJOCO_BUILD_DIR/bin
    fi
fi
