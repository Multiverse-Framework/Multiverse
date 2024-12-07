#!/usr/bin/env sh

# Check if MUJOCO_SRC_DIR or MUJOCO_BUILD_DIR exists
if [ -z "$MUJOCO_SRC_DIR" ] || [ -z "$MUJOCO_BUILD_DIR" ]; then
    echo "MUJOCO_SRC_DIR or MUJOCO_BUILD_DIR is unset." >&2
    exit 1
else
    echo "MUJOCO_SRC_DIR is set to: $MUJOCO_SRC_DIR"
    echo "MUJOCO_BUILD_DIR is set to: $MUJOCO_BUILD_DIR"
    
    cp -r plugin/multiverse_connector $MUJOCO_SRC_DIR/plugin
    cp -f ../../../../../../src/multiverse_client/include/multiverse_client.h $MUJOCO_SRC_DIR/plugin/multiverse_connector
    cp -f ../../../../../../src/multiverse_client/include/multiverse_client_json.h $MUJOCO_SRC_DIR/plugin/multiverse_connector
    cp -f ../../../../../../src/multiverse_client/src/multiverse_client.cpp $MUJOCO_SRC_DIR/plugin/multiverse_connector
    cp -f ../../../../../../src/multiverse_client/src/multiverse_client_json.cpp $MUJOCO_SRC_DIR/plugin/multiverse_connector
    
    # Specify the file path
    MUJOCO_CMAKE_PATH="$MUJOCO_SRC_DIR/CMakeLists.txt"
    
    # Specify the line to add
    LINE_TO_ADD="add_subdirectory(plugin/multiverse_connector)"
    
    # Check if the line already exists in the file
    if ! grep -Fxq "$LINE_TO_ADD" "$MUJOCO_CMAKE_PATH"; then
        # Add the line to the file
        echo "\n$LINE_TO_ADD" >> $MUJOCO_CMAKE_PATH
    fi
    
    MUJOCO_PLUGIN_DIR=$MUJOCO_BUILD_DIR/bin/mujoco_plugin
    mkdir -p $MUJOCO_PLUGIN_DIR
    (cd $MUJOCO_BUILD_DIR && cmake $MUJOCO_SRC_DIR -DCMAKE_INSTALL_PREFIX=$MUJOCO_BUILD_DIR -Wno-deprecated -Wno-dev && cmake --build . && cmake --install . && cp $MUJOCO_BUILD_DIR/lib/libmultiverse_connector.so $MUJOCO_PLUGIN_DIR)
fi