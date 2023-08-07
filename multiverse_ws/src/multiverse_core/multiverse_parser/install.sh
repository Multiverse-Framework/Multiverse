#!/usr/bin/env sh

# Check if BLENDER_BUILD_DIR exists
if [ -z "$BLENDER_BUILD_DIR" ]; then
    echo "BLENDER_BUILD_DIR is unset." >&2
    exit 1
else
    echo "BLENDER_BUILD_DIR is set to: $BLENDER_BUILD_DIR"
    
    SETUP_PATH="$CATKIN_DEVEL_DIR/setup.sh"
    
    if ! echo "$PATH" | grep -q "$BLENDER_BUILD_DIR/build_linux/bin"; then
        export PATH=$PATH:$BLENDER_BUILD_DIR/build_linux/bin
    fi
    
    if ! echo "$PYTHONPATH" | grep -q "$BLENDER_BUILD_DIR/build_linux_bpy/bin"; then
        export PYTHONPATH=$PYTHONPATH:$BLENDER_BUILD_DIR/build_linux_bpy/bin
    fi
    
    PATH_TO_ADD="if ! echo \"\$PATH\" | grep -q \"$BLENDER_BUILD_DIR/build_linux/bin\"; then\n  export PATH=\$PATH:$BLENDER_BUILD_DIR/build_linux/bin\nfi"
    
    # Check if the line already exists in the file
    if ! grep -Fxq "$PATH_TO_ADD" "$SETUP_PATH"; then
        # Add the line to the file
        echo "$PATH_TO_ADD" >> $SETUP_PATH
    fi
    
    PYTHONPATH_TO_ADD="if ! echo \"\$PYTHONPATH\" | grep -q \"$BLENDER_BUILD_DIR/build_linux_bpy/bin\"; then\n  export PYTHONPATH=\$PYTHONPATH:$BLENDER_BUILD_DIR/build_linux_bpy/bin\nfi"
    # Check if the line already exists in the file
    if ! grep -Fxq "$PYTHONPATH_TO_ADD" "$SETUP_PATH"; then
        # Add the line to the file
        echo "$PYTHONPATH_TO_ADD" >> $SETUP_PATH
    fi
fi