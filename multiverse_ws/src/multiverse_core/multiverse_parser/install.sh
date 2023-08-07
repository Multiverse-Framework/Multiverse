#!/usr/bin/env sh

# Check if BLENDER_BUILD_DIR exists
if [ -z "$BLENDER_BUILD_DIR" ]; then
    echo "BLENDER_BUILD_DIR is unset." >&2
    exit 1
else
    echo "BLENDER_BUILD_DIR is set to: $BLENDER_BUILD_DIR"
    
    SETUP_PATH="$CATKIN_DEVEL_DIR/setup.sh"

    PATH_TO_ADD=$BLENDER_BUILD_DIR/bin:$BLENDER_BUILD_DIR/bin/3.6/python/bin
    
    if ! echo "$PATH" | grep -q "$PATH_TO_ADD"; then
        export PATH=$PATH:$PATH_TO_ADD
    fi
    
    if ! echo "$PYTHONPATH" | grep -q "$PYTHONPATH_TO_ADD"; then
        export PYTHONPATH=$PYTHONPATH:$PYTHONPATH_TO_ADD
    fi
    
    PATH_TO_ADD="if ! echo \"\$PATH\" | grep -q \"$PATH_TO_ADD\"; then\n  export PATH=\$PATH:$PATH_TO_ADD\nfi"
    
    # Check if the line already exists in the file
    if ! grep -Fxq "$PATH_TO_ADD" "$SETUP_PATH"; then
        # Add the line to the file
        echo "$PATH_TO_ADD" >> $SETUP_PATH
    fi
fi