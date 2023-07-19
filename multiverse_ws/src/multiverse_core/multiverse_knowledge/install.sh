#!/usr/bin/env sh

# Check if USD_SRC_DIR or USD_BUILD_DIR exists
if [ -z "$USD_SRC_DIR" ] || [ -z "$USD_BUILD_DIR" ]; then
    echo "USD_SRC_DIR or USD_BUILD_DIR is unset." >&2
    exit 1
else
    echo "USD_SRC_DIR is set to: $USD_SRC_DIR"
    echo "USD_BUILD_DIR is set to: $USD_BUILD_DIR"
    
    SETUP_PATH="$CATKIN_DEVEL_DIR/setup.sh"
    
    if ! echo "$PATH" | grep -q "$USD_BUILD_DIR/bin"; then
        export PATH=$PATH:$USD_BUILD_DIR/bin
    fi
    
    if ! echo "$PYTHONPATH" | grep -q "$USD_BUILD_DIR/lib/python"; then
        export PYTHONPATH=$PYTHONPATH:$USD_BUILD_DIR/lib/python
    fi
    
    cp -r USD/plugin $USD_SRC_DIR
    
    # Loop through each folder in the directory
    for USD_PLUGIN in $USD_SRC_DIR/plugin/*; do
        if [ -d "$USD_PLUGIN" ]; then
            # Execute your command within each folder
            (cd "$USD_PLUGIN" && usdGenSchema schema.usda)
        fi
    done
    
    # Specify the file path
    USD_CMAKE_PATH="$USD_SRC_DIR/CMakeLists.txt"
    
    # Specify the line to add
    LINE_TO_ADD="add_subdirectory(plugin)"
    
    # Check if the line already exists in the file
    if ! grep -Fxq "$LINE_TO_ADD" "$USD_CMAKE_PATH"; then
        # Add the line to the file
        echo "\n$LINE_TO_ADD" >> $USD_CMAKE_PATH
    fi
    
    python3 $USD_SRC_DIR/build_scripts/build_usd.py $USD_BUILD_DIR
    
    PATH_TO_ADD="if ! echo \"\$PATH\" | grep -q \"$USD_BUILD_DIR/bin\"; then\n  export PATH=\$PATH:$USD_BUILD_DIR/bin\nfi"
    
    # Check if the line already exists in the file
    if ! grep -Fxq "$PATH_TO_ADD" "$SETUP_PATH"; then
        # Add the line to the file
        echo "$PATH_TO_ADD" >> $SETUP_PATH
    fi
    
    PYTHONPATH_TO_ADD="if ! echo \"\$PYTHONPATH\" | grep -q \"$USD_BUILD_DIR/lib/python\"; then\n  export PYTHONPATH=\$PYTHONPATH:$USD_BUILD_DIR/lib/python\nfi"
    # Check if the line already exists in the file
    if ! grep -Fxq "$PYTHONPATH_TO_ADD" "$SETUP_PATH"; then
        # Add the line to the file
        echo "$PYTHONPATH_TO_ADD" >> $SETUP_PATH
    fi
fi