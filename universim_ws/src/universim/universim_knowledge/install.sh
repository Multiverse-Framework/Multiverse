#!/usr/bin/env sh

# Check if USD_SRC_PATH or USD_BUILD_PATH exists
if [ -z "$USD_SRC_PATH" ] || [ -z "$USD_BUILD_PATH" ]; then
    echo "USD_SRC_PATH or USD_BUILD_PATH is unset." >&2
    exit 1
else
    echo "USD_SRC_PATH is set to: $USD_SRC_PATH"
    echo "USD_BUILD_PATH is set to: $USD_BUILD_PATH"

    cp -r USD/plugin $USD_SRC_PATH

    # Loop through each folder in the directory
    for USD_PLUGIN in $USD_SRC_PATH/plugin/*; do
        if [ -d "$USD_PLUGIN" ]; then
            # Execute your command within each folder
            (cd "$USD_PLUGIN" && $USD_BUILD_PATH/bin/usdGenSchema schema.usda)
        fi
    done

    # Specify the file path
    USD_CMAKE_PATH="$USD_SRC_PATH/CMakeLists.txt"

    # Specify the line to add
    LINE_TO_ADD="add_subdirectory(plugin)"

    # Check if the line already exists in the file
    if ! grep -Fxq "$LINE_TO_ADD" "$USD_CMAKE_PATH"; then

        # Add the line to the file
        echo "\n$LINE_TO_ADD" >> $USD_CMAKE_PATH
    fi

    python3 $USD_SRC_PATH/build_scripts/build_usd.py $USD_BUILD_PATH
fi