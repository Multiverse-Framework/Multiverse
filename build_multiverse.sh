#!/usr/bin/env sh

CURRENT_DIR=$PWD

cd $(dirname $0)

BUILD_DIR=$PWD/multiverse/build/multiverse

LIB_DIR=$PWD/multiverse/lib
if [ ! -d "$LIB_DIR" ]; then
    # Create the folder if it doesn't exist
    mkdir -p $LIB_DIR
fi

RELOAD=false

# Add to PATH
PATH_TO_ADD="export PATH=\$PATH:$PWD/multiverse/bin"
if ! grep -Fxq "$PATH_TO_ADD" ~/.bashrc; then
    echo "$PATH_TO_ADD" >> ~/.bashrc
    echo "Add $PATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

# Add to PYTHONPATH
PYTHONPATH_TO_ADD="export PYTHONPATH=\$PYTHONPATH:$PWD/multiverse/lib/python"
if ! grep -Fxq "$PYTHONPATH_TO_ADD" ~/.bashrc; then
    echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

# Add to LD_LIBRARY_PATH
LD_LIBRARY_PATH_TO_ADD="export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$PWD/multiverse/lib"
if ! grep -Fxq "$LD_LIBRARY_PATH_TO_ADD" ~/.bashrc; then
    echo "$LD_LIBRARY_PATH_TO_ADD" >> ~/.bashrc
    echo "Add $LD_LIBRARY_PATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

cd $CURRENT_DIR

if [ "$RELOAD" = true ]; then
    exec bash # Reload ~/.bashrc
fi

# Build multiverse_server
cmake -S $PWD/multiverse -B $BUILD_DIR -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse
make -C $BUILD_DIR
cmake --install $BUILD_DIR