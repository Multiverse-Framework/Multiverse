#!/usr/bin/env sh

CURRENT_DIR=$PWD

cd $(dirname $0)

BUILD_DIR=$PWD/multiverse/build/multiverse

USD_BUILD_DIR=$BUILD_DIR/USD

BIN_DIR=$PWD/multiverse/bin

LIB_DIR=$PWD/multiverse/lib

if [ ! -d "$LIB_DIR" ]; then
    # Create the folder if it doesn't exist
    mkdir -p $LIB_DIR
fi

RELOAD=false

if ! echo "$PATH" | grep -q "$BIN_DIR"; then
    PATH=$PATH:$BIN_DIR
    PATH_TO_ADD="export PATH=$PATH"
    echo "$PATH_TO_ADD" >> ~/.bashrc
    echo "Add $PATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

if ! echo "$PYTHONPATH" | grep -q "$USD_BUILD_DIR/lib/python"; then
    PYTHONPATH=$PYTHONPATH:$USD_BUILD_DIR/lib/python
    PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH"
    echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

if ! echo "$PYTHONPATH" | grep -q "$LIB_DIR/libstdc++/python"; then
    PYTHONPATH=$PYTHONPATH:$LIB_DIR/libstdc++/python
    PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH"
    echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

# Build multiverse
cmake -S $PWD/multiverse -B $BUILD_DIR -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=STATIC -DSTDLIB=libstdc++
# cmake -S $PWD/multiverse -B $BUILD_DIR -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=SHARED -DSTDLIB=libc++
make -C $BUILD_DIR
cmake --install $BUILD_DIR

cd $CURRENT_DIR

if [ "$RELOAD" = true ]; then
    exec bash # Reload ~/.bashrc
fi

rosdep update