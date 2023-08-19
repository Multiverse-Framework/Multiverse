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

PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH:$LIB_DIR/python"
if ! echo "$PYTHONPATH" | grep -q "$LIB_DIR/libstdc++/python"; then
    echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

# Build multiverse
# cmake -S $PWD/multiverse -B $BUILD_DIR -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=STATIC -DSTDLIB=libstdc++
cmake -S $PWD/multiverse -B $BUILD_DIR -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=SHARED -DSTDLIB=libc++
make -C $BUILD_DIR
cmake --install $BUILD_DIR

cd $CURRENT_DIR

if [ "$RELOAD" = true ]; then
    exec bash # Reload ~/.bashrc
fi