#!/bin/bash

CURRENT_DIR=$PWD

cd $(dirname $0)

BUILD_DIR=$PWD/multiverse/build

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

if ! echo "$PYTHONPATH" | grep -q "$LIB_DIR/dist-packages"; then
    PYTHONPATH=$PYTHONPATH:$LIB_DIR/dist-packages
    PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH"
    echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

BUILD_SRC=ON
BUILD_MODULES=ON
BUILD_CONNECTORS=ON
BUILD_KNOWLEDGE=ON
BUILD_PARSER=ON
BUILD_RESOURCES=ON
BUILD_TESTS=OFF

while [ -n "$1" ]; do
    case "$1" in
        --with-tests) echo "--with-tests option passed"
            shift 1
            BUILD_TESTS=ON
            break
        ;;
        --only-src) echo "--only-src option passed"
            shift 1
            BUILD_MODULES=OFF
            BUILD_RESOURCES=OFF
            BUILD_KNOWLEDGE=OFF
            BUILD_PARSER=OFF
            break
        ;;
        --only-modules) echo -n "--only-modules option passed"
            shift 1
            if [ "$#" -eq 0 ]; then
                echo ""
            else
                echo -n ", with value:"
                BUILD_SRC=OFF
                BUILD_RESOURCES=OFF
                for module in "$@"; do
                    echo -n " $module"
                    shift 1
                    if [ "$module" = "connectors" ]; then
                        BUILD_KNOWLEDGE=OFF
                        BUILD_PARSER=OFF
                    elif [ "$module" = "knowledge" ]; then
                        BUILD_CONNECTORS=OFF
                        BUILD_PARSER=OFF
                    elif [ "$module" = "parser" ]; then
                        BUILD_CONNECTORS=OFF
                        BUILD_KNOWLEDGE=OFF
                    fi
                done
                echo ""
            fi
            break
        ;;
        --no-src) echo "--no-src option passed"
            shift 1
            BUILD_SRC=OFF
        ;;
        --no-modules) echo -n "--no-modules option passed"
            shift 1
            if [ "$#" -eq 0 ]; then
                echo ""
                BUILD_MODULES=OFF
            else
                echo -n ", with value:"
                for module in "$@"; do
                    echo -n " $module"
                    shift 1
                    if [ "$module" = "connectors" ]; then
                        BUILD_CONNECTORS=OFF
                    elif [ "$module" = "knowledge" ]; then
                        BUILD_KNOWLEDGE=OFF
                    elif [ "$module" = "parser" ]; then
                        BUILD_PARSER=OFF
                    fi
                done
                echo ""
            fi
        ;;
        *) echo "Option $1 not recognized"
            shift 1
        ;;
    esac
done

UBUNTU_VERSION=$(lsb_release -rs)
PYTHON_EXECUTABLE=python3.10

CMAKE_EXECUTABLE=$BUILD_DIR/CMake/bin/cmake
if [ ! -f "$CMAKE_EXECUTABLE" ]; then
    CMAKE_EXECUTABLE=$(which cmake)
fi
if [ ! -f "$CMAKE_EXECUTABLE" ]; then
    echo "cmake does not exist."
    exit 1
fi

if [ $BUILD_PARSER = ON ]; then
    echo "Updating multiverse_parser..."
    git submodule update --init $CURRENT_DIR/multiverse/modules/multiverse_parser

    if ! echo "$PYTHONPATH" | grep -q "$USD_BUILD_DIR/lib/python"; then
        PYTHONPATH=$PYTHONPATH:$USD_BUILD_DIR/lib/python
        PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH"
        echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
        echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
        RELOAD=true
    fi
fi
if [ $BUILD_KNOWLEDGE = ON ]; then
    echo "Updating multiverse_knowledge..."
    git submodule update --init $CURRENT_DIR/multiverse/modules/multiverse_knowledge
    (cd $CURRENT_DIR/multiverse/modules/multiverse_knowledge && git submodule update --init ease_lexical_resources)
fi
if [ $BUILD_RESOURCES = ON ]; then
    echo "Updating multiverse_resources..."
    (git submodule update --init $CURRENT_DIR/multiverse/resources; cd $CURRENT_DIR/multiverse/resources; git submodule update --init)
fi
for virtualenvwrapper in $(which virtualenvwrapper.sh) /usr/share/virtualenvwrapper/virtualenvwrapper.sh /usr/local/bin/virtualenvwrapper.sh /home/$USER/.local/bin/virtualenvwrapper.sh; do
    if [ -f $virtualenvwrapper ]; then
        break
    fi
done
if [ ! -f $virtualenvwrapper ]; then
    echo "virtualenvwrapper.sh not found"
    exit 1
else
    . $virtualenvwrapper
    mkvirtualenv --system-site-packages multiverse -p $PYTHON_EXECUTABLE
    $PYTHON_EXECUTABLE -m pip install -U pip build setuptools packaging distro
    $PYTHON_EXECUTABLE -m pip install --force-reinstall numpy==1.26.4 # Fix numpy version
fi

# Build multiverse
# cmake -S $PWD/multiverse -B $BUILD_DIR \
#     -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=SHARED -DSTDLIB=libc++ \
#     -DBUILD_SRC=$BUILD_SRC \
#     -DBUILD_MODULES=$BUILD_MODULES \
#     -DBUILD_CONNECTORS=$BUILD_CONNECTORS \
#     -DBUILD_KNOWLEDGE=$BUILD_KNOWLEDGE \
#     -DBUILD_PARSER=$BUILD_PARSER

rm -f $BUILD_DIR/CMakeCache.txt
if [ $BUILD_SRC = ON ]; then
    $CMAKE_EXECUTABLE -S $PWD/multiverse -B $BUILD_DIR \
        -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=STATIC -DSTDLIB=libstdc++ \
        -DBUILD_SRC=ON \
        -DBUILD_MODULES=OFF \
        -DBUILD_TESTS=OFF \
        -DPYTHON_EXECUTABLE=$PYTHON_EXECUTABLE
    make -C $BUILD_DIR
    $CMAKE_EXECUTABLE --install $BUILD_DIR
    
    if [ $UBUNTU_VERSION = "20.04" ]; then
        rm -f $BUILD_DIR/CMakeCache.txt
        $CMAKE_EXECUTABLE -S $PWD/multiverse -B $BUILD_DIR \
            -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=STATIC -DSTDLIB=libstdc++ \
            -DBUILD_SRC=ON \
            -DBUILD_MODULES=OFF \
            -DBUILD_TESTS=OFF \
            -DPYTHON_EXECUTABLE=python3.8
        make -C $BUILD_DIR
        $CMAKE_EXECUTABLE --install $BUILD_DIR
    elif [ $UBUNTU_VERSION = "24.04" ]; then
        rm -f $BUILD_DIR/CMakeCache.txt
        $CMAKE_EXECUTABLE -S $PWD/multiverse -B $BUILD_DIR \
            -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=STATIC -DSTDLIB=libstdc++ \
            -DBUILD_SRC=ON \
            -DBUILD_MODULES=OFF \
            -DBUILD_TESTS=OFF \
            -DPYTHON_EXECUTABLE=python3.12
        make -C $BUILD_DIR
        $CMAKE_EXECUTABLE --install $BUILD_DIR
    fi
fi

$CMAKE_EXECUTABLE -S $PWD/multiverse -B $BUILD_DIR \
    -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=STATIC -DSTDLIB=libstdc++ \
    -DBUILD_SRC=OFF \
    -DBUILD_MODULES=$BUILD_MODULES \
    -DBUILD_CONNECTORS=$BUILD_CONNECTORS \
    -DBUILD_KNOWLEDGE=$BUILD_KNOWLEDGE \
    -DBUILD_PARSER=$BUILD_PARSER \
    -DBUILD_TESTS=$BUILD_TESTS \
    -DPYTHON_EXECUTABLE=$PYTHON_EXECUTABLE
make -C $BUILD_DIR
$CMAKE_EXECUTABLE --install $BUILD_DIR

cd $CURRENT_DIR

if [ "$RELOAD" = true ]; then
    rosdep update
    exec bash # Reload ~/.bashrc
fi