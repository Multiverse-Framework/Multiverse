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

if ! echo "$PYTHONPATH" | grep -q "$USD_BUILD_DIR/lib/python"; then
    PYTHONPATH=$PYTHONPATH:$USD_BUILD_DIR/lib/python
    PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH"
    echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

if ! echo "$PYTHONPATH" | grep -q "$LIB_DIR/dist-packages"; then
    PYTHONPATH=$PYTHONPATH:$LIB_DIR/dist-packages
    PYTHONPATH_TO_ADD="export PYTHONPATH=$PYTHONPATH"
    echo "$PYTHONPATH_TO_ADD" >> ~/.bashrc
    echo "Add $PYTHONPATH_TO_ADD to ~/.bashrc"
    RELOAD=true
fi

DBUILD_SRC=ON
DBUILD_MODULES=ON
DBUILD_CONNECTORS=ON
DBUILD_KNOWLEDGE=ON
DBUILD_PARSER=ON

while [ -n "$1" ]; do
    case "$1" in
        --only-src) echo "--only-src option passed"
            shift 1
            DBUILD_MODULES=OFF
            break
        ;;
        --only-modules) echo -n "--only-modules option passed, with value:"
            shift 1
            if [ "$#" -eq 0 ]; then
                echo ""
            else
                echo -n ", with value:"
                DBUILD_SRC=OFF
                for module in "$@"; do
                    echo -n " $module"
                    shift 1
                    if [ "$module" = "connectors" ]; then
                        DBUILD_SRC=ON
                        DBUILD_KNOWLEDGE=OFF
                        DBUILD_PARSER=OFF
                    elif [ "$module" = "knowledge" ]; then
                        DBUILD_CONNECTORS=OFF
                        DBUILD_PARSER=OFF
                    elif [ "$module" = "parser" ]; then
                        DBUILD_SRC=ON
                        DBUILD_KNOWLEDGE=OFF
                        DBUILD_PARSER=ON
                    fi
                done
                echo ""
            fi
            break
        ;;
        --no-src) echo "--no-src option passed"
            shift 1
            DBUILD_SRC=OFF
        ;;
        --no-modules) echo -n "--no-modules option passed"
            shift 1
            if [ "$#" -eq 0 ]; then
                echo ""
                DBUILD_MODULES=OFF
            else
                echo -n ", with value:"
                for module in "$@"; do
                    echo -n " $module"
                    shift 1
                    if [ "$module" = "connectors" ]; then
                        DBUILD_CONNECTORS=OFF
                    elif [ "$module" = "knowledge" ]; then
                        DBUILD_KNOWLEDGE=OFF
                    elif [ "$module" = "parser" ]; then
                        DBUILD_PARSER=OFF
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

if [ $DBUILD_KNOWLEDGE = ON ]; then
    echo "Updating multiverse_knowledge..."
    git submodule update --init $PWD/multiverse/modules/multiverse_knowledge
fi
for virtualenvwrapper in $(which virtualenvwrapper.sh) /usr/share/virtualenvwrapper/virtualenvwrapper.sh /usr/local/bin/virtualenvwrapper.sh /home/$USER/.local/bin/virtualenvwrapper.sh; do
    if [ -f $virtualenvwrapper ]; then
        . $virtualenvwrapper
        mkvirtualenv --system-site-packages multiverse
        pip install -U pip build # Ensure pip and build are up-to-date
        break
    fi
done
if [ ! -f $virtualenvwrapper ]; then
    echo "virtualenvwrapper.sh not found"
    exit 1
fi

# Build multiverse
# cmake -S $PWD/multiverse -B $BUILD_DIR \
#     -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=SHARED -DSTDLIB=libc++ \
#     -DBUILD_SRC=$DBUILD_SRC \
#     -DBUILD_MODULES=$DBUILD_MODULES \
#     -DBUILD_CONNECTORS=$DBUILD_CONNECTORS \
#     -DBUILD_KNOWLEDGE=$DBUILD_KNOWLEDGE \
#     -DBUILD_PARSER=$DBUILD_PARSER
cmake -S $PWD/multiverse -B $BUILD_DIR \
    -DCMAKE_INSTALL_PREFIX:PATH=$PWD/multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=STATIC -DSTDLIB=libstdc++ \
    -DBUILD_SRC=$DBUILD_SRC \
    -DBUILD_MODULES=$DBUILD_MODULES \
    -DBUILD_CONNECTORS=$DBUILD_CONNECTORS \
    -DBUILD_KNOWLEDGE=$DBUILD_KNOWLEDGE \
    -DBUILD_PARSER=$DBUILD_PARSER
make -C $BUILD_DIR
cmake --install $BUILD_DIR

cd $CURRENT_DIR

if [ "$RELOAD" = true ]; then
    rosdep update
    exec bash # Reload ~/.bashrc
fi