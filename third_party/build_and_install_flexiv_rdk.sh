#!/bin/sh
echo ">>> Installing components: flexiv_rdk"
SCRIPTPATH="$(dirname $(readlink -f $0))"

set -e
INSTALL_DIR="$SCRIPTPATH/../install"
NUM_JOBS=4

if [ ! -d "$SCRIPTPATH/flexiv_rdk" ] || [ -z "$(ls -A $SCRIPTPATH/flexiv_rdk)" ]; then
    echo ">>> Cloning flexiv_rdk repo..."
    git clone https://github.com/flexivrobotics/flexiv_rdk.git "$SCRIPTPATH/flexiv_rdk"
else
    echo ">>> flexiv_rdk already exists and is not empty"
fi

cd $SCRIPTPATH/flexiv_rdk
# Use specific version
git fetch -p
git checkout v1.7
# git submodule update --init --recursive
# Build and install dependencies 
cd thirdparty
bash build_and_install_dependencies.sh $INSTALL_DIR/rdk_install
cd ..
# Configure CMake
cmake -S . -B build -DCMAKE_PREFIX_PATH=$INSTALL_DIR/rdk_install
# Build and install
cmake --build build --target install --config release -j $NUM_JOBS
echo ">>> Installed components: flexiv rdk"