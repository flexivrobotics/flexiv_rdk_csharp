#!/bin/sh

CURRSCRIPTPATH="$(dirname $(readlink -f $0))"
bash third_party/build_and_install_flexiv_rdk.sh

echo ">>> Build and install flexiv rdk dll v1.5.1"
cd $CURRSCRIPTPATH
rm -rf build/
cmake -S . -B build
cmake --build build --config Release --target install

echo ">>> Installed successfully: flexiv rdk dll v1.5.1"
echo ">>> Installed path: $CURRSCRIPTPATH/install"