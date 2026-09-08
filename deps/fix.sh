#!/bin/bash
sed -i 's/-warnings-as-errors=\*//g' /bos/CMakeLists.txt
sed -i 's/-Werror//g' /bos/CMakeLists.txt
rm -rf /bos/build 
mkdir -p /bos/build 
cd /bos/build
cmake -DBUILD_TESTING=OFF -DCMAKE_CXX_COMPILER_ID=GNU ..
make -j4 ellipse_simulator
