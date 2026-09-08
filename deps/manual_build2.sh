#!/bin/bash
cd /bos
rm -rf sim_bin
# Let's write the exact simplest compilation ignoring camera entirely but using the build structure
cd build
make clean
cd ..
cmake -E env CXXFLAGS="-Wno-error" cmake -S . -B build_clean
cmake --build build_clean -j4 --target ellipse_simulator
./build_clean/src/pathing/ellipse_simulator
