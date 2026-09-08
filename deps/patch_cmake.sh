#!/bin/bash
cd /bos
rm -rf build 
mkdir build
cd build
export LLVM_SYMBOLIZER_PATH="" # attempt preventing some bloat
# Totally disable clang-tidy build integration completely
cmake -DCMAKE_C_COMPILER_LAUNCHER="" -DCMAKE_CXX_COMPILER_LAUNCHER="" -DCMAKE_CXX_CLANG_TIDY="" -DCXXFLAGS="-Wno-error" ..
make ellipse_simulator -j1
./src/pathing/ellipse_simulator
cp /tmp/ellipse_sim_gd.png /root/.gemini/antigravity-cli/brain/40f82922-443e-461d-9a92-826e54d84cca/ellipse_sim.png
