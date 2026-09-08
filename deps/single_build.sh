#!/bin/bash
cd /bos
rm -f /tmp/ellipse_sim_gd.png
g++ -I/bos \
    -I/bos/third_party/json/include \
    -I/usr/include/opencv4 \
    -isystem /usr/local/include/wpimath \
    -isystem /usr/local/include/wpiutil \
    -isystem /usr/local/include/wpilibc \
    -std=c++20 -O2 \
    src/pathing/ellipse_simulator.cc \
    src/pathing/pathfinding.cc \
    src/pathing/splines.cc \
    src/pathing/controller.cc \
    src/pathing/velocity_profile.cc \
    src/pathing/path_follower.cc \
    src/pathing/cluster_follower.cc \
    src/utils/log.cc \
    src/utils/timer.cc \
    src/utils/constants_from_json.cc \
    -lopencv_core -lopencv_highgui -lopencv_imgproc -lwpimath -lwpiutil \
    -o single_sim_bin

./single_sim_bin
