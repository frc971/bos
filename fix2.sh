#!/bin/bash
git restore src/camera/CMakeLists.txt
sed -i 's/add_library(SimulatedUVCCamera simulated_uvc_camera.cc)/#add_library(SimulatedUVCCamera simulated_uvc_camera.cc)/g' src/camera/CMakeLists.txt
sed -i 's/target_link_libraries(SimulatedUVCCamera PRIVATE utils absl::flags absl::flags_parse)/#target_link_libraries(SimulatedUVCCamera PRIVATE utils absl::flags absl::flags_parse)/g' src/camera/CMakeLists.txt
sed -i 's/SimulatedUVCCamera//g' src/camera/CMakeLists.txt
cd build && make ellipse_simulator
