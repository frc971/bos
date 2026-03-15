cmake_minimum_required(VERSION 3.10)

add_executable(intrinsics_calibrate intrinsics_calibrate.cc intrinsics_calibrate_lib.cc)
target_link_libraries(intrinsics_calibrate PRIVATE camera utils)

add_executable(frame_shower frame_shower.cc)
target_link_libraries(frame_shower PRIVATE camera utils)

add_executable(focus_calibrate focus_calibrate.cc)
target_link_libraries(focus_calibrate PRIVATE camera utils)

