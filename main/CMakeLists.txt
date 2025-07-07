cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(calibration)
add_subdirectory(camera)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(main main.cc pose_estimator.cc position_sender.cc)
target_include_directories(main PRIVATE  ${CMAKE_SOURCE_DIR} /usr/local/cuda/include)
target_link_libraries(main PRIVATE ${OpenCV_LIBS} 971apriltag apriltag cameraserver ntcore cscore wpiutil camera)
