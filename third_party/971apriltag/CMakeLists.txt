cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include_directories(. /usr/local/cuda/include ${OpenCV_INCLUDE_DIRS})

find_package(wpilib REQUIRED)
add_library(971apriltag SHARED
971apriltag.cu
  apriltag_detect.cu
  labeling_allegretti_2019_BKE.cu
  line_fit_filter.cu
  points.cu
  threshold.cu
  cuda.cc
)

add_library(971apriltag_dep STATIC)
target_sources(971apriltag_dep
    PRIVATE
    apriltag.c
)
set_target_properties(971apriltag_dep PROPERTIES LINKER_LANGUAGE C)

target_compile_options(971apriltag PRIVATE -w)
target_link_libraries(971apriltag PRIVATE apriltag 971apriltag_dep)
