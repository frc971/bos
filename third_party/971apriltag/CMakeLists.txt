cmake_minimum_required(VERSION 3.10)

include_directories(. /usr/local/cuda/include ${OpenCV_INCLUDE_DIRS})

find_package(wpilib REQUIRED)

add_library(971apriltag_dep STATIC apriltag.c)

set_target_properties(971apriltag_dep PROPERTIES LINKER_LANGUAGE C)

add_library(971apriltag SHARED
971apriltag.cu
  apriltag_detect.cu
  labeling_allegretti_2019_BKE.cu
  line_fit_filter.cu
  points.cu
  threshold.cu
  cuda.cc
)

target_link_libraries(971apriltag PRIVATE apriltag 971apriltag_dep)
