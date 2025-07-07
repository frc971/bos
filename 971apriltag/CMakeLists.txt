cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
include_directories( . /usr/local/cuda/include ${OpenCV_INCLUDE_DIRS})

add_library( 971apriltag SHARED
	971apriltag.cu
  apriltag_detect.cu
  labeling_allegretti_2019_BKE.cu
  line_fit_filter.cu
  points.cu
  threshold.cu
  cuda.cc
)

target_include_directories(971apriltag PRIVATE ${CMAKE_SOURCE_DIR}/third_party/apriltag)
target_compile_options(971apriltag PRIVATE -w)
target_link_libraries(971apriltag PRIVATE ${CMAKE_SOURCE_DIR}/third_party/apriltag/build/libapriltag.a)
