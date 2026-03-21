cmake_minimum_required(VERSION 3.10)

include_directories(. /usr/local/cuda/include ${OpenCV_INCLUDE_DIRS})

find_package(wpilib REQUIRED)
enable_language(CUDA)

add_library(971apriltag_cuda_dep STATIC cuda.cc)
target_link_libraries(971apriltag_cuda_dep PRIVATE apriltag absl::log absl::check)
set_source_files_properties(threshold.cc PROPERTIES LANGUAGE CUDA)


add_library(971apriltag_threshold_dep STATIC threshold.cc)
target_link_libraries(971apriltag_threshold_dep PRIVATE apriltag absl::log absl::check)

add_library(971apriltag STATIC 
  apriltag.cc
  apriltag_detect.cc
  labeling_allegretti_2019_BKE.cc
  line_fit_filter.cc
  points.cc
)

target_compile_options(971apriltag PRIVATE -w)
target_link_libraries(971apriltag PRIVATE apriltag 971apriltag_cuda_dep 971apriltag_threshold_dep absl::log absl::check)
set_source_files_properties(labeling_allegretti_2019_BKE.cc apriltag_detect.cc apriltag.cc line_fit_filter.cc points.cc PROPERTIES LANGUAGE CUDA)
