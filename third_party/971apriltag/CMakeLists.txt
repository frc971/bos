cmake_minimum_required(VERSION 3.10)

include_directories(. /usr/local/cuda/include ${OpenCV_INCLUDE_DIRS})
add_compile_options(-march=armv8-a+simd)

find_package(wpilib REQUIRED)
enable_language(CUDA)


add_library(971apriltag_dep)
target_sources(971apriltag_dep
    PRIVATE
    apriltag.c
)
set_target_properties(971apriltag_dep PROPERTIES LINKER_LANGUAGE C)

add_library(971apriltag_cuda_dep cuda.cc)
target_link_libraries(971apriltag_cuda_dep PUBLIC apriltag absl::log absl::check 971apriltag_dep cudart)
set_source_files_properties(threshold.cc PROPERTIES LANGUAGE CUDA)


add_library(971apriltag_threshold_dep threshold.cc fatal_neon_threshold.cc)
target_link_libraries(971apriltag_threshold_dep PUBLIC apriltag absl::log absl::check 971apriltag_dep)

add_library(971apriltag 
  apriltag.cc
  apriltag_detect.cc
  labeling_allegretti_2019_BKE.cc
  line_fit_filter.cc
  points.cc
)

target_compile_options(971apriltag PRIVATE -w)
target_link_libraries(971apriltag PUBLIC apriltag 971apriltag_cuda_dep 971apriltag_threshold_dep absl::log absl::check 971apriltag_dep)
set_source_files_properties(labeling_allegretti_2019_BKE.cc apriltag_detect.cc apriltag.cc line_fit_filter.cc points.cc PROPERTIES LANGUAGE CUDA)
