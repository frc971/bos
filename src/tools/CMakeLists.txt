add_subdirectory(path_camera_sim)

add_executable(localization_stretch localization_stretch.cc)
target_link_libraries(localization_stretch PRIVATE
  OpencvApriltagDetector
  absl::flags
  absl::flags_parse
  opencv_imgcodecs
  utils
)
