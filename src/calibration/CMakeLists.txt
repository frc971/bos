add_executable(IntrinsicsCalibrate intrinsics_calibrate.cc intrinsics_calibrate_lib.cc)
target_sources(IntrinsicsCalibrate
  PRIVATE
  intrinsics_calibrate.cc
  intrinsics_calibrate_lib.cc
)
target_link_libraries(IntrinsicsCalibrate
  PRIVATE
  camera
  utils
)

add_executable(frame_shower frame_shower.cc)
target_sources(frame_shower
  PRIVATE
  frame_shower.cc
)
target_link_libraries(frame_shower
  PRIVATE
  camera
  utils
)

add_executable(focus_calibrate focus_calibrate.cc)
target_sources(focus_calibrate
  PRIVATE
  focus_calibrate.cc
)
target_link_libraries(focus_calibrate
  PRIVATE
  camera
  utils
)
