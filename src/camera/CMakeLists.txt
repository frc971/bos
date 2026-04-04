add_library(camera cscore_streamer.cc disk_camera.cc cv_camera.cc select_camera.cc camera_source.cc write_frame.cc camera_constants.cc multi_camera_source.cc)
target_link_libraries(camera PRIVATE utils)
