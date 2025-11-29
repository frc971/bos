add_library(camera cscore_streamer.cc disk_camera.cc cv_camera.cc select_camera.cc realsense_camera.cc)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} cscore realsense2::realsense2 nlohmann_json::nlohmann_json)
