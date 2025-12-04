add_library(camera cscore_streamer.cc disk_camera.cc cv_camera.cc select_camera.cc camera_source.cc)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} 971apriltag ntcore apriltag nlohmann_json::nlohmann_json wpilibc)
