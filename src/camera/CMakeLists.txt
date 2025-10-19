add_library(camera cscore_streamer.cc disk_camera.cc cv_camera.cc)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} cscore)
