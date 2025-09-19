add_library(camera imx296_camera.cc streamer.cc cscore_streamer.cc)
target_link_libraries(camera PRIVATE ${OpenCV_LIBS} cscore)
