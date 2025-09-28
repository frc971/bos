add_library(camera imx296_camera.cc streamer.cc realsense_camera.cc cscore_streamer.cc)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} cscore)
