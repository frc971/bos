add_library(camera imx296_camera.cc streamer.cc)
target_link_libraries(camera PRIVATE ${OpenCV_LIBS})
