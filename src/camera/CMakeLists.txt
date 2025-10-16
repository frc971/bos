add_library(camera imx296_camera.cc streamer.cc realsense_camera.cc cscore_streamer.cc)
find_package(realsense2 REQUIRED)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} cscore realsense2::realsense2)
