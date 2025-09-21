add_library(camera imx296_camera.cc streamer.cc realsense_camera.cc)
find_package(realsense2 REQUIRED)
target_link_libraries(camera PRIVATE ${OpenCV_LIBS} realsense2::realsense2)
