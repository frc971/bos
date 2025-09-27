add_library(camera imx296_camera.cc streamer.cc realsense_camera.cc)
target_include_directories(camera
    PUBLIC ${CMAKE_SOURCE_DIR}/main
)
target_link_libraries(camera PRIVATE ${OpenCV_LIBS} cscore)
add_library(camera imx296_camera.cc streamer.cc cscore_streamer.cc cscore)
