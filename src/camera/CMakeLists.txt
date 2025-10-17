add_library(camera imx296_camera.cc streamer.cc cscore_streamer.cc disk_camera.cc usb_camera.cc)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} cscore)

target_precompile_headers(camera PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)