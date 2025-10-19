add_library(camera cscore_streamer.cc disk_camera.cc cv_camera.cc select_camera.cc)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} cscore)

target_precompile_headers(camera PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)