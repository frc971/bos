add_library(camera camera.cc streamer.cc)
target_link_libraries(camera PRIVATE ${OpenCV_LIBS})
