add_executable(image_collector image_collector.cc)
target_link_libraries(image_collector PRIVATE ${OpenCV_LIBS} camera)
