add_executable(image_collector image_collector.cc)
target_link_libraries(image_collector PRIVATE ${OpenCV_LIBS} camera wpilibc)

add_library(yolo yolo.cc)
target_link_libraries(yolo PRIVATE ${OpenCV_LIBS} nvinfer nvinfer_plugin cudart)
