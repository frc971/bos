add_library(yolo yolo.cc)
target_link_libraries(yolo PRIVATE ${OpenCV_LIBS} nvinfer nvinfer_plugin cudart)

add_subdirectory(training)
