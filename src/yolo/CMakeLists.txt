add_library(yolo yolo.cc)
target_link_libraries(yolo PRIVATE nvinfer nvinfer_plugin cudart utils)

