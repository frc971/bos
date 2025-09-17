add_library(yolo yolo.cc)
target_link_libraries(yolo PRIVATE ${OpenCV_LIBS})
