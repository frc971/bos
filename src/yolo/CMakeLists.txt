add_library(yolo yolo.cc)
target_link_libraries(yolo PRIVATE ${OpenCV_LIBS} nvinfer nvinfer_plugin cudart)

add_executable(realsense_yolo_test realsense_yolo_test.cc)
target_link_libraries(realsense_yolo_test PRIVATE camera yolo)

# add_executable(angle_tester angle_tester.cc)
# target_link_libraries(angle_tester PRIVATE camera yolo)

# add_executable(distance_tester distance_tester.cc)
# target_link_libraries(distance_tester PRIVATE camera yolo)

add_subdirectory(training)
