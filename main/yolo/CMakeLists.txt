link_directories(/usr/local/cuda-12.6/targets/aarch64-linux/lib)

add_library(yolo yolo.cc)
target_link_libraries(yolo PRIVATE cudart)
target_include_directories(yolo
    PUBLIC ${CMAKE_SOURCE_DIR}/main
    ./usr/local/cuda-12.6/include
)
target_link_libraries(yolo PRIVATE ${OpenCV_LIBS})

add_executable(realsense_yolo_test realsense_yolo_test.cc)
target_include_directories(realsense_yolo_test
    PRIVATE ${CMAKE_SOURCE_DIR}/main
)
target_link_libraries(realsense_yolo_test PRIVATE camera yolo ${OpenCV_LIBS})
