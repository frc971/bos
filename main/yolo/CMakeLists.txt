link_directories(/usr/local/cuda-12.6/targets/aarch64-linux/lib)

add_library(yolo yolo.cc)
target_include_directories(yolo
    PUBLIC ${CMAKE_SOURCE_DIR}/main
    /usr/local/cuda-12.6/include
    /usr/local/cuda-12.6/compat/
    /usr/lib/aarch64-linux-gnu/
)
target_link_libraries(yolo PUBLIC ${OpenCV_LIBS} cudart)

add_executable(realsense_yolo_test realsense_yolo_test.cc)
target_link_libraries(realsense_yolo_test PRIVATE camera yolo nvinfer nvinfer_plugin ${OpenCV_LIBS})
