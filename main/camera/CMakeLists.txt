find_package(OpenCV REQUIRED)

add_library(camera camera.cc streamer.cc)
target_include_directories(camera PRIVATE  ${CMAKE_SOURCE_DIR})
target_link_libraries(camera PRIVATE ${OpenCV_LIBS})


add_executable(test camera.cc streamer.cc test.cc)
target_include_directories(test PRIVATE  ${CMAKE_SOURCE_DIR})
target_link_libraries(test PRIVATE ${OpenCV_LIBS})

