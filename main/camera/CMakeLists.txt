find_package(OpenCV REQUIRED)

add_library(camera camera.cc streamer.cc)
target_include_directories(camera PRIVATE  ${CMAKE_SOURCE_DIR})
target_link_libraries(camera PRIVATE ${OpenCV_LIBS})
