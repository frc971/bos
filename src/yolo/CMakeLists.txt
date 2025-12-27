add_executable(image_collector image_collector.cc)
target_link_libraries(image_collector PRIVATE ${OpenCV_LIBS} camera wpilibc)

add_library(yolo yolo.cc)
target_link_libraries(yolo PRIVATE ${OpenCV_LIBS} nvinfer nvinfer_plugin cudart)

add_library(game_piece game_piece.cc)
target_link_libraries(game_piece wpilibc ntcore ${OpenCV_LIBS} camera utils yolo nlohmann_json::nlohmann_json)
