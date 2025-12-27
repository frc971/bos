add_library(game_piece game_piece.cc)
target_link_libraries(game_piece wpilibc ntcore ${OpenCV_LIBS} camera utils yolo nlohmann_json::nlohmann_json)  
