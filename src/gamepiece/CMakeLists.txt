add_executable(game_piece_main game_piece_main.cc)
target_link_libraries(game_piece_main wpilibc ntcore ${OpenCV_LIBS} camera utils yolo)

