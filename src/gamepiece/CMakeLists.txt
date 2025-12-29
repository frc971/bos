add_library(gamepiece gamepiece.cc)
target_link_libraries(gamepiece wpilibc ntcore ${OpenCV_LIBS} camera utils yolo)

