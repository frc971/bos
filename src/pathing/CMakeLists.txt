add_library(pathing pathfinding.cc splines.cc controller.cc)
target_link_libraries(pathing PUBLIC wpilibc wpiutil ntcore nlohmann_json::nlohmann_json utils localization ${OpenCV_LIBS})
