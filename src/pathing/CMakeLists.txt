add_library(pathing pathfinding.cc splines.cc controller.cc)
target_link_libraries(pathing PUBLIC wpilibc wpiutil ntcore nlohmann_json::nlohmann_json utils ${OpenCV_LIBS})

# add_executable(simulator simulator.cc)
# target_link_libraries(simulator PRIVATE pathing utils nlohmann_json::nlohmann_json wpiutil)
