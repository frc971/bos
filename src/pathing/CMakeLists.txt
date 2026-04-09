add_library(pathing pathfinding.cc splines.cc controller.cc)
target_link_libraries(pathing PUBLIC wpilibc wpiutil ntcore nlohmann_json::nlohmann_json utils localization ${OpenCV_LIBS})

add_executable(bfs-test bfs_tester.cc)
target_link_libraries(bfs-test PRIVATE pathing utils nlohmann_json::nlohmann_json wpiutil)
