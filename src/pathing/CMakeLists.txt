add_executable(pathing pathing.cc)
target_link_libraries(pathing PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json ntcore wpilibc wpimath)

add_executable(simulator simulator.cc)
target_link_libraries(simulator PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json ntcore wpilibc wpimath)