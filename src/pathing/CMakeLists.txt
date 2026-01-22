add_executable(simulator simulator.cc pathing.cc)
target_link_libraries(simulator PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json ntcore wpilibc wpimath)