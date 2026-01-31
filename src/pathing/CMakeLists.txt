add_executable(simulator simulator.cc pathing.cc) 
target_link_libraries(simulator PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json ntcore wpilibc wpimath)

add_library(controller controller.cc velocity_sender.cc pathing.cc)
target_link_libraries(controller PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json ntcore wpilibc wpimath)
