add_executable(simulator simulator.cc pathing.cc) 
target_link_libraries(simulator PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json ntcore wpilibc wpimath)

add_library(controller controller.cc velocity_sender.cc pathing.cc)
target_link_libraries(controller PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json ntcore wpilibc wpimath)

add_executable(pathing_controller_main controller_main.cc)
target_link_libraries(pathing_controller_main PRIVATE controller ntcore wpilibc wpimath nlohmann_json::nlohmann_json ${OpenCV_LIBS})
