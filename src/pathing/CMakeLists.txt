add_executable(simulator simulator.cc pathing.cc)
target_link_libraries(simulator PRIVATE utils)
