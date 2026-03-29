add_library(pathing pathing.cc)
target_link_libraries(pathing wpilibc absl::log)

add_executable(simulator simulator.cc)
target_link_libraries(simulator PRIVATE utils wpilibc pathing)
