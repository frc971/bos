add_library(pathing)
target_sources(pathing PRIVATE 
  pathfinding.cc
  splines.cc
  controller.cc
  velocity_profile.cc
)
target_link_libraries(pathing PRIVATE
  utils
  localization
)

add_executable(simulator simulator.cc)
target_link_libraries(simulator PRIVATE pathing utils)
