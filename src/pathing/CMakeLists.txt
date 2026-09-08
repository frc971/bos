add_library(pathing)
target_sources(pathing PRIVATE 
  pathfinding.cc
  splines.cc
  controller.cc
  velocity_profile.cc
  path_follower.cc
  cluster_follower.cc
)
target_link_libraries(pathing PRIVATE
  utils
  localization
)

add_executable(simulator simulator.cc)
target_link_libraries(simulator PRIVATE pathing utils)

add_executable(ellipse_simulator ellipse_simulator.cc)
target_link_libraries(ellipse_simulator PRIVATE pathing utils)
