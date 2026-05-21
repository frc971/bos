add_library(pathing)
target_sources(pathing PRIVATE 
  pathfinding.cc
  splines.cc 
  controller.cc
)
target_link_libraries(pathing PRIVATE 
  utils 
  localization
)
