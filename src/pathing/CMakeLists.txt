add_library(pathing)
target_sources(pathing PRIVATE
  pathfinding.cc
  splines.cc
  spline_follower.cc
  controller.cc
)
target_link_libraries(pathing PRIVATE
  utils
  localization
)
target_link_libraries(pathing PUBLIC
  wpilibc
  wpimath
)

add_executable(pathing_simulator simulator.cc)
target_link_libraries(pathing_simulator PRIVATE pathing nlohmann_json::nlohmann_json)
