# Pathing

This directory contains grid pathfinding, spline generation, and a NetworkTables-driven path controller.

## Files

- `CMakeLists.txt` builds the `pathing` library.
- `pathfinding.h` / `pathfinding.cc` define grid `Node` and `Point` structures and implement `pathing::BFS`, which searches an obstacle grid and returns a node path.
- `splines.h` / `splines.cc` provide B-spline helpers (`KnotVector`, `basis`, `evaluate`) and `createSpline`, which converts a BFS path into smooth `frc::Pose2d` waypoints.
- `controller.h` / `controller.cc` implement `RunController`, which reads a navigation grid, receives current and target poses through NetworkTables, generates a spline path, and publishes velocity commands plus the next path pose.
- `simulator.cc` is a standalone path-following simulation experiment that logs pose, velocity, and acceleration data. It is not currently part of the `pathing` library target in this directory's CMake file.

## Main Types And Functions

- `pathing::Node` represents one grid cell, including traversal state, obstacle status, parent link, and path markers.
- `pathing::Point` is the integer grid coordinate used as BFS input.
- `pathing::BFS` finds a grid path between two points.
- `pathing::createSpline` turns that grid path into meter-space poses for motion.
- `pathing::RunController` connects path generation to robot pose and target topics.
