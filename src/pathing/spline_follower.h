#pragma once

#include <frc/geometry/Pose2d.h>
#include <vector>

namespace pathing {

struct FollowResult {
  double vx;
  double vy;
  frc::Pose2d lookahead;
};

// Given the current pose and a precomputed spline, returns the velocity command
// and lookahead pose using a fixed lookahead index offset from the closest point.
auto FollowSpline(const frc::Pose2d& current_pose,
                  const std::vector<frc::Pose2d>& spline_points,
                  int lookahead_offset,
                  double speed,
                  bool verbose = false) -> FollowResult;

}  // namespace pathing
