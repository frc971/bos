#pragma once

#include <frc/geometry/Pose2d.h>
#include <optional>
#include <utility>
#include <vector>
#include "pathfinding.h"
#include "splines.h"

namespace pathing {

struct FollowerOutput {
  double vx = 0.0;
  double vy = 0.0;
  bool done = false;
};

using GridType = std::vector<std::vector<Node>>;
using ProfileType = std::vector<std::pair<double, double>>;

class PathFollower {
 public:
  PathFollower(const std::vector<std::vector<Node>>& grid,
               double nodeSizeMeters, double kp, double errorThreshold,
               int sampleCount);

  auto update(const frc::Pose2d& current_pose, const frc::Pose2d& target_pose)
      -> FollowerOutput;

  auto reset() -> void;

 private:
  auto plan(const frc::Pose2d& current_pose, const frc::Pose2d& target_pose)
      -> void;

  const GridType& grid_;
  double nodeSizeMeters_;
  double kp_;
  double errorThreshold_;
  int sampleCount_;
  std::optional<SplineResult> spline_;
  std::optional<std::vector<std::pair<double, double>>> velocity_profile_;
  std::optional<uint> prev_closest_idx_;
};

}  // namespace pathing
