#pragma once

#include <frc/geometry/Pose2d.h>
#include <climits>
#include <vector>
#include "pathfinding.h"

namespace pathing {

auto KnotVector(int n, int p) -> std::vector<double>;

auto basis(int i, int p, double t, const std::vector<double>& knots) -> double;

auto evaluate(double t, const std::vector<std::pair<double, double>>& controls,
              const std::vector<double>& knots, int p)
    -> std::pair<double, double>;

auto createSpline(const std::vector<std::vector<Node>>& grid, Point start_point,
                  Point target_point, double nodeSizeMeters)
    -> std::vector<frc::Pose2d>;

}  // namespace pathing
