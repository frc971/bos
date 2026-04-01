#include <climits>
#include <vector>
#include <frc/geometry/Pose2d.h>
#include "pathfinding.h"

#pragma once

namespace pathing {

auto knot_vector(int n, int p) -> std::vector<double>;

auto basis(int i, int p, double t, const std::vector<double>& knots) -> double;

auto evaluate(double t, const std::vector<std::pair<double, double>>& controls,
              const std::vector<double>& knots, int p)
    -> std::pair<double, double>;

auto createSpline(std::vector<std::vector<Node>> grid, Point start_point,
                  Point target_point, double nodeSizeMeters)
    -> std::vector<frc::Pose2d>;

}  // namespace pathing
