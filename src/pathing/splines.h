#pragma once

#include <frc/geometry/Pose2d.h>
#include <climits>
#include <vector>
#include "pathfinding.h"

namespace pathing {

struct SplineResult {
  std::vector<frc::Pose2d> points;
  std::vector<std::pair<double, double>> controls;
  std::vector<double> knots;
  std::vector<double> params;
  uint p;
};

auto KnotVector(int n, int p) -> std::vector<double>;

auto FiniteDifferences(const std::vector<std::pair<double, double>>& controls,
                       const std::vector<double>& knots, int p, int k)
    -> std::vector<std::pair<double, double>>;

auto basis(int i, int p, double t, const std::vector<double>& knots) -> double;

auto EvaluatePosition(double t,
                      const std::vector<std::pair<double, double>>& controls,
                      const std::vector<double>& knots, int p)
    -> std::pair<double, double>;
auto EvaluateDerivative(double t,
                        const std::vector<std::pair<double, double>>& controls,
                        const std::vector<double>& knots, int p, int k)
    -> std::pair<double, double>;

auto createSpline(const std::vector<std::vector<Node>>& grid, Point start_point,
                  Point target_point, double nodeSizeMeters) -> SplineResult;

}  // namespace pathing
