#include "splines.h"
#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <vector>
#include "pathfinding.h"

namespace pathing {

auto knot_vector(int n, int p) -> std::vector<double> {
  int length = n + p + 1;

  std::vector<double> knots(length);

  for (int i = 1; i < length; ++i) {
    if (i < p + 1) {
      knots[i] = 0.0;
    } else if (i >= n) {
      knots[i] = 1.0;
    } else {
      knots[i] = static_cast<double>(i - p) / (n - p);
    }
  }

  return knots;
}

auto basis(int i, int p, double t, const std::vector<double>& knots) -> double {
  if (p == 0) {
    if (knots[i] <= t && t < knots[i + 1]) {
      return 1.0;
    } else {
      return 0.0;
    }
  }

  double left = 0.0;
  double denoml = knots[i + p] - knots[i];
  if (denoml != 0) {
    left = (t - knots[i]) / denoml * basis(i, p - 1, t, knots);
  }

  double right = 0.0;
  double denomr = knots[i + p + 1] - knots[i + 1];
  if (denomr != 0) {
    right = (knots[i + p + 1] - t) / denomr * basis(i + 1, p - 1, t, knots);
  }

  return left + right;
}

auto evaluate(double t, const std::vector<std::pair<double, double>>& controls,
              const std::vector<double>& knots, int p)
    -> std::pair<double, double> {
  int n = static_cast<int>(controls.size());
  double x = 0.0, y = 0.0;
  for (int i = 0; i < n; ++i) {
    double b = basis(i, p, t, knots);
    x += b * controls[i].first;
    y += b * controls[i].second;
  }
  return {x, y};
}

auto createSpline(const std::vector<std::vector<pathing::Node>>& grid,
                  Point start_point, Point target_point, double nodeSizeMeters)
    -> std::vector<frc::Pose2d> {

  std::vector<std::vector<pathing::Node>> gridCopy = grid;
  std::vector<pathing::Node> path = BFS(gridCopy, start_point, target_point);
  std::vector<std::pair<double, double>> control_points;
  control_points.reserve(path.size());
  for (const pathing::Node& node : path) {
    control_points.emplace_back(node.x * nodeSizeMeters,
                                node.y * nodeSizeMeters);
  }

  int p = 3;
  if (control_points.size() < 4) {
    p = control_points.size() - 1;
  }
  std::vector<double> knots = knot_vector(control_points.size(), p);

  std::vector<frc::Pose2d> spline_points;
  for (double t = 0; t <= 1; t += 0.01) {
    auto [x, y] = evaluate(t, control_points, knots, p);
    spline_points.emplace_back(units::meter_t{x}, units::meter_t{y}, 0_rad);
  }

  return spline_points;
}

}  // namespace pathing
