#include "splines.h"
#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <vector>
#include "pathfinding.h"
#include "src/utils/log.h"

namespace pathing {

auto KnotVector(int n, int p) -> std::vector<double> {
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

auto FiniteDifferences(const std::vector<std::pair<double, double>>& controls,
                       const std::vector<double>& knots, int p, int k)
    -> std::vector<std::pair<double, double>> {
  if (k == 0) {
    return controls;
  }

  std::vector<std::pair<double, double>> ret;
  ret.reserve(controls.size() - 1);

  std::vector<std::pair<double, double>> prev =
      FiniteDifferences(controls, knots, p, k - 1);

  for (int i = 0; i < (int)prev.size() - 1; ++i) {
    double denom = knots[i + p + 1] - knots[i + k];
    if (denom != 0) {
      double x = (prev[i + 1].first - prev[i].first) / denom;
      double y = (prev[i + 1].second - prev[i].second) / denom;
      ret.emplace_back(x, y);
    } else {
      ret.emplace_back(0.0, 0.0);
    }
  }

  return ret;
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

auto EvaluatePosition(double t,
                      const std::vector<std::pair<double, double>>& controls,
                      const std::vector<double>& knots, int p)
    -> std::pair<double, double> {
  if (t >= knots.back()) {
    return controls.back();
  }

  int n = static_cast<int>(controls.size());
  double x = 0.0, y = 0.0;
  for (int i = 0; i < n; ++i) {
    double b = basis(i, p, t, knots);
    x += b * controls[i].first;
    y += b * controls[i].second;
  }
  return {x, y};
}

auto EvaluateDerivative(double t,
                        const std::vector<std::pair<double, double>>& controls,
                        const std::vector<double>& knots, int p, int k)
    -> std::pair<double, double> {
  std::vector<std::pair<double, double>> first_diffs =
      FiniteDifferences(controls, knots, p, k);
  auto [x, y] = EvaluatePosition(t, first_diffs, knots, p - k);

  int n = 1;
  for (int i = p; i >= p - k + 1; i--) {
    n *= i;
  }
  return {x * n, y * n};
}

auto createSpline(const std::vector<std::vector<pathing::Node>>& grid,
                  Point start_point, Point target_point, double nodeSizeMeters)
    -> SplineResult {

  std::vector<std::vector<pathing::Node>> gridCopy = grid;
  std::vector<pathing::Node> path = BFS(gridCopy, start_point, target_point);

  if (path.empty()) {
    LOG(INFO) << "BFS returned no path";
    return {};
  }

  std::vector<std::pair<double, double>> control_points;
  std::vector<double> knots;
  std::vector<frc::Pose2d> spline_points;
  std::vector<double> spline_params;
  uint p;

  control_points.reserve(path.size());
  for (const pathing::Node& node : path) {
    control_points.emplace_back(node.x * nodeSizeMeters,
                                node.y * nodeSizeMeters);
    uint numControls = control_points.size();

    if (numControls < 4) {
      return {};
    }

    p = 3;
    if (numControls <= p) {
      p = numControls - 1;
    }

    knots = KnotVector(numControls, p);

    for (int t = 0; t <= 1000; t += 1) {
      double t_real = t / 1000.0;
      auto [x, y] = EvaluatePosition(t_real, control_points, knots, p);
      spline_points.emplace_back(units::meter_t{x}, units::meter_t{y}, 0_rad);
      spline_params.emplace_back(t_real);
    }
  }
  return {spline_points, control_points, knots, spline_params, p};

}  // namespace pathing
}  // namespace pathing