#include "velocity_profile.h"
#include <algorithm>
#include <cmath>

namespace pathing {

auto _CreateVelocityProfile(const SplineResult& result, double maxVelocity,
                            double maxAcceleration)
    -> std::vector<std::pair<double, double>> {
  if (result.points.empty() || result.params.empty() || maxVelocity <= 0.0 ||
      maxAcceleration <= 0.0) {
    return {};
  }

  std::vector<double> distances;
  distances.reserve(result.points.size());
  distances.emplace_back(0.0);

  for (int i = 1; i < static_cast<int>(result.points.size()); ++i) {
    frc::Translation2d prev(result.points[i - 1].X(), result.points[i - 1].Y());
    frc::Translation2d current(result.points[i].X(), result.points[i].Y());
    double distance = current.Distance(prev).value();
    distances.emplace_back(distances.back() + distance);
  }

  double total_distance = distances.back();
  if (total_distance <= 0.0) {
    return {};
  }

  double ramp_distance = maxVelocity * maxVelocity / (2.0 * maxAcceleration);
  double profile_velocity = maxVelocity;
  if (ramp_distance * 2.0 > total_distance) {
    ramp_distance = total_distance / 2.0;
    profile_velocity = std::sqrt(total_distance * maxAcceleration);
  }

  std::vector<std::pair<double, double>> velocities;
  velocities.reserve(result.params.size());

  for (int i = 0; i < static_cast<int>(result.params.size()); ++i) {
    double distance =
        distances[std::min(i, static_cast<int>(distances.size()) - 1)];
    double end_distance = total_distance - distance;
    double speed = profile_velocity;

    if (distance < ramp_distance) {
      speed = std::sqrt(2.0 * maxAcceleration * distance);
    }
    if (end_distance < ramp_distance) {
      speed = std::min(speed, std::sqrt(2.0 * maxAcceleration * end_distance));
    }

    auto [dx, dy] = EvaluateDerivative(result.params[i], result.controls,
                                       result.knots, result.p, 1);
    double derivative = std::hypot(dx, dy);
    if (derivative <= 0.0) {
      velocities.emplace_back(0.0, 0.0);
    } else {
      velocities.emplace_back(dx / derivative * speed, dy / derivative * speed);
    }
  }

  return velocities;
}

auto CreateVelocityProfile(const SplineResult& result)
    -> std::vector<std::pair<double, double>> {
  return _CreateVelocityProfile(result, max_velocity, max_accel);
}

}  // namespace pathing
