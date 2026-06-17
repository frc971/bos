#pragma once

#include <utility>
#include <vector>
#include "splines.h"

namespace pathing {

// Gotten from current pathplanner config of team
const double max_velocity = 3.5;
const double max_accel = 3.2;

auto _CreateVelocityProfile(const SplineResult& result, double maxVelocity,
                            double maxAcceleration, double initialVelocity = 0.0)
    -> std::vector<std::pair<double, double>>;

// initialVelocity seeds the start of the profile so a replan doesn't force the
// robot to re-accelerate from rest (defaults to 0 = start from a standstill).
auto CreateVelocityProfile(const SplineResult& result,
                           double initialVelocity = 0.0)
    -> std::vector<std::pair<double, double>>;

}  // namespace pathing
