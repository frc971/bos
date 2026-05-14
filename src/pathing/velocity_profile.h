#pragma once

#include <utility>
#include <vector>
#include "splines.h"

namespace pathing {

const int velocity_lookahead = 5;
// Gotten from current pathplanner config of team
const double max_velocity = 3.5;
const double max_accel = 3.2;

auto _CreateVelocityProfile(const SplineResult& result, double maxVelocity,
                            double maxAcceleration)
    -> std::vector<std::pair<double, double>>;

auto CreateVelocityProfile(const SplineResult& result)
    -> std::vector<std::pair<double, double>>;

}  // namespace pathing
