#pragma once

#include <utility>
#include <vector>
#include "splines.h"

namespace pathing {

const int velocity_lookahead = 5;
const double max_velocity = 1.0;
const double max_accel = 1.0;

auto CreateVelocityProfile(const SplineResult& result, double maxVelocity,
                           double maxAcceleration)
    -> std::vector<std::pair<double, double>>;

auto CreateVelocityProfile(const SplineResult& result)
    -> std::vector<std::pair<double, double>>;

}  // namespace pathing
