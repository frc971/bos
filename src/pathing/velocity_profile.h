#pragma once

#include <utility>
#include <vector>
#include "splines.h"

namespace pathing {

inline constexpr int kVelocityLookahead = 5;
inline constexpr double kMaxProfileVelocity = 1.0;
inline constexpr double kMaxProfileAcceleration = 1.0;

auto CreateVelocityProfile(const SplineResult& result, double maxVelocity,
                           double maxAcceleration)
    -> std::vector<std::pair<double, double>>;

auto CreateVelocityProfile(const SplineResult& result)
    -> std::vector<std::pair<double, double>>;

}  // namespace pathing
