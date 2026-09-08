#pragma once

#include <vector>
#include <random>
#include <cmath>
#include "pathfinding.h"

namespace pathing {

enum class RobotState {
  INTAKING,
  SHOOTING
};

struct EllipseRegion {
  double centerX;
  double centerY;
  double semimajorAxis;
  double semiminorAxis;
  double angle; // in radians
  double density; // 0.0 to 1.0

  // Optional bounding box or sampling utility
  std::vector<Point> SamplePoints(int numPoints, double nodeSizeMeters) const {
    std::vector<Point> points;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distRadius(0, 1.0);
    std::uniform_real_distribution<> distAngle(0, 2 * M_PI);

    int actualPoints = static_cast<int>(numPoints * density);
    if (actualPoints <= 0) actualPoints = 1;

    for (int i = 0; i < actualPoints; ++i) {
      double r = std::sqrt(distRadius(gen));
      double theta = distAngle(gen);

      double x_ell = r * semimajorAxis * std::cos(theta);
      double y_ell = r * semiminorAxis * std::sin(theta);

      // Rotate and translate
      double x = centerX + x_ell * std::cos(angle) - y_ell * std::sin(angle);
      double y = centerY + x_ell * std::sin(angle) + y_ell * std::cos(angle);

      uint gridX = static_cast<uint>(std::max(0.0, x / nodeSizeMeters));
      uint gridY = static_cast<uint>(std::max(0.0, y / nodeSizeMeters));
      points.push_back({gridX, gridY});
    }

    return points;
  }
};

} // namespace pathing
