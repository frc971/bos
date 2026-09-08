#pragma once

#include "ellipse.h"
#include "pathfinding.h"
#include <vector>
#include <utility>

namespace pathing {

// Generates control points using gradient descent on a cost function combining
// path length (distance from consecutive points) and field density.
std::vector<std::pair<double, double>> OptimizePathWithGradientDescent(
    Point startGrid, 
    Point endGrid, 
    const std::vector<EllipseRegion>& ellipses, 
    const std::vector<std::vector<Node>>& grid, 
    double nodeSizeMeters, 
    int num_points = 20,
    int iterations = 100);

} // namespace pathing
