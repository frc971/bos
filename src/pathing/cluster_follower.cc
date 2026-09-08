#include "cluster_follower.h"
#include <algorithm>
#include <cmath>

namespace pathing {

static double clamp(double v, double minv, double maxv) {
  if (v < minv) return minv;
  if (v > maxv) return maxv;
  return v;
}

std::vector<std::pair<double, double>> OptimizePathWithGradientDescent(
    Point startGrid, 
    Point endGrid, 
    const std::vector<EllipseRegion>& ellipses, 
    const std::vector<std::vector<Node>>& grid, 
    double nodeSizeMeters, 
    int num_points,
    int iterations) {
  
  std::vector<std::pair<double, double>> path;
  
  // Initialize with a straight line
  double startX = startGrid.x * nodeSizeMeters;
  double startY = startGrid.y * nodeSizeMeters;
  double endX = endGrid.x * nodeSizeMeters;
  double endY = endGrid.y * nodeSizeMeters;
  
  for (int i = 0; i < num_points; ++i) {
    double t = i / static_cast<double>(num_points - 1);
    path.push_back({
      startX + t * (endX - startX),
      startY + t * (endY - startY)
    });
  }
  
  double alpha = 0.5; // Smoothness factor
  double beta = 2.0;  // Density factor
  double learning_rate = 0.1;
  
  for (int iter = 0; iter < iterations; ++iter) {
    std::vector<std::pair<double, double>> next_path = path;
    
    for (int i = 1; i < num_points - 1; ++i) {
      double px = path[i].first;
      double py = path[i].second;
      
      // 1. Smoothness gradient (pulls point towards midpoint of neighbors)
      double smooth_grad_x = 2 * px - path[i-1].first - path[i+1].first;
      double smooth_grad_y = 2 * py - path[i-1].second - path[i+1].second;
      
      // 2. Density gradient
      double density_grad_x = 0;
      double density_grad_y = 0;
      
      for (const auto& el : ellipses) {
        // Simple density field: gaussian centered on ellipse
        double dx = px - el.centerX;
        double dy = py - el.centerY;
        
        // Un-rotate point to ellipse frame for easier calculation
        double cos_a = std::cos(-el.angle);
        double sin_a = std::sin(-el.angle);
        double dx_rot = dx * cos_a - dy * sin_a;
        double dy_rot = dx * sin_a + dy * cos_a;
        
        double norm_dist = (dx_rot * dx_rot) / (el.semimajorAxis * el.semimajorAxis) + 
                           (dy_rot * dy_rot) / (el.semiminorAxis * el.semiminorAxis);
                           
        if (norm_dist < 2.0) { // Influence radius
            double weight = el.density * std::exp(-norm_dist);
            // Gradient of density
            double grad_x_local = -2.0 * dx_rot / (el.semimajorAxis * el.semimajorAxis) * weight;
            double grad_y_local = -2.0 * dy_rot / (el.semiminorAxis * el.semiminorAxis) * weight;
            
            // Rotate back
            density_grad_x += grad_x_local * std::cos(el.angle) - grad_y_local * std::sin(el.angle);
            density_grad_y += grad_x_local * std::sin(el.angle) + grad_y_local * std::cos(el.angle);
        }
      }
      
      // 3. Obstacle gradient (repel from nearest obstacle)
      double obs_grad_x = 0;
      double obs_grad_y = 0;
      int gridX = static_cast<int>(px / nodeSizeMeters);
      int gridY = static_cast<int>(py / nodeSizeMeters);
      if (gridY >= 0 && gridY < static_cast<int>(grid.size()) && 
          gridX >= 0 && gridX < static_cast<int>(grid[0].size())) {
          if (grid[gridY][gridX].obstacle) {
              // Push away
              obs_grad_x = (std::rand() % 100 - 50) / 100.0;
              obs_grad_y = (std::rand() % 100 - 50) / 100.0;
          }
      }
      
      double tot_grad_x = alpha * smooth_grad_x - beta * density_grad_x - 5.0 * obs_grad_x;
      double tot_grad_y = alpha * smooth_grad_y - beta * density_grad_y - 5.0 * obs_grad_y;
      
      next_path[i].first -= learning_rate * tot_grad_x;
      next_path[i].second -= learning_rate * tot_grad_y;
      
      // Bound the point inside the grid
      next_path[i].first = clamp(next_path[i].first, 0.0, grid[0].size() * nodeSizeMeters - 0.01);
      next_path[i].second = clamp(next_path[i].second, 0.0, grid.size() * nodeSizeMeters - 0.01);
    }
    path = next_path;
  }
  
  return path;
}

} // namespace pathing
