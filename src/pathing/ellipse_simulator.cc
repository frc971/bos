#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <cmath>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "src/pathing/path_follower.h"
#include "src/pathing/pathfinding.h"
#include "src/pathing/splines.h"
#include "src/pathing/velocity_profile.h"
#include "src/pathing/ellipse.h"
#include "src/pathing/cluster_follower.h"

namespace pathing {

int ELLIPSE_CELL_SIZE = 20;

auto drawObstacles(cv::Mat& canvas,
                   const std::vector<std::vector<Node>>& grid) {
  for (int y = 0; y < static_cast<int>(grid.size()); ++y) {
    for (int x = 0; x < static_cast<int>(grid[0].size()); ++x) {
      if (grid[y][x].obstacle) {
        cv::rectangle(
            canvas,
            cv::Rect(x * ELLIPSE_CELL_SIZE, y * ELLIPSE_CELL_SIZE, ELLIPSE_CELL_SIZE, ELLIPSE_CELL_SIZE),
            cv::Scalar(0, 0, 0), cv::FILLED);
      }
    }
  }
}

auto drawEllipses(cv::Mat& canvas, const std::vector<EllipseRegion>& ellipses, double nodeSizeMeters) {
  for (const auto& ellipse : ellipses) {
    cv::Point center(
        (ellipse.centerX / nodeSizeMeters) * ELLIPSE_CELL_SIZE,
        (ellipse.centerY / nodeSizeMeters) * ELLIPSE_CELL_SIZE);
    
    cv::Size axes(
        (ellipse.semimajorAxis / nodeSizeMeters) * ELLIPSE_CELL_SIZE,
        (ellipse.semiminorAxis / nodeSizeMeters) * ELLIPSE_CELL_SIZE);
    
    // Higher density = more saturated green
    int intensity = static_cast<int>(255 * ellipse.density);
    cv::ellipse(canvas, center, axes, ellipse.angle * 180.0 / M_PI, 0, 360, cv::Scalar(0, intensity, 0), 2);
  }
}

}  // namespace pathing

auto main() -> int {
  const auto& navgrid = pathing::GetGrid("/root/bos/constants/navgrid.json");
  const auto& grid = navgrid.grid;
  const auto& nodeSizeMeters = navgrid.nodeSizeMeters;
  cv::Mat canvas(static_cast<int>(grid.size()) * pathing::ELLIPSE_CELL_SIZE,
                 static_cast<int>(grid[0].size()) * pathing::ELLIPSE_CELL_SIZE,
                 CV_8UC3);
  canvas.setTo(cv::Scalar(255, 255, 255));
  
  std::vector<pathing::EllipseRegion> ellipses = {
      {10.0, 5.0, 4.0, 2.0, 0.5, 0.9}, // Neutral zone cluster 1
      {12.0, 10.0, 3.0, 1.5, -0.2, 0.7} // Neutral zone cluster 2
  };
  
  // Game Setup
  double autoTimeRemaining = 20.0; // 20 Seconds Max Auto!
  double estimatedAvgSpeed = 3.5; // m/s traversing the path
  
  pathing::RobotState state = pathing::RobotState::INTAKING;
  pathing::Point currentGridPos = {.x = 2, .y = 3}; // Inside start zone
  pathing::Point scoringGridPos = {.x = 2, .y = 4}; // The scoring target structure
  
  pathing::drawObstacles(canvas, grid);
  pathing::drawEllipses(canvas, ellipses, nodeSizeMeters);
  
  int cycleCount = 0;
  
  while (autoTimeRemaining > 0) {
      pathing::Point targetGridPos;
      cv::Scalar routeColor;
      
      if (state == pathing::RobotState::SHOOTING) {
          targetGridPos = scoringGridPos;
          routeColor = cv::Scalar(0, 0, 255); // Red path back to shoot
      } else {
          // Intake from best ellipse via simple density checking
          targetGridPos = {
              static_cast<uint>(ellipses[0].centerX / nodeSizeMeters),
              static_cast<uint>(ellipses[0].centerY / nodeSizeMeters)
          };
          routeColor = cv::Scalar(255, 0, 0); // Blue path out to neutral zone
      }
      
      // Calculate Active Contour 
      auto controlPoints = pathing::OptimizePathWithGradientDescent(
          currentGridPos, targetGridPos, ellipses, grid, nodeSizeMeters, 25, 200);

      // Interpolate smooth spline
      pathing::SplineResult splineOpt = pathing::CreateSplineFromControls(controlPoints, 200);
      
      if (splineOpt.points.empty()) break; // In case of pathing failure
      
      double timeDriven = 0.0;
      bool timeExpiredMidLeg = false;
      
      // Trace line across time limits
      for (size_t i = 1; i < splineOpt.points.size(); ++i) {
          auto p1 = splineOpt.points[i - 1];
          auto p2 = splineOpt.points[i];
          
          double dx = p2.X().value() - p1.X().value();
          double dy = p2.Y().value() - p1.Y().value();
          double distSegment = std::hypot(dx, dy);
          
          double tNeeded = distSegment / estimatedAvgSpeed;
          
          if (timeDriven + tNeeded > autoTimeRemaining) {
              timeExpiredMidLeg = true;
              break;
          }
          timeDriven += tNeeded;
          
          // Render driven piece
          cv::line(canvas, 
                 cv::Point((p1.X().value() / nodeSizeMeters) * pathing::ELLIPSE_CELL_SIZE, (p1.Y().value() / nodeSizeMeters) * pathing::ELLIPSE_CELL_SIZE),
                 cv::Point((p2.X().value() / nodeSizeMeters) * pathing::ELLIPSE_CELL_SIZE, (p2.Y().value() / nodeSizeMeters)* pathing::ELLIPSE_CELL_SIZE), routeColor, 3);
          
          currentGridPos = {
              static_cast<uint>(p2.X().value() / nodeSizeMeters),
              static_cast<uint>(p2.Y().value() / nodeSizeMeters)
          };
      }
      
      autoTimeRemaining -= timeDriven;
      
      if (timeExpiredMidLeg) {
          // Mark final expiration spot
          cv::circle(canvas, cv::Point(currentGridPos.x * pathing::ELLIPSE_CELL_SIZE, currentGridPos.y * pathing::ELLIPSE_CELL_SIZE), 8, cv::Scalar(0, 255, 255), cv::FILLED);
          break;
      }
      
      // Apply Interaction Time Penalties
      if (state == pathing::RobotState::SHOOTING) {
          autoTimeRemaining -= 1.0; // 1 second to fire
          state = pathing::RobotState::INTAKING;
          cycleCount++;
      } else {
          autoTimeRemaining -= 1.5; // 1.5 seconds to acquire gamepieces
          state = pathing::RobotState::SHOOTING;
      }
  }

  cv::imwrite("/tmp/ellipse_sim_gd.png", canvas);
  std::printf("Auto ended! Completed %d cycles.\n", cycleCount);
  return 0;
}
