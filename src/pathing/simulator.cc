#include <frc/geometry/Pose2d.h>
#include <wpi/DataLogBackgroundWriter.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include "spline_follower.h"
#include "splines.h"

auto main() -> int {
  wpi::log::DataLogBackgroundWriter log{"/root/bos/logs", "sim.wpilog"};

  wpi::log::StructLogEntry<frc::Pose2d> poseLog(log, "/sim/Pose2d");
  wpi::log::DoubleLogEntry velXLog(log, "/sim/VelX");
  wpi::log::DoubleLogEntry velYLog(log, "/sim/VelY");

  std::ifstream file("/root/bos/constants/navgrid.json");
  if (!file.is_open()) {
    return 1;
  }

  nlohmann::json data = nlohmann::json::parse(file);
  file.close();

  const int GRID_W = data["grid"][0].size();
  const int GRID_H = data["grid"].size();
  double nodeSizeMeters = data["nodeSizeMeters"];

  std::vector<std::vector<pathing::Node>> grid(
      GRID_H, std::vector<pathing::Node>(GRID_W));
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      grid[y][x].x = x;
      grid[y][x].y = y;
      grid[y][x].obstacle = data["grid"][y][x];
    }
  }

  pathing::Point start_pt{10, 5};
  pathing::Point target_pt{45, 22};

  auto spline_points =
      pathing::createSpline(grid, start_pt, target_pt, nodeSizeMeters);
  if (spline_points.empty()) {
    return 1;
  }

  constexpr int kLookaheadOffset = 50;
  constexpr double kSpeed = 1.0;
  constexpr int64_t kDtUs = 20'000;
  constexpr double kDtSec = kDtUs / 1'000'000.0;

  double currentX = spline_points[0].X().value();
  double currentY = spline_points[0].Y().value();
  int64_t t = 0;

  frc::Translation2d target2d(spline_points.back().X(), spline_points.back().Y());

  while (true) {
    frc::Pose2d current_pose{units::meter_t{currentX}, units::meter_t{currentY},
                             frc::Rotation2d{}};

    if (current_pose.Translation().Distance(target2d).value() < nodeSizeMeters) {
      break;
    }

    poseLog.Append(current_pose, t);

    auto result = pathing::FollowSpline(current_pose, spline_points,
                                        kLookaheadOffset, kSpeed);

    currentX += result.vx * kDtSec;
    currentY += result.vy * kDtSec;

    velXLog.Append(result.vx, t);
    velYLog.Append(result.vy, t);

    t += kDtUs;
  }

  log.Flush();
  return 0;
}
