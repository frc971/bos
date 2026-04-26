#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogWriter.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include "pathfinding.h"
#include "splines.h"

auto main() -> int {
  const std::string navgrid_path = "/root/bos/constants/navgrid.json";
  const frc::Pose2d start{1_m, 1_m, frc::Rotation2d{}};
  const frc::Pose2d target{14_m, 6_m, frc::Rotation2d{}};

  constexpr double kDt = 0.02;
  constexpr double kSpeed = 1.0;
  constexpr int kLookahead = 5;
  constexpr int kMaxTicks = 30 * 50;

  std::ifstream file(navgrid_path);
  nlohmann::json data = nlohmann::json::parse(file);
  file.close();

  const int GRID_H = data["grid"].size();
  const int GRID_W = data["grid"][0].size();
  const double nodeSize = data["nodeSizeMeters"];

  std::vector<std::vector<pathing::Node>> grid(
      GRID_H, std::vector<pathing::Node>(GRID_W));
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      grid[y][x].x = x;
      grid[y][x].y = y;
      grid[y][x].obstacle = data["grid"][y][x];
    }
  }

  pathing::Point start_pt{
      static_cast<uint>(start.X().value() / nodeSize),
      static_cast<uint>(start.Y().value() / nodeSize)};
  pathing::Point target_pt{
      static_cast<uint>(target.X().value() / nodeSize),
      static_cast<uint>(target.Y().value() / nodeSize)};

  pathing::SplineResult spline =
      pathing::createSpline(grid, start_pt, target_pt, nodeSize);
  if (spline.points.empty()) {
    return 1;
  }

  std::remove("/root/bos/logs/sim.wpilog");
  std::error_code ec;
  wpi::log::DataLogWriter log{"/root/bos/logs/sim.wpilog", ec};
  if (ec) return 1;
  wpi::log::StructLogEntry<frc::Pose2d> pose_log(log, "/sim/Pose");
  wpi::log::DoubleLogEntry vx_log(log, "/sim/Vx");
  wpi::log::DoubleLogEntry vy_log(log, "/sim/Vy");

  double x = start.X().value();
  double y = start.Y().value();
  int64_t t_us = 0;

  for (int tick = 0; tick < kMaxTicks; ++tick) {
    pose_log.Append({units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{}},
                    t_us);

    if (std::hypot(target.X().value() - x, target.Y().value() - y) < nodeSize) {
      break;
    }

    int closest = 0;
    double best = std::hypot(spline.points[0].X().value() - x,
                             spline.points[0].Y().value() - y);
    for (int i = 1; i < (int)spline.points.size(); ++i) {
      double d = std::hypot(spline.points[i].X().value() - x,
                            spline.points[i].Y().value() - y);
      if (d < best) {
        best = d;
        closest = i;
      }
    }

    int idx = std::min(closest + kLookahead, (int)spline.points.size() - 1);
    auto [dx_raw, dy_raw] = pathing::EvaluatePosition(
        spline.params[idx], spline.first_deriv_controls, spline.knots,
        spline.p - 1);
    double dx = dx_raw * spline.p;
    double dy = dy_raw * spline.p;
    double mag = std::hypot(dx, dy);
    if (mag < 1e-6) break;

    double vx = (dx / mag) * kSpeed;
    double vy = (dy / mag) * kSpeed;
    vx_log.Append(vx, t_us);
    vy_log.Append(vy, t_us);

    x += vx * kDt;
    y += vy * kDt;
    t_us += static_cast<int64_t>(kDt * 1'000'000);
  }

  log.Flush();
  return 0;
}
