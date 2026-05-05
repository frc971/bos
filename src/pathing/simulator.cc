#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogBackgroundWriter.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include "src/pathing/splines.h"

auto main() -> int {
  const uint lookahead_ = 0;
  const int64_t dt_us = 20000;
  const double dt_sec = 0.020;
  const double speed_ = 0.1;

  std::ifstream file("/root/bos/constants/navgrid.json");
  if (!file.is_open()) {
    return 1;
  }
  nlohmann::json data = nlohmann::json::parse(file);
  file.close();

  const int grid_h = data["grid"].size();
  const int grid_w = data["grid"][0].size();
  const double node_size_meters = data["nodeSizeMeters"];

  std::vector<std::vector<pathing::Node>> grid(
      grid_h, std::vector<pathing::Node>(grid_w));
  for (int y = 0; y < grid_h; ++y) {
    for (int x = 0; x < grid_w; ++x) {
      grid[y][x].x = x;
      grid[y][x].y = y;
      grid[y][x].obstacle = data["grid"][y][x];
    }
  }

  int start_x = 10;
  int start_y = 5;
  int target_x = 44;
  int target_y = 12;
  start_x = std::clamp(start_x, 0, grid_w - 1);
  start_y = std::clamp(start_y, 0, grid_h - 1);
  target_x = std::clamp(target_x, 0, grid_w - 1);
  target_y = std::clamp(target_y, 0, grid_h - 1);

  frc::Pose2d current_pose(units::meter_t{start_x * node_size_meters},
                           units::meter_t{start_y * node_size_meters},
                           units::radian_t{0.0});
  const frc::Pose2d target_pose(units::meter_t{target_x * node_size_meters},
                                units::meter_t{target_y * node_size_meters},
                                units::radian_t{0.0});

  wpi::log::DataLogBackgroundWriter log{"logs", "sim.wpilog"};
  log.AddStructSchema<frc::Pose2d>();
  wpi::log::StructLogEntry<frc::Pose2d> pose_log(log, "/sim/Pose2d");
  wpi::log::StructLogEntry<frc::Pose2d> target_log(log, "/sim/TargetPose2d");
  wpi::log::DoubleLogEntry vx_log(log, "/sim/pathing/vx");
  wpi::log::DoubleLogEntry vy_log(log, "/sim/pathing/vy");
  target_log.Append(target_pose, 1);

  pathing::SplineResult result;
  constexpr int k_max_steps = 10000;
  int64_t t = 0;
  int prev_closest_idx = -1;
  for (int step = 0; step < k_max_steps; ++step) {
    pathing::Point start_pt{
        .x = static_cast<uint>(current_pose.X().value() / node_size_meters),
        .y = static_cast<uint>(current_pose.Y().value() / node_size_meters)};
    pathing::Point target_pt{
        .x = static_cast<uint>(target_pose.X().value() / node_size_meters),
        .y = static_cast<uint>(target_pose.Y().value() / node_size_meters)};
    start_pt.x = std::clamp(static_cast<int>(start_pt.x), 0, grid_w - 1);
    start_pt.y = std::clamp(static_cast<int>(start_pt.y), 0, grid_h - 1);
    target_pt.x = std::clamp(static_cast<int>(target_pt.x), 0, grid_w - 1);
    target_pt.y = std::clamp(static_cast<int>(target_pt.y), 0, grid_h - 1);

    frc::Translation2d t2d(target_pose.X(), target_pose.Y());
    double dist_to_target = current_pose.Translation().Distance(t2d).value();
    if (dist_to_target < node_size_meters) {
      vx_log.Append(0.0, t);
      vy_log.Append(0.0, t);
      pose_log.Append(current_pose, t);
      break;
    }

    if (result.points.empty()) {
      result =
          pathing::createSpline(grid, start_pt, target_pt, node_size_meters);
    }

    if (result.points.empty()) {
      vx_log.Append(0.0, t);
      vy_log.Append(0.0, t);
      pose_log.Append(current_pose, t);
      break;
    }

    int closest_idx = 0;
    frc::Translation2d first2d(result.points[0].X(), result.points[0].Y());
    double best_dist = current_pose.Translation().Distance(first2d).value();
    for (int i = 1; i < static_cast<int>(result.points.size()); ++i) {
      frc::Translation2d p(result.points[i].X(), result.points[i].Y());
      double d = current_pose.Translation().Distance(p).value();
      if (d < best_dist) {
        best_dist = d;
        closest_idx = i;
      }
    }

    if (prev_closest_idx >= 0 && closest_idx < prev_closest_idx) {
      vx_log.Append(0.0, t);
      vy_log.Append(0.0, t);
      pose_log.Append(current_pose, t);
      break;
    }
    if (prev_closest_idx >= 0 && closest_idx == prev_closest_idx &&
        closest_idx >= static_cast<int>(result.params.size()) - 5) {
      double dx = target_pose.X().value() - current_pose.X().value();
      double dy = target_pose.Y().value() - current_pose.Y().value();
      double dist = std::hypot(dx, dy);

      while (dist > 0.001) {
        double vx = (dx / dist) * speed_;
        double vy = (dy / dist) * speed_;

        vx_log.Append(vx, t);
        vy_log.Append(vy, t);
        pose_log.Append(current_pose, t);

        current_pose =
            frc::Pose2d(units::meter_t{current_pose.X().value() + vx * dt_sec},
                        units::meter_t{current_pose.Y().value() + vy * dt_sec},
                        units::radian_t{0.0});
        t += dt_us;
        dx = target_pose.X().value() - current_pose.X().value();
        dy = target_pose.Y().value() - current_pose.Y().value();
        dist = std::hypot(dx, dy);
      }
      vx_log.Append(0.0, t);
      vy_log.Append(0.0, t);
      pose_log.Append(current_pose, t);
      break;
    }

    prev_closest_idx = closest_idx;

    closest_idx += lookahead_;
    if (closest_idx >= static_cast<int>(result.params.size())) {
      closest_idx = static_cast<int>(result.params.size()) - 1;
    }

    auto [vx, vy] = pathing::EvaluateDerivative(
        result.params[closest_idx], result.controls, result.knots, result.p, 1);
    vx *= speed_;
    vy *= speed_;

    vx_log.Append(vx, t);
    vy_log.Append(vy, t);
    pose_log.Append(current_pose, t);

    current_pose =
        frc::Pose2d(units::meter_t{current_pose.X().value() + vx * dt_sec},
                    units::meter_t{current_pose.Y().value() + vy * dt_sec},
                    units::radian_t{0.0});
    t += dt_us;
  }

  log.Flush();
  return 0;
}
