#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogBackgroundWriter.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>
#include "src/pathing/splines.h"

auto main() -> int {
  const uint lookahead_ = 5;
  constexpr int64_t k_dt_us = 20'000;
  constexpr double k_dt_sec = 0.02;

  std::ifstream file("/bos/constants/navgrid.json");
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
  int target_x = 22;
  int target_y = 14;
  start_x = std::clamp(start_x, 0, grid_w - 1);
  start_y = std::clamp(start_y, 0, grid_h - 1);
  target_x = std::clamp(target_x, 0, grid_w - 1);
  target_y = std::clamp(target_y, 0, grid_h - 1);

  frc::Pose2d initial_pose(units::meter_t{start_x * node_size_meters},
                           units::meter_t{start_y * node_size_meters},
                           units::radian_t{0.0});
  const frc::Pose2d target_pose(units::meter_t{target_x * node_size_meters},
                                units::meter_t{target_y * node_size_meters},
                                units::radian_t{0.0});

  std::vector<std::pair<double, double>> velocities;
  pathing::Point start_pt{.x = static_cast<uint>(start_x),
                          .y = static_cast<uint>(start_y)};
  pathing::Point target_pt{.x = static_cast<uint>(target_x),
                           .y = static_cast<uint>(target_y)};
  pathing::SplineResult result =
      pathing::createSpline(grid, start_pt, target_pt, node_size_meters);
  if (result.points.empty()) {
    velocities.emplace_back(0.0, 0.0);
  } else {
    velocities.reserve(result.params.size());
    for (int i = 0; i < static_cast<int>(result.params.size()); ++i) {
      const int idx =
          std::min(i + static_cast<int>(lookahead_),
                   static_cast<int>(result.params.size()) - 1);
      const auto [raw_vx, raw_vy] =
          pathing::EvaluateDerivative(result.params[idx], result.controls,
                                      result.knots, result.p, 1);

      const int prev_idx = std::max(idx - 1, 0);
      const int next_idx =
          std::min(idx + 1, static_cast<int>(result.params.size()) - 1);
      const double param_delta =
          std::max(result.params[next_idx] - result.params[prev_idx], 1e-9);
      const double sample_dt = (next_idx - prev_idx) * k_dt_sec;
      const double scale = sample_dt > 0.0 ? (param_delta / sample_dt) : 0.0;

      velocities.emplace_back(raw_vx * scale, raw_vy * scale);
    }
  }

  wpi::log::DataLogBackgroundWriter log{"logs", "sim.wpilog"};
  log.AddStructSchema<frc::Pose2d>();
  wpi::log::StructLogEntry<frc::Pose2d> pose_log(log, "/sim/Pose2d");
  wpi::log::StructLogEntry<frc::Pose2d> target_log(log, "/sim/TargetPose2d");
  wpi::log::DoubleLogEntry vx_log(log, "/sim/pathing/vx");
  wpi::log::DoubleLogEntry vy_log(log, "/sim/pathing/vy");

  target_log.Append(target_pose, 1);
  frc::Pose2d replay_pose = initial_pose;
  int64_t t = 0;
  for (const auto& [vx, vy] : velocities) {
    vx_log.Append(vx, t);
    vy_log.Append(vy, t);
    pose_log.Append(replay_pose, t);
    replay_pose =
        frc::Pose2d(units::meter_t{replay_pose.X().value() + vx * k_dt_sec},
                    units::meter_t{replay_pose.Y().value() + vy * k_dt_sec},
                    units::radian_t{0.0});
    t += k_dt_us;
  }

  log.Flush();
  return 0;
}
