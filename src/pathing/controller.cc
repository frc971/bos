#include <frc/geometry/Translation3d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>
#include <chrono>
#include <cmath>
#include "splines.h"
#include "src/localization/position_receiver.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"
#include "velocity_profile.h"

namespace pathing {

auto RunController(
    const std::stop_token& stop_token,
    const std::string& navgrid_path = "/root/bos/constants/navgrid.json",
    bool verbose = false) -> void {

  std::ifstream file(navgrid_path);
  if (!file.is_open()) {
    LOG(FATAL) << "Failed to open navgrid.json";
    return;
  }

  nlohmann::json data = nlohmann::json::parse(file);
  file.close();

  const int GRID_H = data["grid"].size();
  const int GRID_W = data["grid"][0].size();
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
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  auto current_sub = localization::PositionReceiver();
  auto target_sub =
      inst.GetStructTopic<frc::Pose2d>("/pathing/target").Subscribe({});

  auto enabled_sub = inst.GetBooleanTopic("/pathing/enabled").Subscribe(false);

  auto vx_pub = inst.GetDoubleTopic("/pathing/vx").Publish();
  auto vy_pub = inst.GetDoubleTopic("/pathing/vy").Publish();
  auto isDone_pub = inst.GetBooleanTopic("/pathing/isDone").Publish();
  SplineResult result;
  std::vector<std::pair<double, double>> velocity_profile;
  int prev_closest_idx = -1;

  const double kp = 10.0;  // TODO: tune this on the real robot
  const double errorThreshold =
      0.4;  // cross-track error (m) before replanning

  while (!stop_token.stop_requested()) {
    if (!enabled_sub.Get()) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);

      result.points.clear();
      velocity_profile.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    frc::Pose2d target_pose = target_sub.Get();
    frc::Pose2d current_pose = current_sub.Get();

    Point start_pt{
        .x = static_cast<uint>(current_pose.X().value() / nodeSizeMeters),
        .y = static_cast<uint>(current_pose.Y().value() / nodeSizeMeters)};
    Point target_pt{
        .x = static_cast<uint>(target_pose.X().value() / nodeSizeMeters),
        .y = static_cast<uint>(target_pose.Y().value() / nodeSizeMeters)};

    start_pt.x = std::clamp(static_cast<int>(start_pt.x), 0, GRID_W - 1);

    start_pt.y = std::clamp(static_cast<int>(start_pt.y), 0, GRID_H - 1);

    target_pt.x = std::clamp(static_cast<int>(target_pt.x), 0, GRID_W - 1);

    target_pt.y = std::clamp(static_cast<int>(target_pt.y), 0, GRID_H - 1);

    frc::Translation2d t2d(target_pose.X(), target_pose.Y());
    if (current_pose.Translation().Distance(t2d).value() <
        nodeSizeMeters * 0.15) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      isDone_pub.Set(true);
      inst.Flush();
      result.points.clear();
      velocity_profile.clear();
      prev_closest_idx = -1;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    isDone_pub.Set(false);

    if (result.points.empty()) {
      result = CreateSpline(grid, start_pt, target_pt, nodeSizeMeters, 1000);
      velocity_profile = CreateVelocityProfile(result);
      prev_closest_idx = -1;
      if (!result.points.empty()) {
        if (verbose) {
          for (const auto& p : result.points) {
            LOG(INFO) << "spline pt: " << p.X().value() << " " << p.Y().value();
          }
        }
      }
    }

    if (result.points.empty() || velocity_profile.empty()) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;

    } else {

      int start_idx = std::max(0, prev_closest_idx);
      int closest_idx = start_idx;

      frc::Translation2d start2d(result.points[start_idx].X(),
                                 result.points[start_idx].Y());
      double best_dist = current_pose.Translation().Distance(start2d).value();
      for (int i = start_idx + 1; i < (int)result.points.size(); ++i) {
        frc::Translation2d t2d(result.points[i].X(), result.points[i].Y());
        double d = current_pose.Translation().Distance(t2d).value();
        if (d < best_dist) {
          best_dist = d;
          closest_idx = i;
        }
        if (verbose) {
          LOG(INFO) << "d: " << d << " i: " << i;
        }
      }

      if (verbose) {
        LOG(INFO) << "Closeset idx: " << closest_idx
                  << " Spline size: " << result.points.size();
      }

      if (prev_closest_idx >= 0 && closest_idx < prev_closest_idx) {
        vx_pub.Set(0.0);
        vy_pub.Set(0.0);
        inst.Flush();
        result.points.clear();
        velocity_profile.clear();
        prev_closest_idx = -1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      // replan from the current pose if we've drifted too far off the path
      if (best_dist > errorThreshold) {
        result.points.clear();
        velocity_profile.clear();
        prev_closest_idx = -1;
        continue;
      }

      prev_closest_idx = closest_idx;

      if (closest_idx >= static_cast<int>(result.params.size())) {
        closest_idx = static_cast<int>(result.params.size()) - 1;
      }

      // skip index 0 since its speed is 0
      int vidx = std::min(std::max(closest_idx, 1),
                          static_cast<int>(velocity_profile.size()) - 1);
      double cx = result.points[closest_idx].X().value();
      double cy = result.points[closest_idx].Y().value();
      double vx =
          velocity_profile[vidx].first + kp * (cx - current_pose.X().value());
      double vy =
          velocity_profile[vidx].second + kp * (cy - current_pose.Y().value());

      vx_pub.Set(vx);
      vy_pub.Set(vy);
      if (verbose) {
        LOG(INFO) << "set vx " << vx << " vy " << vy;
      }
      inst.Flush();
    }
  }
}

}  // namespace pathing
