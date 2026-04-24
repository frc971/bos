#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>
#include <chrono>
#include <cmath>
#include "splines.h"
#include "src/localization/position_receiver.h"
#include "src/pathing/splines.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"

namespace pathing {

auto RunController(
    const std::string& navgrid_path = "/root/bos/constants/navgrid.json",
    bool verbose = false) -> void {
  const double speed_ = 1.0;

  std::ifstream file(navgrid_path);
  if (!file.is_open()) {
    LOG(FATAL) << "Failed to open navgrid.json" << std::endl;
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
  auto next_pose_sub =
      inst.GetStructTopic<frc::Pose2d>("/pathing/nextPose").Publish();

  SplineResult result;

  while (true) {
    if (!enabled_sub.Get()) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);

      result.points.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    frc::Pose2d current_pose = current_sub.Get();

    frc::Pose2d target_pose = target_sub.Get();
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
    if (current_pose.Translation().Distance(t2d).value() < nodeSizeMeters) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      result.points.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    if (result.points.empty()) {
      result = createSpline(grid, start_pt, target_pt, nodeSizeMeters);
      if (!result.points.empty()) {
        result.points = result.points;
        if (verbose) {
          for (const auto& p : result.points) {
            LOG(INFO) << "spline pt: " << p.X().value() << " " << p.Y().value();
          }
        }
      }
    }

    if (result.points.empty()) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;

    } else {

      int closest_idx = 0;

      // TODO: this can be optimizeed by only searching from the previous closest index instead of the entire spline
      frc::Translation2d first2d(result.points[0].X(), result.points[0].Y());
      double best_dist = current_pose.Translation().Distance(first2d).value();
      for (int i = 1; i < (int)result.points.size(); ++i) {
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

      std::pair<double, double> derivative =
          EvaluateDerivative(result.params[closest_idx], result.controls,
                             result.knots, result.p, 1);

      auto [dx, dy] = derivative;

      if (verbose) {
        LOG(INFO) << "current " << current_pose.X().value() << " "
                  << current_pose.Y().value();
      }

      double mag = std::hypot(dx, dy);
      if (verbose) {
        LOG(INFO) << "dist " << mag;
      }
      if (mag < 1e-6) {
        vx_pub.Set(0.0);
        vy_pub.Set(0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      double vx = (dx / mag) * speed_;
      double vy = (dy / mag) * speed_;

      // NOTE: we need to test whether to divide vx and vy by dist to normalize speeds or not,
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
