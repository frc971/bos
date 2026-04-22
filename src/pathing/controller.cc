#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>
#include "splines.h"
#include "src/localization/position_receiver.h"
#include "src/pathing/splines.h"
#include "src/utils/log.h"

namespace pathing {

auto RunController(
    const std::string& navgrid_path = "/root/bos/constants/navgrid.json",
    bool verbose = false) -> void {
  const int lookahead_offset_ = 50;
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

  std::vector<frc::Pose2d> spline_points;

  while (true) {
    if (!enabled_sub.Get()) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);

      spline_points.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    frc::Pose2d current_pose = current_sub.Get();

    frc::Pose2d target_pose = target_sub.Get();
    Point start_pt{.x = static_cast<uint>(
                       (int)(current_pose.X().value() / nodeSizeMeters)),
                   .y = static_cast<uint>(
                       (int)(current_pose.Y().value() / nodeSizeMeters))};
    Point target_pt{
        .x = static_cast<uint>((int)(target_pose.X().value() / nodeSizeMeters)),
        .y =
            static_cast<uint>((int)(target_pose.Y().value() / nodeSizeMeters))};

    start_pt.x = std::clamp(static_cast<int>(start_pt.x), 0, GRID_W - 1);

    start_pt.y = std::clamp(static_cast<int>(start_pt.y), 0, GRID_H - 1);

    target_pt.x = std::clamp(static_cast<int>(target_pt.x), 0, GRID_W - 1);

    target_pt.y = std::clamp(static_cast<int>(target_pt.y), 0, GRID_H - 1);

    frc::Translation2d t3d(target_pose.X(), target_pose.Y());
    if (current_pose.Translation().Distance(t3d).value() < nodeSizeMeters) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      spline_points.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    if (spline_points.empty()) {
      std::vector<frc::Pose2d> new_spline =
          createSpline(grid, start_pt, target_pt, nodeSizeMeters);
      if (!new_spline.empty()) {
        spline_points = new_spline;
        if (verbose) {
          for (const auto& p : spline_points) {
            LOG(INFO) << "spline pt: " << p.X().value() << " " << p.Y().value();
          }
        }
      }
    }

    if (spline_points.empty()) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;

    } else {

      int closest_idx = 0;

      // TODO: this can be optimizeed by only searching from the previous closest index instead of the entire spline
      frc::Translation2d first2d(spline_points[0].X(), spline_points[0].Y());
      double best_dist = current_pose.Translation().Distance(first2d).value();
      for (int i = 1; i < (int)spline_points.size(); ++i) {
        frc::Translation2d t2d(spline_points[i].X(), spline_points[i].Y());
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
                  << " Spline size: " << spline_points.size();
      }

      int lookahead_idx = std::min(closest_idx + lookahead_offset_,
                                   (int)spline_points.size() - 1);

      frc::Pose2d lookahead = spline_points[lookahead_idx];

      if (verbose) {
        LOG(INFO) << "current " << current_pose.X().value() << " "
                  << current_pose.Y().value();
      }

      double dx = lookahead.X().value() - current_pose.X().value();
      double dy = lookahead.Y().value() - current_pose.Y().value();
      if (verbose) {
        LOG(INFO) << "dx " << dx << " vy " << dy;
        LOG(INFO) << "looakhead " << lookahead.X().value() << " "
                  << lookahead.Y().value();
      }

      next_pose_sub.Set(lookahead);

      double dist = std::hypot(dx, dy);
      if (verbose) {
        LOG(INFO) << "dist " << dist;
      }
      if (dist < 1e-6) {
        vx_pub.Set(0.0);
        vy_pub.Set(0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      double vx = dx * speed_;
      double vy = dy * speed_;

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
