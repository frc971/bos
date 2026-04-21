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

auto RunController(const std::string& navgrid_path =
                       "/root/bos/constants/navgrid.json") -> void {
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
  for (int y = 0; y < GRID_H - 1; ++y) {
    for (int x = 0; x < GRID_W - 1; ++x) {
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
      continue;
    }

    frc::Pose2d current_pose = current_sub.Get();

    frc::Pose2d target_pose = target_sub.Get();
    Point start_pt{.x = (int)(current_pose.X().value() / nodeSizeMeters),
                   .y = (int)(current_pose.Y().value() / nodeSizeMeters)};
    Point target_pt{.x = (int)(target_pose.X().value() / nodeSizeMeters),
                    .y = (int)(target_pose.Y().value() / nodeSizeMeters)};

    start_pt.x = std::clamp(start_pt.x, 0, GRID_W - 1);

    start_pt.y = std::clamp(start_pt.y, 0, GRID_H - 1);

    target_pt.x = std::clamp(target_pt.x, 0, GRID_W - 1);

    target_pt.y = std::clamp(target_pt.y, 0, GRID_H - 1);

    frc::Translation2d t3d(target_pose.X(), target_pose.Y());
    if (current_pose.Translation().Distance(t3d).value() < nodeSizeMeters) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      spline_points.clear();
      continue;
    }

    if (spline_points.empty()) {
      std::vector<frc::Pose2d> new_spline =
          createSpline(grid, start_pt, target_pt, nodeSizeMeters);
      if (!new_spline.empty()) {
        spline_points = new_spline;
        for (const auto& p : spline_points) {
          LOG(INFO) << "spline pt: " << p.X().value() << " " << p.Y().value();
        }
      }
    }

    if (spline_points.empty()) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      continue;

    } else {

      int closest_idx = 0;

      frc::Translation2d first3d(spline_points[0].X(), spline_points[0].Y());
      double best_dist = current_pose.Translation().Distance(first3d).value();
      for (int i = 1; i < (int)spline_points.size(); ++i) {
        frc::Translation2d p3d(spline_points[i].X(), spline_points[i].Y());
        double d = current_pose.Translation().Distance(p3d).value();
        if (d < best_dist) {
          best_dist = d;
          closest_idx = i;
        }
        LOG(INFO) << "d: " << d << " i: " << i;
      }

      LOG(INFO) << "Closeset idx: " << closest_idx
                << " Spline size: " << spline_points.size();

      int lookahead_idx = std::min(closest_idx + lookahead_offset_,
                                   (int)spline_points.size() - 1);

      frc::Pose2d lookahead = spline_points[lookahead_idx];

      LOG(INFO) << "current " << current_pose.X().value() << " "
                << current_pose.Y().value();

      double dx = lookahead.X().value() - current_pose.X().value();
      double dy = lookahead.Y().value() - current_pose.Y().value();
      LOG(INFO) << "dx " << dx << " vy " << dy;
      LOG(INFO) << "looakhead " << lookahead.X().value() << " "
                << lookahead.Y().value();

      next_pose_sub.Set(lookahead);

      double dist = std::sqrt(dx * dx + dy * dy);
      LOG(INFO) << "dist " << dist;
      if (dist < 1e-6) {
        vx_pub.Set(0.0);
        vy_pub.Set(0.0);
        continue;
      }

      double vx = (dx / dist) * speed_;
      double vy = (dy / dist) * speed_;

      vx_pub.Set(vx);
      vy_pub.Set(vy);
      LOG(INFO) << "set vx " << vx << " vy " << vy;
      inst.Flush();
    }
  }
}

}  // namespace pathing
