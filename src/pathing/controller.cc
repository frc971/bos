#include "controller.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/Topic.h>
#include <cfloat>
#include <chrono>
#include <climits>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>
#include "splines.h"
#include "src/localization/position_receiver.h"
#include "src/pathing/splines.h"
#include "src/utils/log.h"
#include "src/utils/nt_utils.h"

namespace pathing {

auto RunController(const std::string& navgrid_path =
                       "/root/bos/constants/navgrid.json") -> void {

  std::ifstream file(navgrid_path);
  if (!file.is_open()) {
    LOG(FATAL) << "Failed to open navgrid.json" << std::endl;
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
  auto current_sub =
      inst.GetStructTopic<frc::Pose3d>("/Orin/PoseEstimate/Left/Pose3d")
          .Subscribe({});
  auto target_sub =
      inst.GetStructTopic<frc::Pose2d>("/pathing/target").Subscribe({});

  auto enabled_sub = inst.GetBooleanTopic("/pathing/enabled").Subscribe(false);
  auto max_speed_sub = inst.GetDoubleTopic("/pathing/max_speed").Subscribe(1.0);

  auto vx_pub = inst.GetDoubleTopic("/pathing/vx").Publish();
  auto vy_pub = inst.GetDoubleTopic("/pathing/vy").Publish();
  // vx_pub.Set(1.0);
  // vy_pub.Set(1.0);

  std::vector<frc::Pose2d> spline_points;
  int spline_idx = 0;
  pathing::Point prev_target_pt{.x = -1, .y = -1};
  pathing::Point prev_start_pt{.x = -1, .y = -1};

  while (true) {
    if (!enabled_sub.Get()) {
      LOG(INFO) << "pathing disabled, enabled=" << enabled_sub.Get();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }
    frc::Pose3d current_pose = current_sub.Get();
    frc::Pose2d target_pose = target_sub.Get();

    LOG(INFO) << current_pose.X().value() << " " << current_pose.Y().value();
    LOG(INFO) << "spline idx: " << spline_idx << " with len "
              << spline_points.size();

    pathing::Point start_pt{
        .x = (int)(current_pose.X().value() / nodeSizeMeters),
        .y = (int)(current_pose.Y().value() / nodeSizeMeters)};
    pathing::Point target_pt{
        .x = (int)(target_pose.X().value() / nodeSizeMeters),
        .y = (int)(target_pose.Y().value() / nodeSizeMeters)};

    frc::Translation3d t3d(target_pose.X(), target_pose.Y(), 0_m);
    if (current_pose.Translation().Distance(t3d).value() < nodeSizeMeters) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    if (target_pt.x != prev_target_pt.x || target_pt.y != prev_target_pt.y ||
        start_pt.x != prev_start_pt.x || start_pt.y != prev_start_pt.y ||
        spline_points.empty()) {
      spline_points =
          pathing::createSpline(grid, start_pt, target_pt, nodeSizeMeters);
      spline_idx = 0;
      prev_target_pt = target_pt;
      prev_start_pt = start_pt;
    }

    if (!spline_points.empty() && spline_idx == 0) {
      LOG(INFO) << "spline[0]=(" << spline_points[0].X().value() << ","
                << spline_points[0].Y().value() << ") spline[last]=("
                << spline_points.back().X().value() << ","
                << spline_points.back().Y().value() << ") start_grid=("
                << start_pt.x << "," << start_pt.y << ") target_grid=("
                << target_pt.x << "," << target_pt.y << ")";
    }

    if (spline_points.empty()) {
      LOG(INFO) << "spline empty";
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    while (spline_idx + 1 < (int)spline_points.size()) {
      frc::Translation3d curr3d(spline_points[spline_idx].X(),
                                spline_points[spline_idx].Y(), 0_m);
      frc::Translation3d next3d(spline_points[spline_idx + 1].X(),
                                spline_points[spline_idx + 1].Y(), 0_m);
      double ds_curr = current_pose.Translation().Distance(curr3d).value();
      double ds_next = current_pose.Translation().Distance(next3d).value();
      if (ds_next < ds_curr) {
        spline_idx++;
      } else {
        break;
      }
    }

    int lookahead_idx =
        std::min(spline_idx + 10, (int)spline_points.size() - 1);
    if (lookahead_idx <= 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    frc::Pose2d lookahead = spline_points[lookahead_idx];

    double dx = lookahead.X().value() - current_pose.X().value();
    double dy = lookahead.Y().value() - current_pose.Y().value();
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 1e-6) {
      LOG(INFO) << "bad distance: " << dist;
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    double vx = (dx / dist);
    double vy = (dy / dist);

    // double vx = -0.5;
    // double vy = -0.5;

    LOG(INFO) << vx << vy << "controller";

    vx_pub.Set(vx);
    vy_pub.Set(vy);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

}  // namespace pathing
