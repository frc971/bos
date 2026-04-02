#include "controller.h"
#include <frc/geometry/Pose2d.h>
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
#include "src/pathing/splines.h"
#include "src/utils/log.h"
#include "src/utils/nt_utils.h"

namespace pathing {

auto RunController() -> void {

  std::ifstream file("/root/bos/constants/navgrid.json");
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
      grid[y][x].obstacle = !data["grid"][y][x];
    }
  }

  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  auto pose_sub = inst.GetStructTopic<frc::Pose2d>("/localization/Left/pose")
                      .Subscribe(frc::Pose2d{});

  auto target_sub = inst.GetStructTopic<frc::Pose2d>("/pathing/target")
                        .Subscribe(frc::Pose2d{});

  auto enabled_sub = inst.GetBooleanTopic("/pathing/enabled").Subscribe(false);
  auto max_speed_sub = inst.GetDoubleTopic("/pathing/max_speed").Subscribe(1.0);

  auto vx_pub = inst.GetDoubleTopic("/pathing/vx").Publish();
  auto vy_pub = inst.GetDoubleTopic("/pathing/vy").Publish();

  std::vector<frc::Pose2d> spline_points = pathing::createSpline(
      grid,
      pathing::Point{.x = (int)(pose_sub.Get().X().value() / nodeSizeMeters),
                     .y = (int)(pose_sub.Get().Y().value() / nodeSizeMeters)},
      pathing::Point{.x = (int)(target_sub.Get().X().value() / nodeSizeMeters),
                     .y = (int)(target_sub.Get().Y().value() / nodeSizeMeters)},
      nodeSizeMeters);
  int spline_idx = 0;

  while (true) {
    if (!enabled_sub.Get()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    frc::Pose2d current_pose = pose_sub.Get();

    double best_dist = DBL_MAX;
    for (int i = spline_idx; i < spline_points.size(); i++) {
      double ds = current_pose.Translation()
                      .Distance(spline_points[i].Translation())
                      .value();
      if (ds < best_dist) {
        best_dist = ds;
        spline_idx = i;
      }
    }

    int lookahead_idx = spline_idx + 10;
    frc::Pose2d lookahead = spline_points[lookahead_idx];

    double dx = lookahead.X().value() - current_pose.X().value();
    double dy = lookahead.Y().value() - current_pose.Y().value();
    double dist = std::sqrt(dx * dx + dy * dy);
    double vx = (dx / dist);
    double vy = (dy / dist);

    vx_pub.Set(vx);
    vy_pub.Set(vy);
  }
}

}  // namespace pathing