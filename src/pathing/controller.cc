#include <frc/geometry/Translation3d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>
#include <chrono>
#include "path_follower.h"
#include "src/localization/position_receiver.h"
#include "src/utils/log.h"
#include "src/utils/pch.h"

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

  PathFollower follower(grid, nodeSizeMeters, 10.0, 0.4, 1000);
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  auto current_sub = localization::PositionReceiver();
  auto target_sub =
      inst.GetStructTopic<frc::Pose2d>("/pathing/target").Subscribe({});

  auto enabled_sub = inst.GetBooleanTopic("/pathing/enabled").Subscribe(false);

  auto vx_pub = inst.GetDoubleTopic("/pathing/vx").Publish();
  auto vy_pub = inst.GetDoubleTopic("/pathing/vy").Publish();
  auto isDone_pub = inst.GetBooleanTopic("/pathing/isDone").Publish();

  while (!stop_token.stop_requested()) {
    if (!enabled_sub.Get()) {
      vx_pub.Set(0.0);
      vy_pub.Set(0.0);
      follower.reset();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    frc::Pose2d target_pose = target_sub.Get();
    frc::Pose2d current_pose = current_sub.Get();

    FollowerOutput out = follower.update(current_pose, target_pose);
    vx_pub.Set(out.vx);
    vy_pub.Set(out.vy);
    isDone_pub.Set(out.done);
    inst.Flush();
  }
}

}  // namespace pathing
