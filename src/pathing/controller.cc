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

  const auto& navgrid = pathing::GetGrid("/root/bos/constants/navgrid.json");
  const auto& grid = navgrid.grid;
  const auto& nodeSizeMeters = navgrid.nodeSizeMeters;

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
