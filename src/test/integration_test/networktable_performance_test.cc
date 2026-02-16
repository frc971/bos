#include <frc/geometry/Pose2d.h>
#include <networktables/StructTopic.h>
#include "src/utils/log.h"
#include "src/utils/nt_utils.h"
#include "src/utils/pch.h"

auto main() -> int {
  utils::StartNetworktables();

  auto instance = nt::NetworkTableInstance::GetDefault();

  std::shared_ptr<nt::NetworkTable> table = instance.GetTable("DriveState");

  auto pose_subscriber =
      table->GetStructTopic<frc::Pose2d>("Pose").Subscribe(frc::Pose2d());
  for (;;) {
    auto pose = pose_subscriber.GetAtomic();
    LOG(INFO) << pose.value;
    LOG(INFO) << pose.time - frc::Timer::GetFPGATimestamp().to<double>();
  }
}
