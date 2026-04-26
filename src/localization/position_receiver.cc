#include "src/localization/position_receiver.h"

namespace localization {

PositionReceiver::PositionReceiver() {
  auto instance = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = instance.GetTable("DriveState");
  nt::StructTopic<frc::Pose2d> pose_topic =
      table->GetStructTopic<frc::Pose2d>("Pose");
  pose_subscriber_ = pose_topic.Subscribe({});
}

auto PositionReceiver::Get() -> frc::Pose2d {
  return pose_subscriber_.Get();
}

}  // namespace localization
