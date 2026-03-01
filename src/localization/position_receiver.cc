#include "src/localization/position_receiver.h"

namespace localization {

PositionReceiver::PositionReceiver() {
  auto instance = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = instance.GetTable("");
  nt::StructTopic<frc::Pose3d> pose3d_topic =
      table->GetStructTopic<frc::Pose3d>("Pose3d");
  pose3d_subscriber_ = pose3d_topic.Subscribe({});
}

auto PositionReceiver::Get() -> frc::Pose3d {
  return pose3d_subscriber_.Get();
}

}  // namespace localization
