#include "src/pathing/velocity_sender.h"
#include <frc/Timer.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>
#include <units/angle.h>
#include "frc/DataLogManager.h"

namespace pathing {

constexpr auto RadianToDegree(double radian) -> double {
  return radian * (180 / M_PI);
}

VelocitySender::VelocitySender()
    : instance_(nt::NetworkTableInstance::GetDefault()) {
  std::shared_ptr<nt::NetworkTable> table =
      instance_.GetTable("Orin/OTFVelocity/");

  nt::DoubleArrayTopic vel_topic = table->GetDoubleArrayTopic("Velocity");
  vel_publisher_ = vel_topic.Publish();
}

void VelocitySender::Send(const double ax, const double ay) {
  if (mutex_.try_lock()) {
    double timestamp = frc::Timer::GetFPGATimestamp().to<double>() +
                       instance_.GetServerTimeOffset().value_or(0) / 1000000.0;
    std::vector<double> vel_array = {ax, ay, timestamp};
    vel_publisher_.Set(vel_array);
    mutex_.unlock();
  }
}
}  // namespace pathing