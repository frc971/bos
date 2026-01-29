#include "src/pathing/velocity_sender.h"
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

  nt::DoubleTopic latency_topic = table->GetDoubleTopic("Latency");
  latency_publisher_ = latency_topic.Publish();
}


void VelocitySender::Send(const double ax, const double bx, double latency) {
  if (mutex_.try_lock()) {
    std::vector<double> vel_array = {ax, bx};
    vel_publisher_.Set(vel_array);
    latency_publisher_.Set(latency);
  }

  mutex_.unlock();
}
}  // namespace pathing
