#include "position_sender.h"
#include <networktables/NetworkTableInstance.h>
#include <string>
#include "src/localization/position.h"

namespace localization {

constexpr double RadianToDegree(double radian) {
  return radian * (180 / M_PI);
}

PositionSender::PositionSender(bool verbose)
    : instance_(nt::NetworkTableInstance::GetDefault()), verbose_(verbose) {
  std::shared_ptr<nt::NetworkTable> table =
      instance_.GetTable("orin/pose_estimate");

  nt::DoubleTopic translation_x_topic = table->GetDoubleTopic("translation_x");
  nt::DoubleTopic translation_y_topic = table->GetDoubleTopic("translation_y");

  nt::DoubleTopic rotation_topic = table->GetDoubleTopic("rotation");

  nt::DoubleTopic translation_x_varience_topic =
      table->GetDoubleTopic("translation_x_varience");
  nt::DoubleTopic translation_y_varience_topic =
      table->GetDoubleTopic("translation_y_varience");

  nt::DoubleTopic rotation_varience_topic =
      table->GetDoubleTopic("rotation_varience");

  translation_x_publisher_ = translation_x_topic.Publish();
  translation_y_publisher_ = translation_y_topic.Publish();

  rotation_publisher_ = rotation_topic.Publish();

  translation_x_varience_publisher_ = translation_x_varience_topic.Publish();
  translation_y_varience_publisher_ = translation_y_varience_topic.Publish();

  rotation_varience_publisher_ = rotation_varience_topic.Publish();
}

void PositionSender::Send(pose2d_t position_estimates, pose2d_t varience) {
  if (mutex_.try_lock()) {
    translation_x_publisher_.Set(position_estimates.x);
    translation_y_publisher_.Set(position_estimates.y);
    rotation_publisher_.Set(position_estimates.rotation);

    translation_x_varience_publisher_.Set(varience.x);
    translation_y_varience_publisher_.Set(varience.y);
    rotation_varience_publisher_.Set(varience.rotation);
    mutex_.unlock();
  }
  if (verbose_) {
    std::cout << "Position Sender: "
              << "\n";
    std::cout << "Translation: "
              << "\n";
    std::cout << position_estimates.x << "\n";
    std::cout << position_estimates.y << "\n";
    std::cout << RadianToDegree(position_estimates.rotation) << "\n";

    std::cout << "Varience: "
              << "\n";
    std::cout << varience.x << "\n";
    std::cout << varience.y << "\n";
    std::cout << varience.rotation << "\n";
  }
}
}  // namespace localization
