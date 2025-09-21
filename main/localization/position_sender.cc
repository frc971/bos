#include "position_sender.h"
#include <networktables/NetworkTableInstance.h>
#include <string>
#include "main/localization/position.h"

PositionSender::PositionSender()
    : instance_(nt::NetworkTableInstance::GetDefault()) {
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

void PositionSender::Send(Localization::pose2d_t position_estimates,
                          Localization::pose2d_t varience) {
  translation_x_publisher_.Set(position_estimates.x);
  translation_y_publisher_.Set(position_estimates.y);
  rotation_publisher_.Set(position_estimates.rotation);

  translation_x_varience_publisher_.Set(varience.x);
  translation_y_varience_publisher_.Set(varience.y);
  rotation_varience_publisher_.Set(varience.rotation);
}
