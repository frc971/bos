#include "position_sender.h"
#include <networktables/NetworkTableInstance.h>
#include <string>

PositionSender::PositionSender(std::string name, std::vector<int> tag_ids)
    : tag_ids_(tag_ids), instance_(nt::NetworkTableInstance::GetDefault()) {
  for (size_t i = 0; i < tag_ids_.size(); i++) {
    std::shared_ptr<nt::NetworkTable> table = instance_.GetTable(
        "orin/pose_estimate/" + name + "/" + std::to_string(tag_ids_[i]));

    nt::DoubleTopic translation_x_topic =
        table->GetDoubleTopic("translation_x");
    nt::DoubleTopic translation_y_topic =
        table->GetDoubleTopic("translation_y");
    nt::DoubleTopic translation_z_topic =
        table->GetDoubleTopic("translation_z");

    nt::DoubleTopic rotation_x_topic = table->GetDoubleTopic("rotation_x");
    nt::DoubleTopic rotation_y_topic = table->GetDoubleTopic("rotation_y");
    nt::DoubleTopic rotation_z_topic = table->GetDoubleTopic("rotation_z");

    nt::BooleanTopic status_topic = table->GetBooleanTopic("status");

    translation_x_publisher_.push_back(translation_x_topic.Publish());
    translation_y_publisher_.push_back(translation_y_topic.Publish());
    translation_z_publisher_.push_back(translation_z_topic.Publish());

    rotation_x_publisher_.push_back(rotation_x_topic.Publish());
    rotation_y_publisher_.push_back(rotation_y_topic.Publish());
    rotation_z_publisher_.push_back(rotation_z_topic.Publish());

    status_.push_back(status_topic.Publish());
  }
}

void PositionSender::Send(
    std::vector<Localization::tag_detection_t> position_estimates) {
  for (size_t i = 0; i < tag_ids_.size(); i++) {
    for (size_t j = 0; j < position_estimates.size(); j++) {
      status_[i].Set(false);
      if (tag_ids_[i] == position_estimates[j].tag_id) {
        translation_x_publisher_[i].Set(position_estimates[j].translation.y);
        translation_y_publisher_[i].Set(position_estimates[j].translation.z);
        translation_z_publisher_[i].Set(position_estimates[j].translation.x);

        rotation_x_publisher_[i].Set(position_estimates[j].rotation.x);
        rotation_y_publisher_[i].Set(position_estimates[j].rotation.y);
        rotation_z_publisher_[i].Set(position_estimates[j].rotation.z);
        status_[i].Set(true);
      }
    }
  }
  instance_.Flush();
}
