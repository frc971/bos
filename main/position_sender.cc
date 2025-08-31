#include "position_sender.h"

PositionSender::PositionSender() {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("orin/pose_estimate");

  nt::DoubleTopic translation_x_topic = table->GetDoubleTopic("translation_x");
  nt::DoubleTopic translation_y_topic = table->GetDoubleTopic("translation_y");
  nt::DoubleTopic translation_z_topic = table->GetDoubleTopic("translation_z");

  nt::DoubleTopic rotation_x_topic = table->GetDoubleTopic("position_x");
  nt::DoubleTopic rotation_y_topic = table->GetDoubleTopic("position_y");
  nt::DoubleTopic rotation_z_topic = table->GetDoubleTopic("position_z");

  translation_x_publisher_ = translation_x_topic.Publish();
  translation_y_publisher_ = translation_y_topic.Publish();
  translation_z_publisher_ = translation_z_topic.Publish();

  rotation_x_publisher_ = rotation_x_topic.Publish();
  rotation_y_publisher_ = rotation_y_topic.Publish();
  rotation_z_publisher_ = rotation_z_topic.Publish();
}

void PositionSender::Send(
    std::vector<PoseEstimator::position_estimate_t> position_estimate) {
  // TODO
  // translation_x_publisher_.Set(position_estimate.translation.x);
  // translation_y_publisher_.Set(position_estimate.translation.y);
  // translation_z_publisher_.Set(position_estimate.translation.z);
  //
  // rotation_x_publisher_.Set(position_estimate.translation.x);
  // rotation_y_publisher_.Set(position_estimate.translation.y);
  // rotation_z_publisher_.Set(position_estimate.translation.z);
  // flush?
}
