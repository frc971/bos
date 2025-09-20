#ifndef POSITION_SENDER_H
#define POSITION_SENDER_H

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "pose_estimator.h"

class PositionSender {
 public:
  PositionSender(std::string name, std::vector<int> tag_ids);
  void Send(std::vector<Localization::position_t> position_estimate);

 private:
  std::vector<int> tag_ids_;
  nt::NetworkTableInstance instance_;

  std::vector<nt::DoublePublisher> translation_x_publisher_;
  std::vector<nt::DoublePublisher> translation_y_publisher_;
  std::vector<nt::DoublePublisher> translation_z_publisher_;

  std::vector<nt::DoublePublisher> rotation_x_publisher_;
  std::vector<nt::DoublePublisher> rotation_y_publisher_;
  std::vector<nt::DoublePublisher> rotation_z_publisher_;
  
  std::vector<nt::DoublePublisher> dist_camera_tag_publisher_;

  std::vector<nt::BooleanPublisher> status_;  // Was a tag detected?
};

#endif  // POSITION_SENDER_H
