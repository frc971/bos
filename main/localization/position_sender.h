#ifndef BOS_MAIN_LOCALIZATION_POSITION_SENDER_H_
#define BOS_MAIN_LOCALIZATION_POSITION_SENDER_H_

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "pose_estimator.h"

class PositionSender {
 public:
  PositionSender(std::vector<int> tag_ids);
  void Send(std::vector<PoseEstimator::position_estimate_t> position_estimate,
            std::vector<);

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

  std::vector<nt::BooleanPublisher> status_;
};

#endif  // POSITION_SENDER_H
