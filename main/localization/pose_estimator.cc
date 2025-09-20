#include "pose_estimator.h"
#include <frc/geometry/Pose2d.h>

namespace Localization {

PoseEstimator::PoseEstimator(SimpleKalmanConfig x_filter_config,
                             SimpleKalmanConfig y_filter_config,
                             SimpleKalmanConfig rotation_filter_config)
    : x_filter_(x_filter_config),
      y_filter_(y_filter_config),
      rotation_filter_(rotation_filter_config) {}

void PoseEstimator::Update(double x, double y, double rotation) {
  x_filter_.Update(x);
  y_filter_.Update(y);
  rotation_filter_.Update(rotation);
}

void PoseEstimator::Update(std::vector<tag_detection_t> position) {
  for (size_t i = 0; i < position.size(); i++) {
    Update(position[i].translation.x, position[i].translation.y,
           position[i].rotation.z);
  }
}

pose2d_t PoseEstimator::GetPose() {
  pose2d_t position2d{.x = x_filter_.position(),
                      .y = y_filter_.position(),
                      .rotation = rotation_filter_.position()};
  return position2d;
}

}  // namespace Localization
