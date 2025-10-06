#include "pose_estimator.h"
#include <frc/geometry/Pose2d.h>
#include <iostream>

namespace localization {

constexpr double DistanceToVarience(double distance) {
  return distance;
}

constexpr double ClampAngle(double angle) {
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0) {
    angle += 2.0 * M_PI;
  }
  angle -= M_PI;

  if (angle > M_PI_2) {
    angle -= M_PI;
  }
  if (angle < -M_PI_2) {
    angle += M_PI;
  }
  return angle;
}

PoseEstimator::PoseEstimator(SimpleKalmanConfig x_filter_config,
                             SimpleKalmanConfig y_filter_config,
                             SimpleKalmanConfig rotation_filter_config)
    : x_filter_(x_filter_config),
      y_filter_(y_filter_config),
      rotation_filter_(rotation_filter_config) {}

void PoseEstimator::Update(std::vector<tag_detection_t> position) {
  update_mutex_.lock();
  for (size_t i = 0; i < position.size(); i++) {
    // std::cout << "trainslation x and y"
    //           << "\n";
    // std::cout << position[i].translation.x << "\n";
    // std::cout << position[i].translation.y << "\n";
    // std::cout << "\n";
    UpdateKalmanFilter(position[i].translation.x, position[i].translation.y,
                       position[i].rotation.z,
                       DistanceToVarience(position[i].distance),
                       position[i].timestamp);
  }
  update_mutex_.unlock();
}

void PoseEstimator::Update(double x, double y, double rotation, double varience,
                           double time) {
  update_mutex_.lock();
  UpdateKalmanFilter(x, y, rotation, varience, time);
  update_mutex_.unlock();
}

void PoseEstimator::UpdateKalmanFilter(double x, double y, double rotation,
                                       double varience, double time) {
  // time = 0.1;
  // std::cout << "x: " << x << "\n";
  // std::cout << "time: " << time << "\n";
  // std::cout << "varience: " << varience << "\n";
  x_filter_.Update(x, time, varience);
  // std::cout << "x filter position: " << x_filter_.position() << "\n";
  y_filter_.Update(y, time, varience);
  rotation_filter_.Update(ClampAngle(rotation), time, varience);
  // rotation_filter_.position() = ClampAngle(rotation_filter_.position());
}
pose2d_t PoseEstimator::GetPose() {
  pose2d_t position2d{.x = x_filter_.position(),
                      .y = y_filter_.position(),
                      .rotation = rotation_filter_.position()};
  return position2d;
}

pose2d_t PoseEstimator::GetPoseVarience() {

  pose2d_t varience{.x = x_filter_.position_varience(),
                    .y = y_filter_.position_varience(),
                    .rotation = rotation_filter_.position_varience()};
  return varience;
}

}  // namespace localization
