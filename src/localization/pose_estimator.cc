#include "include/pch.h"

#include "pose_estimator.h"
#include <frc/geometry/Pose2d.h>
#include <wpilibc/frc/Timer.h>

namespace localization {

constexpr double DistanceToVarience(double distance) {
  return distance;
}

constexpr double ClampAngle(double angle) {
  return std::remainder(angle, 2.0 * std::numbers::pi_v<double>);
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
  x_filter_.Update(x, time, varience);
  y_filter_.Update(y, time, varience);
  rotation_filter_.Update(ClampAngle(rotation), time, varience);
  rotation_filter_.set_position(ClampAngle(rotation_filter_.position()));
}
pose2d_t PoseEstimator::GetPose() {
  double current_time = frc::Timer::GetFPGATimestamp().to<double>();
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> x_predicted_state_and_variance =
      x_filter_.Predict(current_time);
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> y_predicted_state_and_variance =
      y_filter_.Predict(current_time);
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> rot_predicted_state_and_variance =
      rotation_filter_.Predict(current_time);
  pose2d_t position2d{.x = x_predicted_state_and_variance.first(0),
                      .y = y_predicted_state_and_variance.first(0),
                      .rotation = rot_predicted_state_and_variance.first(0)};

  return position2d;
}

pose2d_t PoseEstimator::GetPoseVarience() {
  double current_time = frc::Timer::GetFPGATimestamp().to<double>();
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> x_predicted_state_and_variance =
      x_filter_.Predict(current_time);
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> y_predicted_state_and_variance =
      y_filter_.Predict(current_time);
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> rot_predicted_state_and_variance =
      rotation_filter_.Predict(current_time);
  pose2d_t position2d{.x = x_predicted_state_and_variance.second(0),
                      .y = y_predicted_state_and_variance.second(0),
                      .rotation = rot_predicted_state_and_variance.second(0)};
  return position2d;
}

}  // namespace localization  