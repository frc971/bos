#include "simple_kalman.h"
#include <wpilibc/frc/Timer.h>
#include <iostream>

namespace localization {

SimpleKalman::SimpleKalman(double position, double velocity, double time,
                           double measurment_noise, double process_noise) {
  constexpr double placeholder_dt = 1;
  Eigen::MatrixXd A(2, 2);
  Eigen::MatrixXd C(1, 2);
  Eigen::MatrixXd Q(2, 2);
  Eigen::MatrixXd R(1, 1);
  Eigen::MatrixXd P(2, 2);

  A << 1, placeholder_dt, 0, 1;
  C << 1, 0;
  Q << process_noise, 0, 0, process_noise;
  R << measurment_noise;
  P << 1, 0, 0, 1;
  kalman_filter_ = KalmanFilter(placeholder_dt, A, C, Q, R, P);
  Eigen::VectorXd initial_pose(2);
  initial_pose << position, velocity;
  kalman_filter_.init(time, initial_pose);
  time_ = frc::Timer::GetFPGATimestamp().to<double>();
}

SimpleKalman::SimpleKalman(SimpleKalmanConfig config)
    : SimpleKalman(config.position, config.velocity, config.time,
                   config.measurment_noise, config.process_noise) {}

void SimpleKalman::Update(double position_update, double time,
                          double varience) {
  double dt = time - time_;
  time_ = time;

  Eigen::MatrixXd A(2, 2);
  A << 1, dt, 0, 1;

  Eigen::VectorXd position_update_(1);
  position_update_ << position_update;
  set_measurment_varience(varience);
  kalman_filter_.update(position_update_, dt, A);
}

std::pair<double, double> SimpleKalman::prediction(double dt) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  A(0, 1) = dt;
  Eigen::VectorXd x_pred = A * kalman_filter_.state();
  Eigen::MatrixXd P_pred =
      A * kalman_filter_.P * A.transpose() + kalman_filter_.Q;
  double predicted_position = x_pred(0);
  double predicted_velocity = x_pred(1);
  double predicted_position_variance = P_pred(0, 0);
  return {predicted_position, predicted_position_variance};
}

}  // namespace localization
