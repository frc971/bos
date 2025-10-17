#include "pch.h"

#include "simple_kalman.h"
#include <wpilibc/frc/Timer.h>

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


std::pair<Eigen::MatrixXd, Eigen::MatrixXd> SimpleKalman::Predict(double time) {
  double dt = time - time_;
  Eigen::MatrixXd pred_pos = kalman_filter_.predict_position(dt);
  Eigen:: MatrixXd pred_variance = kalman_filter_.predict_variance(dt);

  return {pred_pos, pred_variance};

} 


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

}  // namespace localization