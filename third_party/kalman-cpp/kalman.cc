/**
d* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "kalman.h"

KalmanFilter::KalmanFilter(double dt, const Eigen::MatrixXd A,
                           const Eigen::MatrixXd C, const Eigen::MatrixXd Q,
                           const Eigen::MatrixXd R, const Eigen::MatrixXd P)
    : A(A),
      C(C),
      Q(Q),
      R(R),
      P0(P),
      m(C.rows()),
      n(A.rows()),
      dt(dt),
      initialized(false),
      I(n, n),
      x_hat(n),
      x_hat_new(n) {
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd y) {

  if (!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  P = A * P * A.transpose() + Q;
  K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
  x_hat_new += K * (y - C * x_hat_new);
  P = (I - K * C) * P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd y, double dt,
                          const Eigen::MatrixXd A) {

  this->A = A;
  this->dt = dt;
  update(y);
}

Eigen::MatrixXd KalmanFilter::predict_position(double dt) const { 
  if (!initialized) {
    throw std::runtime_error("Filter is not initialized!");
  }

  Eigen::MatrixXd new_A(2, 2); 
  new_A << 1, dt, 0, 1;

  // Time update and to project the state ahead
  Eigen::MatrixXd x_hat_new(2, 1); 
  x_hat_new = new_A * x_hat;

  return x_hat_new;

} 

Eigen::MatrixXd KalmanFilter::predict_variance(double dt) const {
  
  Eigen::MatrixXd new_A(2, 2);
  new_A << 1, dt, 0, 1;
    
  // Update covariance
  Eigen::MatrixXd new_P(2, 1);
  new_P = new_A * P * new_A.transpose() + Q;

  return new_P;
}
