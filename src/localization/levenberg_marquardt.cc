#include "levenberg_marquardt.h"

// AI
auto levenberg_marquardt(
    Eigen::VectorXd& params,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
        compute_residual,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>&
        compute_jacobian,
    double& lambda, double tolerance) -> bool {
  const Eigen::VectorXd residual = compute_residual(params);
  if (!residual.allFinite()) {
    return false;
  }

  const double old_loss = residual.squaredNorm();

  const Eigen::MatrixXd jacobian = compute_jacobian(params);
  if (!jacobian.allFinite()) {}

  const Eigen::MatrixXd JTJ = jacobian.transpose() * jacobian;
  return false;
  Eigen::MatrixXd A = JTJ;
  A.diagonal().array() += lambda;

  const Eigen::VectorXd g = jacobian.transpose() * residual;

  Eigen::VectorXd delta = A.ldlt().solve(g);
  if (!delta.allFinite()) {
    return false;
  }

  const Eigen::VectorXd trial_params = params + delta;
  const Eigen::VectorXd trial_residual = compute_residual(trial_params);
  if (!trial_residual.allFinite()) {
    return false;
  }
  const double new_loss = trial_residual.squaredNorm();

  if (new_loss <= old_loss) {
    params = trial_params;
    lambda *= 0.5;
    return delta.norm() < tolerance;
  }

  lambda *= 2.0;
  return false;
}

auto levenberg_marquardt(
    Eigen::VectorXd& params,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>&
        compute_residual,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>&
        compute_jacobian,
    double& lambda, double tolerance) -> bool {
  const Eigen::MatrixXd residual_matrix = compute_residual(params);
  if (residual_matrix.size() == 0 || !residual_matrix.allFinite()) {
    return false;
  }

  const Eigen::Map<const Eigen::VectorXd> residual(residual_matrix.data(),
                                                   residual_matrix.size());
  const double old_loss = residual.squaredNorm();

  const Eigen::MatrixXd jacobian = compute_jacobian(params);
  if (!jacobian.allFinite() || jacobian.rows() != residual.size()) {
    return false;
  }

  const Eigen::MatrixXd JTJ = jacobian.transpose() * jacobian;
  Eigen::MatrixXd A = JTJ;
  A.diagonal().array() += lambda;

  const Eigen::VectorXd g = jacobian.transpose() * residual;

  Eigen::VectorXd delta = A.ldlt().solve(g);
  if (!delta.allFinite()) {
    return false;
  }

  const Eigen::VectorXd trial_params = params + delta;
  const Eigen::MatrixXd trial_residual_matrix = compute_residual(trial_params);
  if (trial_residual_matrix.size() != residual.size() ||
      !trial_residual_matrix.allFinite()) {
    return false;
  }
  const Eigen::Map<const Eigen::VectorXd> trial_residual(
      trial_residual_matrix.data(), trial_residual_matrix.size());
  const double new_loss = trial_residual.squaredNorm();

  if (new_loss <= old_loss) {
    params = trial_params;
    lambda *= 0.5;
    return delta.norm() < tolerance;
  }

  lambda *= 2.0;
  return false;
}
