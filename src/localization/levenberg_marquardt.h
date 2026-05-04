#pragma once

#include <Eigen/Dense>
#include <functional>

// Performs a single damped Gauss-Newton step using user-provided callbacks to
// compute the residual vector and Jacobian at arbitrary parameter values. The
// trial step is accepted (and lambda halved) when the recomputed residual norm
// does not increase; otherwise lambda is doubled and the parameters remain
// unchanged. Returns true when an accepted step falls below the provided
// tolerance.
auto levenberg_marquardt(
    Eigen::VectorXd& params,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
        compute_residual,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>&
        compute_jacobian,
    double& lambda, double tolerance = 1e-8) -> bool;
