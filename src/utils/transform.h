#pragma once
#include <absl/flags/flag.h>
#include <frc/geometry/Pose3d.h>
#include <opencv2/calib3d.hpp>
#include "src/utils/pch.h"

// Helper functions for converting between wpilib, opencv and eigen
namespace utils {
enum Basis { WPI_TO_CV, CV_TO_WPI };

// clang-format off
const cv::Mat cv_to_wpilib = (cv::Mat_<double>(4, 4) << 
  0, 0, 1, 0,
  -1, 0, 0, 0,
  0, -1, 0, 0,
  0, 0, 0, 1);
// clang-format on

const std::map<Basis, cv::Mat> cv_bases = {{WPI_TO_CV, cv_to_wpilib.t()},
                                           {CV_TO_WPI, cv_to_wpilib}};

auto MakeTransform(const cv::Mat& rvec, const cv::Mat& tvec) -> cv::Mat;

template <typename Derived>
auto EigenToCvMat(const Eigen::MatrixBase<Derived>& mat) -> cv::Mat {
  cv::Mat cvMat(mat.rows(), mat.cols(), CV_64F);
  Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      cvMat.ptr<double>(), mat.rows(), mat.cols()) = mat;
  return cvMat;
}

auto CvMatToEigen(const cv::Mat& mat) -> Eigen::Matrix4d;

auto inline ChangeBasis(cv::Mat& mat, Basis basis) -> void {
  const cv::Mat& basis_mat = cv_bases.at(basis);
  mat = basis_mat * mat;
  if (mat.cols == mat.rows) {
    mat = mat * basis_mat.t();
  }
}

template <typename Derived>
auto inline ChangeBasis(Eigen::MatrixBase<Derived>& mat, Basis basis) -> void {
  const Eigen::MatrixXd& basis_mat = CvMatToEigen(cv_bases.at(basis));
  mat = basis_mat * mat;
  if (mat.ColsAtCompileTime == mat.RowsAtCompileTime) {
    mat = mat * basis_mat.transpose();
  }
}

auto ConvertOpencvCoordinateToWpilib(cv::Mat& vec) -> void;

auto ConvertOpencvTransformationMatrixToWpilibPose(const cv::Mat& matrix)
    -> frc::Pose3d;

auto Pose3dToCvMat(frc::Pose3d) -> cv::Mat;

auto inline Homogenize(const Eigen::Vector2d point) -> Eigen::Vector3d {
  Eigen::Vector3d homogenized_point;
  homogenized_point << point, 1;
  return homogenized_point;
}

auto inline Homogenize(const Eigen::Vector3d point) -> Eigen::Vector4d {
  Eigen::Vector4d homogenized_point;
  homogenized_point << point, 1;
  return homogenized_point;
}

struct TransformValues {
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
  bool yaw_only = true;

  friend void operator+=(TransformValues& left, const TransformValues& right) {
    left.x += right.x;
    left.y += right.y;
    left.z += right.z;
    left.ry += right.ry;
    if (!left.yaw_only) {
      left.rx += right.rx;
      left.rz += right.rz;
    }
  }

  friend void operator-=(TransformValues& left, const TransformValues& right) {
    left.x -= right.x;
    left.y -= right.y;
    left.z -= right.z;
    left.ry -= right.ry;
    if (!left.yaw_only) {
      left.rx -= right.rx;
      left.rz -= right.rz;
    }
  }

  friend void operator/=(TransformValues& left, const double right) {
    left.x /= right;
    left.y /= right;
    left.z /= right;
    left.ry /= right;
    if (!left.yaw_only) {
      left.rx /= right;
      left.rz /= right;
    }
  }

  friend auto operator*(TransformValues& left, const double right)
      -> TransformValues {
    return {
        .x = left.x * right,
        .y = left.y * right,
        .z = left.z * right,
        .rx = left.rx * right,
        .ry = left.ry * right,
        .rz = left.rz * right,
    };
  }

  friend auto operator<<(std::ostream& os, const TransformValues& p)
      -> std::ostream& {
    os << "x: " << p.x << "\ty: " << p.y << "\tz: " << p.z << "\trx: " << p.rx
       << "\try: " << p.ry << "\trz: " << p.rz << std::endl;
    return os;
  }
};

ABSL_FLAG(double, first_moment_decay, 0.9, "");
ABSL_FLAG(double, second_moment_decay, 0.999, "");

struct AdamTrackedValue {
  double val;
  double first_moment{};
  double second_moment{};

  inline AdamTrackedValue(double initial_val) : val(initial_val) {}

  inline void Update(const double gradient, const double learning_rate) {
    // Adam optimizer literature values
    static constexpr double kfirst_moment_decay_rate_ =
        absl::GetFlag(FLAGS_first_moment_decay).value_or(0.9);
    static constexpr double ksecond_moment_decay_rate_ =
        absl::GetFlag(FLAGS_first_moment_decay).value_or(0.999);
    first_moment = kfirst_moment_decay_rate_ * first_moment +
                   (1 - kfirst_moment_decay_rate_) * gradient;
    second_moment = ksecond_moment_decay_rate_ * second_moment +
                    (1 - ksecond_moment_decay_rate_) * gradient * gradient;
    val =
        val +
        learning_rate * first_moment /
            (std::sqrt(second_moment) +
             std::numeric_limits<double>::
                 epsilon());  // normally minus but in this case we already have the gradient going in the opposite direction
  }

  friend auto operator<<(std::ostream& os, const AdamTrackedValue& p)
      -> std::ostream& {
    os << "[" << p.val << ", " << p.first_moment << ", " << p.second_moment
       << "]";
    return os;
  }
};

struct TrackedTransformValues {
  AdamTrackedValue x, y, z, rx, ry, rz;

  inline TrackedTransformValues(const TransformValues& untracked_values)
      : x(untracked_values.x),
        y(untracked_values.y),
        z(untracked_values.z),
        rx(untracked_values.rx),
        ry(untracked_values.ry),
        rz(untracked_values.rz) {}

  inline void Update(const TransformValues& step, const double learning_rate) {
    x.Update(step.x, learning_rate);
    y.Update(step.y, learning_rate);
    z.Update(step.z, learning_rate);
    ry.Update(step.ry, learning_rate);
    if (false) {  // TODO make dependent on yaw_only
      rx.Update(step.rx, learning_rate);
      rz.Update(step.rz, learning_rate);
    }
  }

  friend auto operator<<(std::ostream& os, const TrackedTransformValues& p)
      -> std::ostream& {
    os << "x: " << p.x << "\ty: " << p.y << "\tz: " << p.z << "\trx: " << p.rx
       << "\try: " << p.ry << "\trz: " << p.rz << std::endl;
    return os;
  }
};

struct TransformDecomposition {
  Eigen::Matrix4d translation;
  Eigen::Matrix4d Rx;
  Eigen::Matrix4d Ry;
  Eigen::Matrix4d Rz;

  auto UpdateTransformDecomposition(const TransformValues& new_vals) {
    Rx(1, 1) = cos(new_vals.rx);
    Rx(2, 1) = sin(new_vals.rx);
    Rx(1, 2) = -sin(new_vals.rx);
    Rx(2, 2) = cos(new_vals.rx);

    Ry(0, 0) = cos(new_vals.ry);
    Ry(0, 2) = sin(new_vals.ry);
    Ry(2, 0) = -sin(new_vals.ry);
    Ry(2, 2) = cos(new_vals.ry);

    Rz(0, 0) = cos(new_vals.rz);
    Rz(0, 1) = -sin(new_vals.rz);
    Rz(1, 0) = sin(new_vals.rz);
    Rz(1, 1) = cos(new_vals.rz);

    translation(0, 3) = new_vals.x;
    translation(1, 3) = new_vals.y;
    translation(2, 3) = new_vals.z;
  }

  auto UpdateTransformDecomposition(const TrackedTransformValues& new_vals) {
    Rx(1, 1) = cos(new_vals.rx.val);
    Rx(2, 1) = sin(new_vals.rx.val);
    Rx(1, 2) = -sin(new_vals.rx.val);
    Rx(2, 2) = cos(new_vals.rx.val);

    Ry(0, 0) = cos(new_vals.ry.val);
    Ry(0, 2) = sin(new_vals.ry.val);
    Ry(2, 0) = -sin(new_vals.ry.val);
    Ry(2, 2) = cos(new_vals.ry.val);

    Rz(0, 0) = cos(new_vals.rz.val);
    Rz(0, 1) = -sin(new_vals.rz.val);
    Rz(1, 0) = sin(new_vals.rz.val);
    Rz(1, 1) = cos(new_vals.rz.val);

    translation(0, 3) = new_vals.x.val;
    translation(1, 3) = new_vals.y.val;
    translation(2, 3) = new_vals.z.val;
  }
};

auto ExtractTranslationAndRotation(const Eigen::Matrix4d& transform_mat)
    -> TransformValues;

auto SeparateTranslationAndRotationMatrices(const TransformValues& combined)
    -> TransformDecomposition;

inline auto PoseOffField(frc::Pose3d pose) -> bool {
  constexpr double kerror_margin = 0.2;
  return pose.X().value() < 0 - kerror_margin ||
         pose.X().value() > 16.54 + kerror_margin ||
         pose.Y().value() < 0 - kerror_margin ||
         pose.Y().value() >
             8 + kerror_margin /* || pose.Z().value() > kerror_margin*/;
}

}  // namespace utils
