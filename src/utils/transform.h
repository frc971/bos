#pragma once
#include <frc/geometry/Pose3d.h>
#include <opencv2/calib3d.hpp>
#include "src/utils/pch.h"

// Helper functions for converting between wpilib, opencv and eigen
namespace utils {
enum Basis { WPI_TO_CV, CV_TO_WPI };

const cv::Mat cv_to_wpilib = (cv::Mat_<double>(4, 4) << 0, 0, 1, 0, -1, 0, 0, 0,
                              0, -1, 0, 0, 0, 0, 0, 1);

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
};

auto ExtractTranslationAndRotation(const Eigen::Matrix4d& transform_mat)
    -> TransformValues;

auto SeparateTranslationAndRotationMatrices(const TransformValues& combined)
    -> std::array<Eigen::Matrix4d, 4>;

}  // namespace utils
