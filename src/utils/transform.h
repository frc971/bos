#pragma once
#include <frc/geometry/Pose3d.h>
#include <opencv2/calib3d.hpp>
#include "src/utils/pch.h"

// Helper functions for converting between wpilib, opencv and eigen
namespace utils {
auto MakeTransform(const cv::Mat& rvec, const cv::Mat& tvec) -> cv::Mat;

template <typename Derived>
auto EigenToCvMat(const Eigen::MatrixBase<Derived>& mat) -> cv::Mat;

auto CvMatToEigen(const cv::Mat& mat) -> Eigen::Matrix4d;

auto OpencvCoordinateToWpilib(cv::Mat& vec) -> void;

auto OpencvTransformationMatrixToPose3d(const cv::Mat& matrix) -> frc::Pose3d;

auto Pose3dToCvMat(frc::Pose3d) -> cv::Mat;

auto Transform3dToCvMat(frc::Transform3d transform) -> cv::Mat;

auto Point3dToHomogenizedMat(cv::Point3d point) -> cv::Mat;

auto CvMatToPoint3f(cv::Mat mat) -> cv::Point3d;

auto HomogenizePoint3d(cv::Point3d point) -> cv::Mat;

}  // namespace utils
