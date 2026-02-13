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

auto ConvertOpencvCoordinateToWpilib(cv::Mat& vec) -> void;

auto ConvertOpencvTransformationMatrixToWpilibPose(const cv::Mat& matrix)
    -> frc::Pose3d;

auto Pose3dToCvMat(frc::Pose3d) -> cv::Mat;
}  // namespace utils
