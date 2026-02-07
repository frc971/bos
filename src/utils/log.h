#pragma once
#include <frc/geometry/Pose3d.h>
#include <opencv2/core/mat.hpp>
#include "absl/log/check.h"
#include "absl/log/log.h"

namespace utils {
void PrintPose3d(const frc::Pose3d& pose);
void PrintTransform3d(const frc::Transform3d& transform);
void PrintTransformationMatrix(const cv::Mat& mat);
}  // namespace utils
