#pragma once
#include <frc/geometry/Pose3d.h>
#include <opencv2/core/mat.hpp>
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "src/utils/transform.h"

namespace utils {
void PrintPose3d(const frc::Pose3d& pose);
void PrintTransform3d(const frc::Transform3d& transform);
void PrintTransformationMatrix(
    const cv::Mat& mat, const std::optional<std::string>& name = std::nullopt);
void PrintPose2d(const frc::Pose2d& pose);
template <typename Derived>
inline void PrintTransformationMatrix(const Eigen::MatrixBase<Derived>& mat,
                                      const std::optional<std::string>& name) {
  PrintTransformationMatrix(utils::EigenToCvMat(mat));
}
}  // namespace utils
