#pragma once

#include <optional>
#include <vector>

#include <frc/geometry/Translation2d.h>
#include <opencv2/core.hpp>

namespace utils {

// Converts a BGR image to HSV, selects pixels within the inclusive HSV range,
// and optionally undistorts the selected pixel coordinates. All outputs are
// supplied by the caller so their allocations can be reused between frames.
// Without a camera matrix, the returned points remain in pixel coordinates.
void HSVThreshold(
    const cv::Mat3b& bgr_image, const cv::Scalar& lower_bound,
    const cv::Scalar& upper_bound, std::vector<cv::Point2f>& thresholded_points,
    cv::Mat3b& hsv_image, cv::Mat1b& threshold_mask,
    const std::optional<cv::Matx33d>& camera_matrix = std::nullopt,
    const std::optional<cv::Vec<double, 5>>& distortion_coefficients =
        std::nullopt);

[[nodiscard]] auto DistortedPinholePointOffset(
    const cv::Point2f& point, float world_relative_vertical,
    const cv::Matx44f& camera_extrinsics_cv,
    const cv::Matx33f& camera_intrinsics) -> std::optional<frc::Translation2d>;

// Takes in points in the focal-length-normalized format used by cv::undistortPoints. If points aren't undistorted, use DistortedPinholePointOffset instead
[[nodiscard]] auto UndistortedPinholePointOffset(
    const cv::Point2f& point, float world_relative_vertical,
    const cv::Matx44f& camera_extrinsics_cv)
    -> std::optional<frc::Translation2d>;

}  // namespace utils
