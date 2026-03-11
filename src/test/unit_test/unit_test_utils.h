#pragma once

#include <frc/geometry/Pose3d.h>
#include <ostream>
#include "src/camera/camera_constants.h"
#include "src/localization/position.h"

namespace test_utils {
constexpr auto deg2rad(double degrees) -> double {
  return degrees * M_PI / 180;
}
// bottom-left, bottom-right, top-right, top-left
static const std::array<cv::Point2d, 4> fake_image_points = {
    {cv::Point2d(100.0f, 200.0f), cv::Point2d(200.0f, 200.0f),
     cv::Point2d(200.0f, 100.0f), cv::Point2d(100.0f, 100.0f)}};
static const std::vector<localization::tag_detection_t> fake_detections(
    {{.tag_id = 31,
      .corners = fake_image_points,
      .timestamp = 0.0,
      .confidence = 0.0}});

static const std::vector<camera::Camera> joint_solve_cameras =
    std::vector<camera::Camera>{camera::DEV_ORIN};
static constexpr double ERROR_MARGIN_TRANSLATION = 0.01;
static constexpr double ERROR_MARGIN_EULER_ANGLE = deg2rad(1.0);  // 1 deg
}  // namespace test_utils

namespace frc {

void PrintTo(const frc::Pose3d& pose, std::ostream* os);

}  // namespace frc

namespace localization {
auto wrap_compare(double angle_diff) -> double {
  double diff = std::abs(angle_diff);
  if (std::abs(diff - 2 * M_PI) < test_utils::ERROR_MARGIN_EULER_ANGLE) {
    diff = std::abs(diff - 2 * M_PI);
  }
  return diff;
}

auto operator==(const PositionEstimate& left, const PositionEstimate& right)
    -> bool {
  if (left.pose.Translation().Distance(right.pose.Translation()).value() >
      test_utils::ERROR_MARGIN_TRANSLATION)
    return false;

  const frc::Rotation3d rot_diff = left.pose.Rotation() - right.pose.Rotation();
  const double euler_diff = wrap_compare(rot_diff.X().value()) +
                            wrap_compare(rot_diff.Y().value()) +
                            wrap_compare(rot_diff.Z().value());
  std::cout << "I am here!" << std::endl;

  return euler_diff < test_utils::ERROR_MARGIN_EULER_ANGLE;
}
}  // namespace localization
