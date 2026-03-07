#pragma once

#include <frc/geometry/Pose3d.h>
#include <ostream>
#include "src/localization/position.h"

namespace frc {

void PrintTo(const frc::Pose3d& pose, std::ostream* os);

auto wrap_compare(double angle_diff) -> double;

auto operator==(const localization::position_estimate_t& lhs,
                const localization::position_estimate_t& rhs) -> bool;

}  // namespace frc

namespace test_utils {
constexpr auto deg2rad(double degrees) -> double {
  return degrees * M_PI / 180;
}
static const std::array<cv::Point2d, 4> fake_image_points = {
    {cv::Point2d(100.0f, 100.0f), cv::Point2d(200.0f, 100.0f),
     cv::Point2d(200.0f, 200.0f), cv::Point2d(100.0f, 200.0f)}};
static const std::vector<localization::tag_detection_t> fake_detections(
    {{.tag_id = 31,
      .corners = fake_image_points,
      .timestamp = 0.0,
      .confidence = 0.0}});

static const frc::Transform3d joint_solve_input_noise(
    frc::Translation3d(units::meter_t{0.4}, units::meter_t{0.5},
                       units::meter_t{0.2}),
    frc::Rotation3d(units::degree_t{deg2rad(3)}, units::degree_t{deg2rad(3)},
                    units::degree_t{deg2rad(3)}));
static constexpr double ERROR_MARGIN_TRANSLATION = 0.01;
static constexpr double ERROR_MARGIN_EULER_ANGLE = deg2rad(1.0);  // 1 deg
}  // namespace test_utils
