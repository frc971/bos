#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <iostream>
#include <numbers>
#include <vector>
#include <opencv2/core/types.hpp>
#include "src/localization/position.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/intrinsics_from_json.h"

constexpr double ERROR_MARGIN_TRANSLATION = 0.01;
constexpr double ERROR_MARGIN_EULER_ANGLE = 0.01745; // 1 deg

namespace frc {

void PrintTo(const frc::Pose3d& pose, std::ostream* os) {
  *os << "Pose{"
      << "t = [" << pose.Translation().X().value() << ", "
      << pose.Translation().Y().value() << ", "
      << pose.Translation().Z().value() << "], "
      << "R = [" << pose.Rotation().X().value() << ", "
      << pose.Rotation().Y().value() << ", "
      << pose.Rotation().Z().value() << "]"
      << "}";
}

auto wrap_compare(double angle_diff) -> double {
  double diff = std::abs(angle_diff);
  if (std::abs(diff - 2 * std::numbers::pi) < ERROR_MARGIN_EULER_ANGLE) {
    diff = 0;
  }
  return diff;
}

inline auto operator==(const localization::position_estimate_t& lhs,
                       const localization::position_estimate_t& rhs) -> bool {
  if (lhs.pose.Translation().Distance(rhs.pose.Translation()).value() >
      ERROR_MARGIN_TRANSLATION) {
    return false;
  }

  const frc::Rotation3d rot_diff = lhs.pose.Rotation() - rhs.pose.Rotation();
  const double euler_diff =
      wrap_compare(rot_diff.X().value()) +
      wrap_compare(rot_diff.Y().value()) +
      wrap_compare(rot_diff.Z().value());

  return euler_diff == 0;
}

}  

constexpr int kimage_width = 20;
constexpr int kimage_height = 20;

const cv::Point2f tag_center{kimage_width / 2.0f,
                             kimage_height / 2.0f};

constexpr std::array<int, 2> ktag_ids = {15, 31};

constexpr float kNearDx = 40.0f;
constexpr float kNearDy = 40.0f;

constexpr float kFarDx = 5.0f;
constexpr float kFarDy = 5.0f;

auto MakeSquare(const cv::Point2f& center, float dx, float dy)
    -> std::array<cv::Point2f, 4> {
  return {
      cv::Point2f(center.x - dx, center.y + dy),
      cv::Point2f(center.x + dx, center.y + dy),
      cv::Point2f(center.x + dx, center.y - dy),
      cv::Point2f(center.x - dx, center.y - dy)
  };
}

auto ExpectedPoseForTag(int id) -> localization::position_estimate_t {
  frc::Pose3d flipped_tag =
      localization::kapriltag_layout.GetTagPose(id).value();

  flipped_tag = frc::Pose3d(
      flipped_tag.Translation(),
      frc::Rotation3d(
          flipped_tag.Rotation().X(),
          flipped_tag.Rotation().Y(),
          flipped_tag.Rotation().Z() +
              units::radian_t{-std::numbers::pi / 2.0}));

  return localization::position_estimate_t{flipped_tag, 0, 0};
}

auto SolveOnce(localization::SquareSolver& solver,
               int tag_id,
               const std::array<cv::Point2f, 4>& image_points)
    -> localization::position_estimate_t {
  const localization::tag_detection_t detection{
      .tag_id = tag_id,
      .corners = image_points};

  const std::vector<localization::tag_detection_t> detections{detection};

  return solver.EstimatePosition(detections)[0];
}

TEST(SolverTest, NEAR) {
  localization::SquareSolver solver(camera::Camera::DUMMY_CAMERA);

  const auto image_points =
      MakeSquare(tag_center, kNearDx, kNearDy);

  for (int id : ktag_ids) {
    auto estimate = SolveOnce(solver, id, image_points);
    auto expected = ExpectedPoseForTag(id);

    EXPECT_EQ(estimate, expected);
  }
}

TEST(SolverTest, FAR) {
  localization::SquareSolver solver(camera::Camera::DUMMY_CAMERA);

  const auto image_points =
      MakeSquare(tag_center, kFarDx, kFarDy);

  for (int id : ktag_ids) {
    auto estimate = SolveOnce(solver, id, image_points);
    auto expected = ExpectedPoseForTag(id);

    EXPECT_EQ(estimate, expected);
  }
}
