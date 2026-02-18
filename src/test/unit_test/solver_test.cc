#include <gtest/gtest.h>
#include "src/localization/joint_solver.h"
#include "src/localization/position.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/transform.h"

constexpr double ERROR_MARGIN_TRANSLATION = 0.01;
constexpr double ERROR_MARGIN_EULER_ANGLE = 0.01745;  // 1 deg

namespace frc {
void PrintTo(const frc::Pose3d& pose, std::ostream* os) {
  *os << "Pose{"
      << "t = [" << pose.Translation().X().value() << ", "
      << pose.Translation().Y().value() << ", "
      << pose.Translation().Z().value() << "], "
      << "R = [" << pose.Rotation().X().value() << ", "
      << pose.Rotation().Y().value() << ", " << pose.Rotation().Z().value()
      << "]"
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
      ERROR_MARGIN_TRANSLATION)
    return false;
  const frc::Rotation3d rot_diff = lhs.pose.Rotation() - rhs.pose.Rotation();
  const double euler_diff = wrap_compare(rot_diff.X().value()) +
                            wrap_compare(rot_diff.Y().value()) +
                            wrap_compare(rot_diff.Z().value());
  return euler_diff == 0;
}
}  // namespace frc

constexpr int kimage_tag_width = 20;
constexpr int kimage_tag_height = 20;
const std::vector<int> ktag_ids = {31};
constexpr int kimage_width = 20;
constexpr int kimage_height = 20;
const cv::Point2f tag_center =
    cv::Point2f(kimage_width / 2.0, kimage_height / 2.0);
const int dx = kimage_tag_width / 2.0;
const int dy = kimage_tag_height / 2.0;
// bottom left, bottom right, top right, top left
const std::array<cv::Point2d, 4> image_points = {
    cv::Point2f(tag_center.x - dx, tag_center.y + dy),
    cv::Point2f(tag_center.x + dx, tag_center.y + dy),
    cv::Point2f(tag_center.x + dx, tag_center.y - dy),
    cv::Point2f(tag_center.x - dx, tag_center.y - dy)};

TEST(SolverTest, Basic) {
  for (const auto& point : image_points) {
    std::cout << point << std::endl;
  }
  localization::SquareSolver square_solver(camera::Camera::DUMMY_CAMERA);
  localization::JointSolver joint_solver(
      std::vector<camera::Camera>{camera::Camera::DUMMY_CAMERA});

  for (const int id : ktag_ids) {
    const localization::tag_detection_t fake_detection{.tag_id = id,
                                                       .corners = image_points};
    const std::vector<localization::tag_detection_t> fake_detections{
        fake_detection};

    localization::position_estimate_t estimate =
        square_solver.EstimatePosition(fake_detections)[0];
    frc::Pose3d flipped_tag =
        localization::kapriltag_layout.GetTagPose(id).value();
    flipped_tag = frc::Pose3d(
        flipped_tag.Translation(),
        frc::Rotation3d(flipped_tag.Rotation().X(), flipped_tag.Rotation().Y(),
                        flipped_tag.Rotation().Z() +
                            units::radian_t{-std::numbers::pi / 2.0}));
    localization::position_estimate_t expected{flipped_tag, 0, 0};
    ASSERT_EQ(estimate, expected);
    std::map<camera::Camera, std::vector<localization::tag_detection_t>>
        associated_detections;
    associated_detections.insert(
        {camera::Camera::DUMMY_CAMERA, fake_detections});
    Eigen::Matrix4d square_estimate = estimate.pose.ToMatrix();
    utils::ChangeBasis(square_estimate, utils::WPI_TO_CV);
    joint_solver.field_to_robot_ = square_estimate;
    joint_solver.EstimatePosition(associated_detections);
  }
}
