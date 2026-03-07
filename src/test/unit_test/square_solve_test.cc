#include <gtest/gtest.h>
#include <algorithm>
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

constexpr int kimage_tag_width = 10;
constexpr int kimage_tag_height = 10;
const std::vector<int> ktag_ids = {31};
constexpr int kimage_width = 20;
constexpr int kimage_height = 20;
const cv::Point2f tag_center =
    cv::Point2f(kimage_width / 2.0, kimage_height / 2.0);
const int dx = kimage_tag_width / 2.0;
const int dy = kimage_tag_height / 2.0;
std::array<cv::Point2d, 4> simple_image_points = {
    cv::Point2f(tag_center.x - dx, tag_center.y - dy),
    cv::Point2f(tag_center.x + dx, tag_center.y - dy),
    cv::Point2f(tag_center.x + dx, tag_center.y + dy),
    cv::Point2f(tag_center.x - dx, tag_center.y + dy)};

std::array<cv::Point2d, 4> harder_image_points = {
    {cv::Point2d(100.0f, 100.0f), cv::Point2d(200.0f, 100.0f),
     cv::Point2d(200.0f, 200.0f), cv::Point2d(100.0f, 200.0f)}};

TEST(SolverTest, Basic) {
  localization::SquareSolver square_solver(camera::Camera::DUMMY_CAMERA);
  for (const int id : ktag_ids) {
    const localization::tag_detection_t fake_detection{
        .tag_id = id, .corners = harder_image_points};
    const std::vector<localization::tag_detection_t> fake_detections{
        fake_detection};

    localization::position_estimate_t estimate =
        square_solver.EstimatePosition(fake_detections)[0];
    std::cout << "hard estimate:\n" << estimate << std::endl;
  }
}
