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

const localization::tag_detection_t detection{
    .tag_id = 31,
    .corners = {cv::Point2d(100.0, 200.0), cv::Point2d(200.0, 200.0),
                cv::Point2d(200.0, 100.0), cv::Point2d(100.0, 100.0)},
    .timestamp = 0.0,
    .confidence = 0.0};

TEST(SolverTest, Basic) {
  localization::SquareSolver square_solver(camera::Camera::DUMMY_CAMERA);
  const std::vector<localization::tag_detection_t> fake_detections{detection};

  localization::position_estimate_t estimate =
      square_solver.EstimatePosition(fake_detections)[0];
  std::cout << "hard estimate:\n" << estimate << std::endl;
}
