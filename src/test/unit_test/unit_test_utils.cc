#include "unit_test_utils.h"

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
  if (std::abs(diff - 2 * std::numbers::pi) <
      test_utils::ERROR_MARGIN_EULER_ANGLE) {
    diff = 0;
  }
  return diff;
}

auto operator==(const localization::position_estimate_t& lhs,
                const localization::position_estimate_t& rhs) -> bool {
  if (lhs.pose.Translation().Distance(rhs.pose.Translation()).value() >
      test_utils::ERROR_MARGIN_TRANSLATION)
    return false;

  const frc::Rotation3d rot_diff = lhs.pose.Rotation() - rhs.pose.Rotation();
  const double euler_diff = wrap_compare(rot_diff.X().value()) +
                            wrap_compare(rot_diff.Y().value()) +
                            wrap_compare(rot_diff.Z().value());

  return euler_diff == 0;
}

}  // namespace frc
