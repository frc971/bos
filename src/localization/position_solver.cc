#include "src/localization/position_solver.h"
#include "absl/flags/flag.h"

ABSL_FLAG(std::string, apriltag_field_layout_path,          // NOLINT
          "/bos/constants/field_map_apr_01_20_22_53.json",  // NOLINT
          "Path to the JSON file containing AprilTag field layout");

namespace localization {

auto GetAprilTagFieldLayout() -> const frc::AprilTagFieldLayout& {
  static const frc::AprilTagFieldLayout layout =
      frc::AprilTagFieldLayout::LoadField(
          absl::GetFlag(FLAGS_apriltag_field_layout_path));
  return layout;
}

}  // namespace localization
