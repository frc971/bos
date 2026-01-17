#include "src/localization/absolute_apriltag_layout.h"

namespace localization {

auto Pose3dToPoint3f(const frc::Pose3d& pose) -> cv::Point3f {
  // We ignore rotation
  return {static_cast<float>(pose.X()), static_cast<float>(pose.Y()),
          static_cast<float>(pose.Z())};
}

// Gives tag corners in wpilib coordinates
AbsoluteAprilTagLayout::AbsoluteAprilTagLayout(
    const frc::AprilTagFieldLayout& layout, float tag_size) {
  // top left corner from the camera's perspective, so it is the top right corner of the tag
  frc::Transform3d top_left_corner_transform{units::meter_t{0},
                                             units::meter_t{-(tag_size / 2)},
                                             units::meter_t{tag_size / 2},
                                             {}};

  frc::Transform3d top_right_corner_transform{units::meter_t{0},
                                              units::meter_t{tag_size / 2},
                                              units::meter_t{tag_size / 2},
                                              {}};

  frc::Transform3d bottom_right_corner_transform{
      units::meter_t{0},
      units::meter_t{tag_size / 2},
      units::meter_t{-(tag_size / 2)},
      {}};

  frc::Transform3d bottom_left_corner_transform{units::meter_t{0},
                                                units::meter_t{-(tag_size / 2)},
                                                units::meter_t{-(tag_size / 2)},
                                                {}};

  for (frc::AprilTag apriltag : layout.GetTags()) {
    frc::Pose3d pose = apriltag.pose;
    frc::Pose3d top_left_corner = pose.TransformBy(top_left_corner_transform);
    frc::Pose3d top_right_corner = pose.TransformBy(top_right_corner_transform);
    frc::Pose3d bottom_right_corner =
        pose.TransformBy(bottom_right_corner_transform);
    frc::Pose3d bottom_left_corner =
        pose.TransformBy(bottom_left_corner_transform);

    absolute_apriltag_layout_[apriltag.ID] = std::vector<cv::Point3f>{
        Pose3dToPoint3f(top_left_corner), Pose3dToPoint3f(top_right_corner),
        Pose3dToPoint3f(bottom_right_corner),
        Pose3dToPoint3f(bottom_left_corner)};
  }
}

auto AbsoluteAprilTagLayout::GetTagPoints(int tag_id)
    -> std::optional<std::vector<cv::Point3f>> {
  return absolute_apriltag_layout_.contains(tag_id)
             ? std::make_optional<std::vector<cv::Point3f>>(
                   absolute_apriltag_layout_[tag_id])
             : std::nullopt;
}
}  // namespace localization
