#include "src/localization/get_field_relitive_position.h"
#include "src/utils/log.h"
namespace localization {

auto ToFieldRelitivePosition(tag_detection_t tag_relative_position,
                             frc::Transform3d camera_to_robot,
                             const frc::AprilTagFieldLayout& apriltag_layout,
                             bool verbose) -> tag_detection_t {

  frc::Transform3d camera_to_tag(
      units::meter_t{tag_relative_position.pose.X()},
      units::meter_t{-tag_relative_position.pose.Y()},
      units::meter_t{-tag_relative_position.pose.Z()},
      frc::Rotation3d(
          units::radian_t{tag_relative_position.pose.Rotation().X()},
          units::radian_t{-tag_relative_position.pose.Rotation().Y()},
          units::radian_t{-tag_relative_position.pose.Rotation().Z()} +
              180_deg));

  frc::Transform3d tag_to_camera = camera_to_tag.Inverse();

  if (verbose) {
    std::cout << "tag to camera: \n";
    PrintTransform3d(tag_to_camera);
    std::cout << "\n\n";
  }

  frc::Pose3d tag_pose =
      apriltag_layout.GetTagPose(tag_relative_position.tag_id).value();

  if (verbose) {
    std::cout << "tag id: " << tag_relative_position.tag_id << std::endl;
    std::cout << "tagpose: \n";
    PrintPose3d(tag_pose);
    std::cout << "\n\n";
  }

  frc::Pose3d camera_pose = tag_pose.TransformBy(tag_to_camera);

  if (verbose) {
    std::cout << "camerapose: \n";
    PrintPose3d(camera_pose);
    std::cout << "\n\n";
  }

  frc::Pose3d robot_pose = camera_pose.TransformBy(camera_to_robot);

  tag_detection_t field_relative_pose;

  field_relative_pose.tag_id = tag_relative_position.tag_id;

  field_relative_pose.pose = robot_pose;

  field_relative_pose.distance = tag_relative_position.distance;

  field_relative_pose.timestamp = tag_relative_position.timestamp;

  return field_relative_pose;
}

auto ExtrinsicsJsonToCameraToRobot(nlohmann::json extrinsics_json)
    -> frc::Transform3d {
  frc::Pose3d camera_pose(
      units::meter_t{extrinsics_json["translation_x"]},
      units::meter_t{extrinsics_json["translation_y"]},
      units::meter_t{extrinsics_json["translation_z"]},
      frc::Rotation3d(units::radian_t{extrinsics_json["rotation_x"]},
                      units::radian_t{extrinsics_json["rotation_y"]},
                      units::radian_t{extrinsics_json["rotation_z"]}));
  frc::Transform3d robot_to_camera(frc::Pose3d(), camera_pose);
  return robot_to_camera.Inverse();
}

auto ToFieldRelitivePosition(std::vector<tag_detection_t> detections,
                             frc::Transform3d camera_to_robot,
                             const frc::AprilTagFieldLayout& apriltag_layout,
                             bool verbose) -> std::vector<tag_detection_t> {
  for (tag_detection_t& detection : detections) {
    detection = ToFieldRelitivePosition(detection, camera_to_robot,
                                        apriltag_layout, verbose);
  }
  return detections;
}
}  // namespace localization
