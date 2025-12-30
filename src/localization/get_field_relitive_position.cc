#include "src/localization/get_field_relitive_position.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include "src/utils/log.h"
namespace localization {

tag_detection_t GetFeildRelitivePosition(
    tag_detection_t tag_relative_position, frc::Transform3d camera_to_robot,
    frc::AprilTagFieldLayout apriltag_layout, bool verbose) {

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

frc::Transform3d ExtrinsicsJsonToCameraToRobot(nlohmann::json extrinsics_json) {
  frc::Transform3d robot_to_camera(
      units::meter_t{static_cast<double>(extrinsics_json["translation_x"])},
      units::meter_t{static_cast<double>(extrinsics_json["translation_y"])},
      units::meter_t{static_cast<double>(extrinsics_json["translation_z"])},
      frc::Rotation3d(units::radian_t{extrinsics_json["rotation_x"]},
                      units::radian_t{extrinsics_json["rotation_y"]},
                      units::radian_t{extrinsics_json["rotation_z"]}));
  return robot_to_camera.Inverse();
}

std::vector<tag_detection_t> GetFeildRelitivePosition(
    std::vector<tag_detection_t> detections, frc::Transform3d camera_to_robot,
    frc::AprilTagFieldLayout apriltag_layout, bool verbose) {
  for (size_t i = 0; i < detections.size(); ++i) {
    detections[i] = GetFeildRelitivePosition(detections[i], camera_to_robot,
                                             apriltag_layout, verbose);
  }
  return detections;
}
}  // namespace localization
