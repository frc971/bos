#include "src/localization/square_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/extrinsics_from_json.h"
#include "src/utils/intrinsics_from_json.h"

namespace localization {

SquareSolver::SquareSolver(const std::string& intrinsics_path,
                           const std::string& extrinsics_path,
                           frc::AprilTagFieldLayout layout,
                           std::vector<cv::Point3f> tag_corners)
    : layout_(std::move(layout)),
      tag_corners_(std::move(tag_corners)),
      camera_matrix_(utils::camera_matrix_from_json<cv::Mat>(
          utils::read_intrinsics(intrinsics_path))),
      distortion_coefficients_(
          utils::distortion_coefficients_from_json<cv::Mat>(
              utils::read_intrinsics(intrinsics_path))),
      camera_to_robot_(utils::ExtrinsicsJsonToCameraToRobot(
          utils::read_extrinsics(extrinsics_path))) {}

SquareSolver::SquareSolver(camera::Camera camera_config,
                           frc::AprilTagFieldLayout layout,
                           std::vector<cv::Point3f> tag_corners)
    : SquareSolver(camera::camera_constants[camera_config].intrinsics_path,
                   camera::camera_constants[camera_config].extrinsics_path,
                   std::move(layout), std::move(tag_corners)) {}

auto SquareSolver::EstimatePosition(
    const std::vector<tag_detection_t>& detections)
    -> std::vector<position_estimate_t> {
  // map?
  std::vector<position_estimate_t> position_estimates(detections.size());
  for (const auto& detection : detections) {
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output translation vector
    cv::solvePnP(tag_corners_, detection.corners, camera_matrix_,
                 distortion_coefficients_, rvec, tvec, false,
                 cv::SOLVEPNP_IPPE_SQUARE);

    // Currently we do not use transation z, rotation x and rotation y
    // Converting to wpi coordinates
    const double translation_x = tvec.ptr<double>()[2];
    const double translation_y = tvec.ptr<double>()[0];
    const double translation_z = tvec.ptr<double>()[1];

    const double rotation_x = rvec.ptr<double>()[2];
    const double rotation_y = rvec.ptr<double>()[0];
    const double rotation_z = rvec.ptr<double>()[1];

    // TODO check if there is a way of doing this in a better way
    frc::Transform3d camera_to_tag(
        units::meter_t{translation_x}, units::meter_t{-translation_y},
        units::meter_t{-translation_z},
        frc::Rotation3d(units::radian_t{rotation_x},
                        units::radian_t{-rotation_y},
                        units::radian_t{-rotation_z} + 180_deg));

    frc::Transform3d tag_to_camera = camera_to_tag.Inverse();

    auto maybe_tag_pose = layout_.GetTagPose(detection.tag_id);
    if (!maybe_tag_pose.has_value()) {
      LOG(WARNING) << "Got invalid tag id: " << detection.tag_id;
    }
    frc::Pose3d tag_pose = maybe_tag_pose.value();

    frc::Pose3d camera_pose = tag_pose.TransformBy(tag_to_camera);

    frc::Pose3d robot_pose = camera_pose.TransformBy(camera_to_robot_);

    position_estimates.push_back({robot_pose,
                                  std::hypot(translation_x, translation_y),
                                  detection.timestamp});
  }
  return position_estimates;
}

}  // namespace localization
