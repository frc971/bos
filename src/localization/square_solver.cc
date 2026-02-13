#include "src/localization/square_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/intrinsics_from_json.h"
#include "src/utils/log.h"
#include "src/utils/transform.h"

static const cv::Mat zero_vec = (cv::Mat_<double>(3, 1) << 0, 0, 0);

namespace localization {

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
      camera_to_robot_(utils::EigenToCvMat(
          ExtrinsicsJsonToCameraToRobot(utils::read_extrinsics(extrinsics_path))
              .ToMatrix())) {
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, std::numbers::pi);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  rotate_z_ = utils::MakeTransform(rvec, tvec);
  invert_translation_ = cv::Mat::eye(4, 4, CV_64F) * -1;
  invert_translation_.at<double>(3, 3) = 1;
}

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
  std::vector<position_estimate_t> position_estimates;
  position_estimates.reserve(detections.size());
  for (const auto& detection : detections) {
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output translation vector
    cv::solvePnP(tag_corners_, detection.corners, camera_matrix_,
                 distortion_coefficients_, rvec, tvec, false,
                 cv::SOLVEPNP_IPPE_SQUARE);

    const double translation_x = tvec.ptr<double>()[2];
    const double translation_y = tvec.ptr<double>()[0];

    utils::ConvertOpencvCoordinateToWpilib(tvec);
    utils::ConvertOpencvCoordinateToWpilib(rvec);

    cv::Mat camera_to_tag_rotation = utils::MakeTransform(rvec, zero_vec);
    cv::Mat camera_to_tag_translation = utils::MakeTransform(zero_vec, tvec);
    cv::Mat tag_to_camera_rotation = camera_to_tag_rotation.inv();
    cv::Mat tag_to_camera_translation =
        invert_translation_ * camera_to_tag_translation * invert_translation_;
    cv::Mat tag_to_camera = tag_to_camera_translation * tag_to_camera_rotation;
    cv::Mat field_to_tag = utils::EigenToCvMat(
        localization::kapriltag_layout.GetTagPose(detection.tag_id)
            .value()
            .ToMatrix());
    frc::Pose3d robot_pose(utils::CvMatToEigen(
        ((field_to_tag * rotate_z_) * tag_to_camera) * camera_to_robot_));

    position_estimates.push_back({robot_pose,
                                  std::hypot(translation_x, translation_y),
                                  detection.timestamp});
  }

  return position_estimates;
}

}  // namespace localization
