#include "src/localization/square_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/log.h"
#include "src/utils/transform.h"

static const cv::Mat zero_vec = (cv::Mat_<double>(3, 1) << 0, 0, 0);

namespace localization {

SquareSolver::SquareSolver(const std::string& intrinsics_path,
                           const std::string& extrinsics_path,
                           frc::AprilTagFieldLayout layout,
                           std::vector<cv::Point3f> tag_corners)
    : layout_(std::move(layout)),
      tag_corners_(std::move(tag_corners)),
      camera_matrix_(utils::CameraMatrixFromJson<cv::Mat>(
          utils::ReadIntrinsics(intrinsics_path))),
      distortion_coefficients_(utils::DistortionCoefficientsFromJson<cv::Mat>(
          utils::ReadIntrinsics(intrinsics_path))),
      camera_to_robot_(
          utils::EigenToCvMat(utils::ExtrinsicsJsonToCameraToRobot(
                                  utils::ReadExtrinsics(extrinsics_path))
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

    cv::Mat camera_to_tag = utils::MakeTransform(rvec, tvec);
    cv::Mat tag_to_camera = camera_to_tag.inv();
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
