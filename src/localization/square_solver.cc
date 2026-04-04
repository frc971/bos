#include "src/localization/square_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "Eigen/Dense"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/log.h"
#include "src/utils/transform.h"

static const cv::Mat zero_vec = (cv::Mat_<double>(3, 1) << 0, 0, 0);

namespace localization {

SquareSolver::SquareSolver(const std::string& intrinsics_path,
                           const std::string& extrinsics_path,
                           frc::AprilTagFieldLayout layout,
                           std::vector<cv::Point3d> tag_corners)
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
  cv::Mat rvec_cv = (cv::Mat_<double>(3, 1) << 0, std::numbers::pi, 0);
  cv::Mat rvec_wpi = (cv::Mat_<double>(3, 1) << 0, 0, std::numbers::pi);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  rotate_yaw_cv_ = utils::MakeTransform(rvec_cv, tvec);
  rotate_yaw_wpilib_ = utils::MakeTransform(rvec_wpi, tvec);
  invert_translation_ = cv::Mat::eye(4, 4, CV_64F) * -1;
  invert_translation_.at<double>(3, 3) = 1;
}

SquareSolver::SquareSolver(camera::camera_constant_t camera_constant,
                           frc::AprilTagFieldLayout layout,
                           std::vector<cv::Point3d> tag_corners)
    : SquareSolver(camera_constant.intrinsics_path.value(),
                   camera_constant.extrinsics_path.value(), std::move(layout),
                   std::move(tag_corners)) {}

auto SquareSolver::EstimatePosition(
    const std::vector<tag_detection_t>& detections, const bool reject_far_tags)
    -> std::vector<position_estimate_t> {
  std::vector<position_estimate_t> position_estimates;
  position_estimates.reserve(detections.size());
  for (const auto& detection : detections) {
    if (reject_far_tags) {
      const auto& c = detection.corners;
      const double area = 0.5 * std::abs((c[0].x - c[2].x) * (c[1].y - c[3].y) -
                                         (c[1].x - c[3].x) * (c[0].y - c[2].y));
      if (area < kmin_tag_area_pixels) {
        continue;
      }
    }
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output translation vector
    cv::solvePnP(tag_corners_, detection.corners, camera_matrix_,
                 distortion_coefficients_, rvec, tvec, false,
                 cv::SOLVEPNP_IPPE_SQUARE);

    if (reject_far_tags && cv::norm(tvec) > 5.0) {
      continue;
    }

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
        ((field_to_tag * rotate_yaw_wpilib_) * tag_to_camera) *
        camera_to_robot_));

    const double distance = std::hypot(translation_x, translation_y);

    position_estimates.push_back(position_estimate_t{
        .tag_ids = {detection.tag_id},
        .rejected_tag_ids = {},  // TODO
        .pose = robot_pose,
        .variance = Variance(1, distance, kvariance_min_, kvariance_scalar_),
        .timestamp = detection.timestamp,
        .num_tags = 1,
        .avg_tag_dist = distance});
  }

  return position_estimates;
}

auto SquareSolver::EstimatePositionAmbiguous(
    const std::vector<tag_detection_t>& detections, const bool reject_far_tags)
    -> std::vector<std::pair<position_estimate_t, position_estimate_t>> {

  std::vector<std::pair<position_estimate_t, position_estimate_t>>
      position_estimates;
  position_estimates.reserve(detections.size());

  for (const auto& detection : detections) {

    if (reject_far_tags) {
      const auto& c = detection.corners;
      const double area = 0.5 * std::abs((c[0].x - c[2].x) * (c[1].y - c[3].y) -
                                         (c[1].x - c[3].x) * (c[0].y - c[2].y));
      if (area < kmin_tag_area_pixels) {
        continue;
      }
    }

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    cv::solvePnPGeneric(tag_corners_, detection.corners, camera_matrix_,
                        distortion_coefficients_, rvecs, tvecs, false,
                        cv::SOLVEPNP_IPPE_SQUARE);

    if (rvecs.size() < 2 || tvecs.size() < 2) {
      continue;
    }

    auto build_estimate = [&](cv::Mat rvec,
                              cv::Mat tvec) -> position_estimate_t {
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
          ((field_to_tag * rotate_yaw_wpilib_) * tag_to_camera) *
          camera_to_robot_));

      const double distance = std::hypot(translation_x, translation_y);

      return position_estimate_t{
          .tag_ids = {detection.tag_id},
          .rejected_tag_ids = {},
          .pose = robot_pose,
          .variance = Variance(1, distance, kvariance_min_, kvariance_scalar_),
          .timestamp = detection.timestamp,
          .num_tags = 1,
          .avg_tag_dist = distance};
    };

    auto est1 = build_estimate(rvecs[0], tvecs[0]);
    auto est2 = build_estimate(rvecs[1], tvecs[1]);

    if (reject_far_tags &&
        (cv::norm(tvecs[0]) > 5.0 && cv::norm(tvecs[1]) > 5.0)) {
      continue;
    }

    position_estimates.emplace_back(est1, est2);
  }

  return position_estimates;
}

auto SquareSolver::EstimatePositionNew(
    const std::vector<tag_detection_t>& detections, const bool reject_far_tags)
    -> std::vector<position_estimate_t> {
  std::vector<position_estimate_t> position_estimates;
  position_estimates.reserve(detections.size());
  for (const tag_detection_t& detection : detections) {
    if (reject_far_tags) {
      const auto& c = detection.corners;
      const double area = 0.5 * std::abs((c[0].x - c[2].x) * (c[1].y - c[3].y) -
                                         (c[1].x - c[3].x) * (c[0].y - c[2].y));
      if (area < kmin_tag_area_pixels) {
        continue;
      }
    }
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output translation vector

    try {
      cv::solvePnP(tag_corners_, detection.corners, camera_matrix_,
                   distortion_coefficients_, rvec, tvec, false,
                   cv::SOLVEPNP_IPPE_SQUARE);
    } catch (std::exception& e) {
      LOG(WARNING) << "Caught solve pnp exception:\n" << e.what();
      return {};
    }

    double distance = cv::norm(tvec);
    if (reject_far_tags && distance > 5.0) {
      continue;
    }

    cv::Mat camera_to_tag = utils::MakeTransform(rvec, tvec);
    cv::Mat tag_to_camera = camera_to_tag.inv();
    cv::Mat field_to_tag = utils::EigenToCvMat(
        localization::kapriltag_layout.GetTagPose(detection.tag_id)
            .value()
            .ToMatrix());
    utils::ChangeBasis(field_to_tag, utils::WPI_TO_CV);
    cv::Mat field_to_robot =
        field_to_tag * rotate_yaw_cv_ * tag_to_camera * camera_to_robot_;
    utils::ChangeBasis(field_to_robot, utils::CV_TO_WPI);

    frc::Pose3d robot_pose(utils::CvMatToEigen(field_to_robot));

    position_estimates.push_back(position_estimate_t{
        .tag_ids = {detection.tag_id},
        .rejected_tag_ids = {},  // TODO
        .pose = robot_pose,
        .variance = distance * distance,
        .timestamp = detection.timestamp,
    });
  }
  return position_estimates;
}
}  // namespace localization
