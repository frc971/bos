#include "src/localization/square_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
namespace localization {

inline auto camera_matrix_from_json(json intrinsics) -> cv::Mat {
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics["fx"], 0, intrinsics["cx"], 0,
       intrinsics["fy"], intrinsics["cy"], 0, 0, 1);
  return camera_matrix;
}
inline auto distortion_coefficients_from_json(json intrinsics) -> cv::Mat {
  cv::Mat distortion_coefficients =
      (cv::Mat_<double>(1, 5) << intrinsics["k1"], intrinsics["k2"],
       intrinsics["p1"], intrinsics["p2"], intrinsics["k3"]);
  return distortion_coefficients;
}

SquareSolver::SquareSolver(const std::string& intrinsics_path,
                           const std::string& extrinsics_path,
                           frc::AprilTagFieldLayout layout,
                           std::vector<cv::Point3f> tag_corners)
    : layout_(std::move(layout)),
      tag_corners_(std::move(tag_corners)),
      camera_matrix_(
          camera_matrix_from_json(utils::read_intrinsics(intrinsics_path))),
      distortion_coefficients_(distortion_coefficients_from_json(
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

    position_estimate_t position_estimate{
        .pose = frc::Pose3d(frc::Translation3d(units::meter_t{translation_x},
                                               units::meter_t{translation_y},
                                               units::meter_t{translation_z}),
                            frc::Rotation3d(units::radian_t{rotation_x},
                                            units::radian_t{rotation_y},
                                            units::radian_t{rotation_z})),
        .distance = std::hypot(translation_x, translation_y),
        .timestamp = detection.timestamp};
    position_estimates.push_back(position_estimate);
  }
  return position_estimates;
}

}  // namespace localization
