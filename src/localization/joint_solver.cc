#include "src/localization/joint_solver.h"
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

JointSolver::JointSolver(const std::string& intrinsics_path,
                         const std::string& extrinsics_path,
                         const frc::AprilTagFieldLayout& layout, int tag_size)
    : layout_(layout, tag_size),
      camera_matrix_(
          camera_matrix_from_json(utils::read_intrinsics(intrinsics_path))),
      distortion_coefficients_(distortion_coefficients_from_json(
          utils::read_intrinsics(intrinsics_path))) {}

JointSolver::JointSolver(camera::Camera camera_config,
                         const frc::AprilTagFieldLayout& layout, int tag_size)
    : JointSolver(camera::camera_constants[camera_config].intrinsics_path,
                  camera::camera_constants[camera_config].extrinsics_path,
                  layout, tag_size) {}

auto JointSolver::EstimatePosition(
    const std::vector<tag_detection_t>& detections)
    -> std::vector<position_estimate_t> {
  if (detections.empty()) {
    return {};
  }
  std::vector<position_estimate_t> position_estimates(detections.size());
  std::vector<cv::Point3f> tag_corners;
  std::vector<cv::Point2f> image_points;
  for (const auto& detection : detections) {
    std::optional<std::vector<cv::Point3f>> tag_corner =
        layout_.GetTagPoints(detection.tag_id);
    if (!tag_corner.has_value()) {
      LOG(WARNING) << "Got invalid tag id: " << detection.tag_id
                   << ". Rejecting tag";
      continue;
    }
    for (int i = 0; i < 4; i++) {
      tag_corners.push_back(tag_corner.value()[i]);
      image_points.push_back(detection.corners[i]);
    }
  }

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output rotation vector
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);  // output translation vector
  cv::solvePnP(tag_corners, image_points, camera_matrix_,
               distortion_coefficients_, rvec, tvec, false, cv::SOLVEPNP_EPNP);

  // Absolute apriltag feild layout already gives in wpilib coordinates
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
      .timestamp = detections[0].timestamp};
  return {position_estimate};
}

}  // namespace localization
