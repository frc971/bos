#include "src/utils/constants_from_json.h"
#include "third_party/971apriltag/971apriltag.h"

namespace utils {
template <>
auto camera_matrix_from_json<frc971::apriltag::CameraMatrix>(
    nlohmann::json intrinsics) -> frc971::apriltag::CameraMatrix {
  frc971::apriltag::CameraMatrix camera_matrix = {.fx = intrinsics["fx"],
                                                  .cx = intrinsics["cx"],
                                                  .fy = intrinsics["fy"],
                                                  .cy = intrinsics["cy"]};
  return camera_matrix;
}

template <>
auto distortion_coefficients_from_json<frc971::apriltag::DistCoeffs>(
    nlohmann::json intrinsics) -> frc971::apriltag::DistCoeffs {
  frc971::apriltag::DistCoeffs distortion_coefficients = {
      .k1 = intrinsics["k1"],
      .k2 = intrinsics["k2"],
      .p1 = intrinsics["p1"],
      .p2 = intrinsics["p2"],
      .k3 = intrinsics["k3"]};

  return distortion_coefficients;
}

template <>
auto camera_matrix_from_json<cv::Mat>(nlohmann::json intrinsics) -> cv::Mat {
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics["fx"], 0, intrinsics["cx"], 0,
       intrinsics["fy"], intrinsics["cy"], 0, 0, 1);
  return camera_matrix;
}

template <>
auto distortion_coefficients_from_json<cv::Mat>(nlohmann::json intrinsics)
    -> cv::Mat {
  cv::Mat distortion_coefficients =
      (cv::Mat_<double>(1, 5) << intrinsics["k1"], intrinsics["k2"],
       intrinsics["p1"], intrinsics["p2"], intrinsics["k3"]);
  return distortion_coefficients;
}

template <>
auto camera_matrix_from_json<Eigen::Matrix3d>(nlohmann::json intrinsics)
    -> Eigen::Matrix3d {
  Eigen::Matrix3d K;
  K << intrinsics["fx"], 0, intrinsics["cx"], 0, intrinsics["fy"],
      intrinsics["cy"], 0, 0, 1;
  return K;
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

}  // namespace utils
