#include "src/localization/square_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/intrinsics_from_json.h"
#include "src/utils/log.h"

auto MakeTransform(const cv::Mat& rvec, const cv::Mat& tvec) -> cv::Mat {
  CV_Assert(rvec.total() == 3 && tvec.total() == 3);

  cv::Mat R;
  cv::Rodrigues(rvec, R);  // 3x3

  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

  R.copyTo(T(cv::Rect(0, 0, 3, 3)));

  T.at<double>(0, 3) = tvec.at<double>(0);
  T.at<double>(1, 3) = tvec.at<double>(1);
  T.at<double>(2, 3) = tvec.at<double>(2);

  return T;
}

template <typename Derived>
auto EigenToCvMat(const Eigen::MatrixBase<Derived>& mat) -> cv::Mat {
  cv::Mat cvMat(mat.rows(), mat.cols(), CV_64F);
  Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      cvMat.ptr<double>(), mat.rows(), mat.cols()) = mat;
  return cvMat;
}

auto CvMatToEigen(const cv::Mat& mat) -> Eigen::Matrix4d {
  Eigen::Matrix4d out;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      out(i, j) = mat.at<double>(i, j);
    }
  }
  return out;
}

auto ConvertOpencvCoordinateToWpilib(cv::Mat& vec) {
  const double x = vec.ptr<double>()[2];
  const double y = vec.ptr<double>()[0];
  const double z = vec.ptr<double>()[1];
  vec.ptr<double>()[0] = x;
  vec.ptr<double>()[1] = y;
  vec.ptr<double>()[2] = z;
}

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
      camera_to_robot_(EigenToCvMat(
          ExtrinsicsJsonToCameraToRobot(utils::read_extrinsics(extrinsics_path))
              .ToMatrix())) {}

SquareSolver::SquareSolver(camera::Camera camera_config,
                           frc::AprilTagFieldLayout layout,
                           std::vector<cv::Point3f> tag_corners)
    : SquareSolver(camera::camera_constants[camera_config].intrinsics_path,
                   camera::camera_constants[camera_config].extrinsics_path,
                   std::move(layout), std::move(tag_corners)) {

  cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, std::numbers::pi);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  rotate_z_ = MakeTransform(rvec, tvec);
}

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

    ConvertOpencvCoordinateToWpilib(tvec);
    ConvertOpencvCoordinateToWpilib(rvec);

    cv::Mat camera_to_tag = MakeTransform(rvec, tvec);
    cv::Mat camera_to_tag_rotation = MakeTransform(rvec, zero_vec);
    cv::Mat camera_to_tag_translation = MakeTransform(zero_vec, tvec);
    cv::Mat tag_to_camera_rotation = camera_to_tag_rotation.inv();
    cv::Mat tag_to_camera_translation =
        rotate_z_ * camera_to_tag_translation * rotate_z_;
    cv::Mat tag_to_camera = tag_to_camera_translation * tag_to_camera_rotation;

    cv::Mat feild_to_tag =
        EigenToCvMat(localization::kapriltag_layout.GetTagPose(detection.tag_id)
                         .value()
                         .ToMatrix());

    frc::Pose3d robot_pose(CvMatToEigen(
        ((feild_to_tag * rotate_z_) * tag_to_camera) * camera_to_robot_));

    position_estimates.push_back({robot_pose,
                                  std::hypot(translation_x, translation_y),
                                  detection.timestamp});
  }

  return position_estimates;
}

}  // namespace localization
