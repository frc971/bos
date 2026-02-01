#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/extrinsics_from_json.h"
#include "src/utils/intrinsics_from_json.h"

namespace localization {
using frc::AprilTagFieldLayout;

static const Eigen::MatrixXd PI = []() {
  Eigen::Matrix<double, 3, 4> pi = Eigen::Matrix<double, 3, 4>::Zero();
  pi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  return pi;
}();

auto cvPoseToEigen(cv::Mat pose) -> Eigen::MatrixX4d {
  Eigen::Matrix4d eigen;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      eigen(r, c) = pose.at<double>(r, c);
    }
  }
  return eigen;
}

auto inverse(cv::Mat& rvec, cv::Mat& tvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);

  cv::Mat R_inv = R.t();
  tvec = -R_inv * tvec;

  cv::Mat rvec_inv;
  cv::Rodrigues(R_inv, rvec);
}

auto createTransformMatrix(double rx, double ry, double rz, double tx,
                           double ty, double tz) -> cv::Mat {
  // --- Rotation matrices ---
  cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(rx), -sin(rx), 0,
                sin(rx), cos(rx));

  cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(ry), 0, sin(ry), 0, 1, 0,
                -sin(ry), 0, cos(ry));

  cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(rz), -sin(rz), 0, sin(rz),
                cos(rz), 0, 0, 0, 1);

  // Combined rotation: Z * Y * X
  cv::Mat R = Rz * Ry * Rx;

  // --- Translation vector ---
  cv::Mat t = (cv::Mat_<double>(3, 1) << tx, ty, tz);

  // --- 4x4 Homogeneous transformation matrix ---
  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);  // initialize identity
  R.copyTo(T(cv::Rect(0, 0, 3, 3)));       // top-left 3x3 rotation
  t.copyTo(T(cv::Rect(3, 0, 1, 3)));       // top-right 3x1 translation

  return T;
}

auto createTransformMatrix(frc::Pose3d transform) -> cv::Mat {
  return createTransformMatrix(
      transform.Rotation().Y().to<double>(),
      transform.Rotation().Z().to<double>(),
      transform.Rotation().X().to<double>(), transform.Y().to<double>(),
      transform.Z().to<double>(), transform.X().to<double>());
}

auto createTransformMatrix(cv::Mat rvec, cv::Mat tvec) -> cv::Mat {
  return createTransformMatrix(rvec.at<double>(0), rvec.at<double>(1),
                               rvec.at<double>(2), tvec.at<double>(0),
                               tvec.at<double>(1), tvec.at<double>(2));
}

JointSolver::JointSolver(const std::string& intrinsics_path,
                         const std::string& extrinsics_path,
                         AprilTagFieldLayout layout, double tag_size)
    : layout_(std::move(layout)),
      camera_matrix_(utils::camera_matrix_from_json<cv::Mat>(
          utils::read_intrinsics(intrinsics_path))),
      distortion_coefficients_(
          utils::distortion_coefficients_from_json<cv::Mat>(
              utils::read_intrinsics(intrinsics_path))),
      camera_to_robot_(utils::ExtrinsicsJsonToCameraToRobot(
          utils::read_extrinsics(extrinsics_path))) {}

JointSolver::JointSolver(camera::Camera camera_config,
                         const frc::AprilTagFieldLayout& layout,
                         double tag_size)
    : JointSolver(camera::camera_constants[camera_config].intrinsics_path,
                  camera::camera_constants[camera_config].extrinsics_path,
                  layout, tag_size) {
  for (const auto& tag : layout.GetTags()) {
    // TODO document
    std::vector<cv::Point3f> absolute_tag_corners;
    absolute_tag_corners.reserve(4);
    frc::Pose3d tag_pose = tag.pose;
    auto feild_to_tag = createTransformMatrix(tag_pose);
    feild_to_tag.at<double>(0, 3) = -feild_to_tag.at<double>(0, 3);
    feild_to_tag.at<double>(0, 2) = -feild_to_tag.at<double>(0, 2);
    feild_to_tag.at<double>(0, 1) = -feild_to_tag.at<double>(0, 1);
    feild_to_tag.at<double>(0, 0) = -feild_to_tag.at<double>(0, 0);

    for (const cv::Point3f& apriltag_corner : kapriltag_corners) {
      cv::Mat tag_corner = (cv::Mat_<double>(4, 1) << -apriltag_corner.x,
                            apriltag_corner.y, apriltag_corner.z, 1.0);
      cv::Mat result = feild_to_tag * tag_corner;
      absolute_tag_corners.push_back(cv::Point3d(
          result.at<double>(0), result.at<double>(1), result.at<double>(2)));
    }
    absolute_apriltag_corners_.insert({tag.ID, absolute_tag_corners});
  }
}

auto JointSolver::EstimatePosition(
    const std::vector<tag_detection_t>& detections)
    -> std::vector<position_estimate_t> {
  std::vector<cv::Point3d> apriltag_corners;
  std::vector<cv::Point2d> image_points;
  for (auto& detection : detections) {
    auto apriltag_corner = absolute_apriltag_corners_.find(detection.tag_id);
    if (apriltag_corner == absolute_apriltag_corners_.end()) {
      LOG(WARNING) << "Invalid tag id: " << detection.tag_id;
      continue;
    }
    apriltag_corners.insert(apriltag_corners.end(),
                            apriltag_corner->second.begin(),
                            apriltag_corner->second.end());
    image_points.insert(image_points.end(), detection.corners.begin(),
                        detection.corners.end());
  }

  if (apriltag_corners.empty()) {
    // Need at least one tag
    return {};
  }

  // detections should all have the same timestamp (if they come from the same camera)
  const double timestamp = detections[0].timestamp;

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::solvePnP(apriltag_corners, image_points, camera_matrix_,
               distortion_coefficients_, rvec, tvec, false,
               cv::SOLVEPNP_ITERATIVE);

  inverse(rvec, tvec);

  const double translation_x = tvec.ptr<double>()[2];
  const double translation_y = tvec.ptr<double>()[0];
  const double translation_z = tvec.ptr<double>()[1];

  const double rotation_x = rvec.ptr<double>()[2];
  const double rotation_y = -rvec.ptr<double>()[0];
  const double rotation_z = rvec.ptr<double>()[1];

  frc::Pose3d camera_pose(
      units::meter_t{translation_x}, units::meter_t{-translation_y},
      units::meter_t{translation_z},
      frc::Rotation3d(units::radian_t{rotation_x}, units::radian_t{rotation_y},
                      units::radian_t{rotation_z}));

  frc::Pose3d robot_pose = camera_pose.TransformBy(camera_to_robot_);
  return {position_estimate_t{
      .pose = robot_pose, .distance = 0, .timestamp = timestamp}};
}

}  // namespace localization
