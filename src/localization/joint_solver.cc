#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

using data_point_t = struct DataPoint {
  Eigen::Vector2d undistorted_point;
  camera::Camera source;
  Eigen::Vector4d field_to_tag_corner_homogenous;
};

constexpr auto sq(double num) -> double {
  return num * num;
}

using frc::AprilTagFieldLayout;

JointSolver::JointSolver(const std::vector<camera::Camera>& camera_constants_,
                         const AprilTagFieldLayout& layout)
    : robot_to_field_(Eigen::Matrix4d::Identity()) {
  // clang-format off
  const Eigen::Matrix4d rotate_yaw_cv = (Eigen::Matrix4d() <<
      -1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, -1, 0,
      0, 0, 0, 1).finished();
  // clang-format on
  for (const frc::AprilTag& tag : layout.GetTags()) {
    Eigen::Matrix4d field_to_tag = tag.pose.ToMatrix();
    utils::ChangeBasis(field_to_tag, utils::WPI_TO_CV);
    std::array<Eigen::Vector4d, 4> field_relative_corners;
    for (size_t i = 0; i < kapriltag_corners_eigen.size(); i++) {
      field_relative_corners[i] = field_to_tag * rotate_yaw_cv *
                                  utils::Homogenize(kapriltag_corners_eigen[i]);
    }
    tag_corners_[tag.ID] = field_relative_corners;
  }
  Eigen::Matrix<double, 3, 4> pi = Eigen::Matrix<double, 3, 4>::Zero();
  pi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  for (const camera::Camera& camera_config : camera_constants_) {
    const nlohmann::json intrinsics_json = utils::ReadIntrinsics(
        camera::camera_constants[camera_config].intrinsics_path);
    const Eigen::Matrix3d camera_matrix =
        utils::CameraMatrixFromJson<Eigen::Matrix3d>(intrinsics_json);
    const Eigen::Matrix<double, 3, 4> image_to_camera = camera_matrix * pi;
    const Eigen::Matrix4d camera_to_robot =
        utils::ExtrinsicsJsonToCameraToRobot(
            utils::ReadExtrinsics(
                camera::camera_constants[camera_config].extrinsics_path))
            .ToMatrix();
    const Eigen::Matrix<double, 3, 4> image_to_robot =
        image_to_camera * camera_to_robot;
    camera_matrices_.insert(
        {camera_config,
         {.image_to_robot = image_to_robot,
          .distortion_coefficients =
              utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics_json),
          .camera_matrix = utils::EigenToCvMat(camera_matrix)}});
  }
}

auto JointSolver::EstimatePosition(
    const std::map<camera::Camera, std::vector<tag_detection_t>>&
        all_cam_detections,
    const frc::Pose3d& starting_pose) -> position_estimate_t {
  if (all_cam_detections.empty()) {
    return {};
  }
  robot_to_field_ = starting_pose.ToMatrix().inverse();
  utils::ChangeBasis(robot_to_field_, utils::WPI_TO_CV);
  std::vector<data_point_t> data_points;
  for (const auto& pair : all_cam_detections) {
    const CameraMatrices& camera_mats = camera_matrices_.at(pair.first);
    for (const tag_detection_t& detection : pair.second) {
      std::vector<cv::Point2d> undistorted_corners;
      cv::undistortImagePoints(detection.corners, undistorted_corners,
                               camera_mats.camera_matrix,
                               camera_mats.distortion_coefficients);
      for (size_t i = 0; i < undistorted_corners.size(); i++) {
        Eigen::Vector2d undistorted_image_point;
        undistorted_image_point << undistorted_corners[i].x,
            undistorted_corners[i].y;
        const data_point_t datapoint = {
            .undistorted_point = undistorted_image_point,
            .source = pair.first,
            .field_to_tag_corner_homogenous =
                tag_corners_[detection.tag_id].value()[i]};
        data_points.push_back(datapoint);
      }
    }
  }

  double loss = std::numeric_limits<double>::infinity();
  int counter = 0;

  const double step = 1e-6;

  double x = robot_to_field_(0, 3);
  double y = robot_to_field_(1, 3);
  double z = robot_to_field_(2, 3);

  while (loss > kacceptable_reprojection_error && counter < 100000) {
    counter++;

    for (const data_point_t& data_point : data_points) {
      const auto& camera_mats = camera_matrices_.at(data_point.source);

      Eigen::Vector3d projection = camera_mats.image_to_robot *
                                   robot_to_field_ *
                                   data_point.field_to_tag_corner_homogenous;

      const double lambda = projection(2);
      projection /= lambda;
      const Eigen::Vector2d projection_error =
          (Eigen::Vector2d()
               << projection.x() - data_point.undistorted_point.x(),
           projection.y() - data_point.undistorted_point.y())
              .finished();
      loss = 0.5 * projection_error.squaredNorm();
      const Eigen::Vector3d d_projection_d_loss =
          (Eigen::Vector3d() << projection_error.x() / lambda,
           projection_error.y() / lambda,
           -projection_error.x() * projection.x() / lambda -
               projection_error.y() * projection.y() / lambda)
              .finished();
      const Eigen::Vector4d d_image_to_field_d_loss =
          camera_matrices_.at(data_point.source).image_to_robot.transpose() *
          d_projection_d_loss;
      const Eigen::Matrix4d d_robot_to_field_d_loss =
          d_image_to_field_d_loss *
          data_point.field_to_tag_corner_homogenous.transpose();
      robot_to_field_.block<3, 1>(0, 3) -=
          d_robot_to_field_d_loss.block<3, 1>(0, 3) / 1000000;
    }
  }

  Eigen::Matrix4d field_to_robot = robot_to_field_.inverse();
  utils::ChangeBasis(field_to_robot, utils::CV_TO_WPI);

  return {.pose = frc::Pose3d(field_to_robot), .variance = 0, .timestamp = 0};
}

}  // namespace localization
