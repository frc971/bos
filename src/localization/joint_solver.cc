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

  double total_loss = std::numeric_limits<double>::infinity();
  int counter = 0;

  const double step = 1e-8;  // single consistent step

  while (total_loss > kacceptable_reprojection_error && counter < 100000) {
    counter++;

    total_loss = 0.0;
    Eigen::Matrix4d total_grad = Eigen::Matrix4d::Zero();

    for (const data_point_t& data_point : data_points) {
      const auto& camera_mats = camera_matrices_.at(data_point.source);

      Eigen::Vector3d projection = camera_mats.image_to_robot *
                                   robot_to_field_ *
                                   data_point.field_to_tag_corner_homogenous;

      const double lambda = projection(2);
      projection /= lambda;

      Eigen::Vector2d error_vec;
      error_vec << projection.x() - data_point.undistorted_point.x(),
          projection.y() - data_point.undistorted_point.y();

      // ----- squared loss -----
      total_loss += 0.5 * error_vec.squaredNorm();

      // ----- dL / d projection (before divide) -----
      Eigen::Vector3d dL_dproj;
      dL_dproj << error_vec.x() / lambda, error_vec.y() / lambda,
          -error_vec.x() * projection.x() / lambda -
              error_vec.y() * projection.y() / lambda;

      // ----- chain rule -----
      Eigen::Vector4d dL_dimage_to_field =
          camera_mats.image_to_robot.transpose() * dL_dproj;

      Eigen::Matrix4d dL_dT =
          dL_dimage_to_field *
          data_point.field_to_tag_corner_homogenous.transpose();

      total_grad += dL_dT;
    }

    // ----- Apply translation update -----
    robot_to_field_.block<3, 1>(0, 3) -= step * total_grad.block<3, 1>(0, 3);

    // ----- Apply rotation update -----
    Eigen::Matrix3d dR = total_grad.block<3, 3>(0, 0);
    Eigen::Matrix3d R = robot_to_field_.block<3, 3>(0, 0);

    Eigen::Matrix3d skew = 0.5 * (R.transpose() * dR - dR.transpose() * R);

    Eigen::Vector3d omega;
    omega << skew(2, 1), skew(0, 2), skew(1, 0);

    Eigen::Vector3d delta = -step * omega;

    Eigen::Matrix3d delta_skew;
    delta_skew << 0, -delta(2), delta(1), delta(2), 0, -delta(0), -delta(1),
        delta(0), 0;

    Eigen::Matrix3d dR_exp = Eigen::Matrix3d::Identity() + delta_skew;

    R = dR_exp * R;

    // re-orthogonalize
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        R, Eigen::ComputeFullU | Eigen::ComputeFullV);

    R = svd.matrixU() * svd.matrixV().transpose();

    robot_to_field_.block<3, 3>(0, 0) = R;

    if (counter % 1000 == 0) {
      std::cout << "iter " << counter << " loss: " << total_loss << std::endl;
    }
  }

  Eigen::Matrix4d field_to_robot = robot_to_field_.inverse();
  utils::ChangeBasis(field_to_robot, utils::CV_TO_WPI);

  return {.pose = frc::Pose3d(field_to_robot), .variance = 0, .timestamp = 0};
}

}  // namespace localization
