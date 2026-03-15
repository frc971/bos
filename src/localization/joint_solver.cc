#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

using data_point_t = struct DataPoint {
  Eigen::Vector2d undistorted_point;
  std::string name;
  Eigen::Vector4d field_to_tag_corner_homogenous;
};

constexpr auto sq(double num) -> double {
  return num * num;
}

using frc::AprilTagFieldLayout;

JointSolver::JointSolver(
    const std::vector<camera::camera_constant_t>& camera_constants_,
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
  for (const camera::camera_constant_t& camera_constant : camera_constants_) {
    const nlohmann::json intrinsics_json =
        utils::ReadIntrinsics(camera_constant.intrinsics_path.value());
    const auto camera_matrix =
        utils::CameraMatrixFromJson<Eigen::Matrix3d>(intrinsics_json);
    const Eigen::Matrix<double, 3, 4> image_to_camera = camera_matrix * pi;
    const Eigen::Matrix4d camera_to_robot =
        utils::ExtrinsicsJsonToCameraToRobot(
            utils::ReadExtrinsics(camera_constant.extrinsics_path.value()))
            .ToMatrix();
    const Eigen::Matrix<double, 3, 4> image_to_robot =
        image_to_camera * camera_to_robot;
    camera_matrices_.insert(
        {camera_constant.name,
         {.image_to_robot = image_to_robot,
          .distortion_coefficients =
              utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics_json),
          .camera_matrix = utils::EigenToCvMat(camera_matrix)}});
  }
}

auto JointSolver::EstimatePosition(
    const std::map<std::string, std::vector<tag_detection_t>>&
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
            .name = pair.first,
            .field_to_tag_corner_homogenous =
                tag_corners_[detection.tag_id].value()[i]};
        data_points.push_back(datapoint);
      }
    }
  }

  double loss = std::numeric_limits<double>::infinity();
  int counter = 0;

  const double step = 1e-7;

  utils::TransformValues translation_and_rotation =
      utils::ExtractTranslationAndRotation(robot_to_field_);
  utils::TransformDecomposition decomposed_robot_to_field;

  while (loss > kacceptable_reprojection_error && counter < 1000000) {
    bool printed = false;
    for (const data_point_t& data_point : data_points) {
      decomposed_robot_to_field = utils::SeparateTranslationAndRotationMatrices(
          translation_and_rotation);
      const Eigen::Matrix<double, 3, 4>& image_to_robot =
          camera_matrices_.at(data_point.name).image_to_robot;

      const Eigen::Vector4d Rx_activation =
          decomposed_robot_to_field.Rx *
          data_point.field_to_tag_corner_homogenous;
      const Eigen::Vector4d Ry_activation =
          decomposed_robot_to_field.Ry * Rx_activation;
      const Eigen::Vector4d Rz_activation =
          decomposed_robot_to_field.Rz * Ry_activation;

      Eigen::Vector3d projection = image_to_robot *
                                   decomposed_robot_to_field.translation *
                                   Rz_activation;
      const double lambda = projection(2);
      projection /= lambda;

      const Eigen::Vector2d projection_error(
          projection.x() - data_point.undistorted_point.x(),
          projection.y() - data_point.undistorted_point.y());
      loss = 0.5 * projection_error.squaredNorm();

      if (counter % 10000 == 0 && !printed) {
        printed = true;
        std::cout << "loss: " << loss << std::endl;
      }

      const Eigen::Vector3d d_projection(
          projection_error.x() / lambda, projection_error.y() / lambda,
          -(projection_error.x() * projection.x() +
            projection_error.y() * projection.y()) /
              lambda);

      Eigen::Vector4d accumulated_gradient =
          image_to_robot.transpose() * d_projection;

      const Eigen::Matrix4d d_translation =
          accumulated_gradient * Rz_activation.transpose();

      accumulated_gradient = decomposed_robot_to_field.translation.transpose() *
                             accumulated_gradient;
      const Eigen::Matrix4d d_Rz =
          accumulated_gradient * Ry_activation.transpose();

      accumulated_gradient =
          decomposed_robot_to_field.Rz.transpose() * accumulated_gradient;
      const Eigen::Matrix4d d_Ry =
          accumulated_gradient * Rx_activation.transpose();

      accumulated_gradient =
          decomposed_robot_to_field.Ry.transpose() * accumulated_gradient;
      const Eigen::Matrix4d d_Rx =
          accumulated_gradient *
          data_point.field_to_tag_corner_homogenous.transpose();

      // clang-format off
      const Eigen::Matrix4d d_Rz_d_rz =
          (Eigen::Matrix4d() << 
        -sin(translation_and_rotation.rz), -cos(translation_and_rotation.rz), 0, 0,
        cos(translation_and_rotation.rz), -sin(translation_and_rotation.rz), 0, 0, 
        0, 0, 0, 0,
        0, 0, 0, 0).finished();
      const Eigen::Matrix4d d_Ry_d_ry =
          (Eigen::Matrix4d() << 
        -sin(translation_and_rotation.ry), 0, cos(translation_and_rotation.ry), 0,
        0, 0, 0, 0,
        -cos(translation_and_rotation.ry), 0, -sin(translation_and_rotation.ry), 0,
        0, 0, 0, 0).finished();
      const Eigen::Matrix4d d_Rx_d_rx =
        (Eigen::Matrix4d() << 
        0, 0, 0, 0, 
        0, -sin(translation_and_rotation.rx), -cos(translation_and_rotation.rx), 0,
        0, cos(translation_and_rotation.rx), -sin(translation_and_rotation.rx), 0,
        0, 0, 0, 0).finished();
      // clang-format on

      translation_and_rotation.rx -=
          step * (d_Rx_d_rx.transpose() * d_Rx).trace();
      translation_and_rotation.ry -=
          step * (d_Ry_d_ry.transpose() * d_Ry).trace();
      translation_and_rotation.rz -=
          step * (d_Rz_d_rz.transpose() * d_Rz).trace();

      translation_and_rotation.x -= step * d_translation(0, 3);
      translation_and_rotation.y -= step * d_translation(1, 3);
      translation_and_rotation.z -= step * d_translation(2, 3);
    }
    counter++;
  }

  Eigen::Matrix4d field_to_robot =
      (decomposed_robot_to_field.translation * decomposed_robot_to_field.Rz *
       decomposed_robot_to_field.Ry * decomposed_robot_to_field.Rx)
          .inverse();

  utils::PrintTransformationMatrix(utils::EigenToCvMat(field_to_robot),
                                   "Field to robot cv");
  utils::ChangeBasis(field_to_robot, utils::CV_TO_WPI);
  utils::PrintTransformationMatrix(utils::EigenToCvMat(field_to_robot),
                                   "Field to robot wpi");

  return {.pose = frc::Pose3d(field_to_robot), .variance = 0, .timestamp = 0};
}

}  // namespace localization
