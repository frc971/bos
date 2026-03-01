#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

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

auto JointSolver::Forward(
    const utils::TransformDecomposition& position_estimate,
    Eigen::Vector4d& Rx_activation, Eigen::Vector4d& Ry_activation,
    Eigen::Vector4d& Rz_activation, Eigen::Vector3d& projection,
    const data_point_t& data_point) -> void {
  const Eigen::Matrix<double, 3, 4>& image_to_robot =
      camera_matrices_.at(data_point.source).image_to_robot;

  Rx_activation =
      position_estimate.Rx * data_point.field_to_tag_corner_homogenous;
  Ry_activation = position_estimate.Ry * Rx_activation;
  Rz_activation = position_estimate.Rz * Ry_activation;

  projection = image_to_robot * position_estimate.translation * Rz_activation;
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

  int counter = 0;

  double step_size = 2e-8;
  constexpr double translation_step_multiplier = 100.0;

  utils::TransformValues translation_and_rotation =
      utils::ExtractTranslationAndRotation(robot_to_field_);
  utils::TransformValues step;
  utils::TransformDecomposition decomposed_robot_to_field =
      utils::SeparateTranslationAndRotationMatrices(translation_and_rotation);
  std::vector<Eigen::Vector4d> Rx_activations, Ry_activations, Rz_activations;
  std::vector<Eigen::Vector3d> projection;
  double loss = 0;

  while (loss > kacceptable_reprojection_error && counter < 1000000) {
    bool printed = false;
    utils::TransformValues new_step{};
    for (int i = 0; i < data_points.size(); i++) {

      Forward(decomposed_robot_to_field, Rx_activations[i], Ry_activations[i],
              Rz_activations[i], projection[i], data_points[i]);
      const double lambda = projection[i](2);
      projection[i] /= lambda;

      const Eigen::Vector2d projection_error(
          projection[i].x() - data_points[i].undistorted_point.x(),
          projection[i].y() - data_points[i].undistorted_point.y());
      const Eigen::Vector3d d_projection = Eigen::Vector3d(
          projection_error.x() / lambda, projection_error.y() / lambda,
          -(projection_error.x() * projection[i].x() +
            projection_error.y() * projection[i].y()) /
              lambda);

      const Eigen::Matrix<double, 3, 4>& image_to_robot =
          camera_matrices_.at(data_points[i].source).image_to_robot;
      Eigen::Vector4d accumulated_gradient =
          image_to_robot.transpose() * d_projection;

      const Eigen::Matrix4d d_translation =
          accumulated_gradient * Rz_activations[i].transpose();

      accumulated_gradient = decomposed_robot_to_field.translation.transpose() *
                             accumulated_gradient;
      const Eigen::Matrix4d d_Rz =
          accumulated_gradient * Ry_activations[i].transpose();

      accumulated_gradient =
          decomposed_robot_to_field.Rz.transpose() * accumulated_gradient;
      const Eigen::Matrix4d d_Ry =
          accumulated_gradient * Rx_activations[i].transpose();

      accumulated_gradient =
          decomposed_robot_to_field.Ry.transpose() * accumulated_gradient;
      const Eigen::Matrix4d d_Rx =
          accumulated_gradient *
          data_points[i].field_to_tag_corner_homogenous.transpose();

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

      new_step.rx += -step_size * (d_Rx_d_rx.transpose() * d_Rx).trace();
      new_step.ry += -step_size * (d_Ry_d_ry.transpose() * d_Ry).trace();
      new_step.rz += -step_size * (d_Rz_d_rz.transpose() * d_Rz).trace();

      new_step.x +=
          -step_size * translation_step_multiplier * d_translation(0, 3);
      new_step.y +=
          -step_size * translation_step_multiplier * d_translation(1, 3);
      new_step.z +=
          -step_size * translation_step_multiplier * d_translation(2, 3);
    }
    translation_and_rotation += new_step;
    const utils::TransformDecomposition new_estimate =
        utils::SeparateTranslationAndRotationMatrices(translation_and_rotation);
    double new_loss = 0;
    for (int i = 0; i < data_points.size(); i++) {
      Forward(new_estimate, Rx_activations[i], Ry_activations[i],
              Rz_activations[i], projection[i], data_points[i]);
      const Eigen::Vector2d projection_error(
          projection[i].x() - data_points[i].undistorted_point.x(),
          projection[i].y() - data_points[i].undistorted_point.y());
      new_loss += 0.5 * projection_error.squaredNorm();
    }
    if (new_loss > loss) {
      translation_and_rotation -= new_step;
      step_size /= 2.0;
    } else {
      loss = new_loss;
      step_size *= 2.0;
    }
    if (counter % 10000 == 0 && !printed) {
      printed = true;
      std::cout << "new_loss: " << new_loss << "\told_loss: " << loss
                << "\tstep size: " << step_size << "\tstep: " << new_step
                << std::endl;
    }
    counter++;
  }

  std::cout << "Final step: " << step << std::endl;
  std::cout << "Final loss: " << loss << std::endl;

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
