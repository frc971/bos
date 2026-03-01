#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

using frc::AprilTagFieldLayout;
// clang-format off
const Eigen::Matrix4d JointSolver::rotate_yaw_cv_ = (Eigen::Matrix4d() <<
      -1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, -1, 0,
      0, 0, 0, 1).finished();
// clang-format on

JointSolver::JointSolver(const std::vector<camera::Camera>& camera_constants_,
                         const AprilTagFieldLayout& layout)
    : robot_to_field_(Eigen::Matrix4d::Identity()) {
  for (const frc::AprilTag& tag : layout.GetTags()) {
    Eigen::Matrix4d field_to_tag = tag.pose.ToMatrix();
    utils::ChangeBasis(field_to_tag, utils::WPI_TO_CV);
    std::array<Eigen::Vector4d, 4> field_relative_corners;
    for (size_t i = 0; i < kapriltag_corners_eigen.size(); i++) {
      field_relative_corners[i] = field_to_tag * rotate_yaw_cv_ *
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

auto JointSolver::Forward(const utils::TransformDecomposition& current_estimate,
                          std::vector<Eigen::Vector4d>& Rx_activations,
                          std::vector<Eigen::Vector4d>& Ry_activations,
                          std::vector<Eigen::Vector4d>& Rz_activations,
                          std::vector<Eigen::Vector3d>& projections,
                          std::vector<Eigen::Vector2d>& projection_errors,
                          const std::vector<data_point_t>& data_points)
    -> double {
  double net_loss = 0;
  for (size_t i = 0; i < data_points.size(); i++) {
    net_loss += Forward(current_estimate, Rx_activations[i], Ry_activations[i],
                        Rz_activations[i], projections[i], projection_errors[i],
                        data_points[i]);
  }
  return net_loss;
}

auto JointSolver::Forward(
    const utils::TransformDecomposition& position_estimate,
    Eigen::Vector4d& Rx_activation, Eigen::Vector4d& Ry_activation,
    Eigen::Vector4d& Rz_activation, Eigen::Vector3d& projection,
    Eigen::Vector2d& projection_error, const data_point_t& data_point)
    -> double {
  const Eigen::Matrix<double, 3, 4>& image_to_robot =
      camera_matrices_.at(data_point.source).image_to_robot;

  Rx_activation =
      position_estimate.Rx * data_point.field_to_tag_corner_homogenous;
  Ry_activation = position_estimate.Ry * Rx_activation;
  Rz_activation = position_estimate.Rz * Ry_activation;

  projection = image_to_robot * position_estimate.translation * Rz_activation;
  projection_error = Eigen::Vector2d(
      projection.x() / projection[2] - data_point.undistorted_point.x(),
      projection.y() / projection[2] - data_point.undistorted_point.y());
  return 0.5 * projection_error.squaredNorm();
}

auto JointSolver::ComputeStep(
    const utils::TransformValues translation_and_rotation,
    const utils::TransformDecomposition& current_estimate,
    const Eigen::Vector4d& Rx_activation, const Eigen::Vector4d& Ry_activation,
    const Eigen::Vector4d& Rz_activation, const Eigen::Vector3d& projection,
    const Eigen::Vector2d& projection_error, const data_point_t& data_point)
    -> utils::TransformValues {
  const double lambda = projection(2);
  const Eigen::Vector3d normalized_projection = projection / lambda;

  const Eigen::Vector3d d_projection = Eigen::Vector3d(
      projection_error.x() / lambda, projection_error.y() / lambda,
      -(projection_error.x() * normalized_projection.x() +
        projection_error.y() * normalized_projection.y()) /
          lambda);

  const Eigen::Matrix<double, 3, 4>& image_to_robot =
      camera_matrices_.at(data_point.source).image_to_robot;
  Eigen::Vector4d accumulated_gradient =
      image_to_robot.transpose() * d_projection;

  const Eigen::Matrix4d d_translation =
      accumulated_gradient * Rz_activation.transpose();

  accumulated_gradient =
      current_estimate.translation.transpose() * accumulated_gradient;
  const Eigen::Matrix4d d_Rz = accumulated_gradient * Ry_activation.transpose();

  accumulated_gradient = current_estimate.Rz.transpose() * accumulated_gradient;
  const Eigen::Matrix4d d_Ry = accumulated_gradient * Rx_activation.transpose();

  accumulated_gradient = current_estimate.Ry.transpose() * accumulated_gradient;
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

  return {.x = -d_translation(0, 3),
          .y = -d_translation(1, 3),
          .z = -d_translation(2, 3),
          .rx = -(d_Rx_d_rx.transpose() * d_Rx).trace() *
                krotation_adjustment_slowdown_scalar,
          .ry = -(d_Ry_d_ry.transpose() * d_Ry).trace() *
                krotation_adjustment_slowdown_scalar,
          .rz = -(d_Rz_d_rz.transpose() * d_Rz).trace() *
                krotation_adjustment_slowdown_scalar};
}

auto JointSolver::ComputeNetStep(
    const utils::TransformValues translation_and_rotation,
    const utils::TransformDecomposition& position_decomposition,
    const std::vector<Eigen::Vector4d>& Rx_activations,
    const std::vector<Eigen::Vector4d>& Ry_activations,
    const std::vector<Eigen::Vector4d>& Rz_activations,
    const std::vector<Eigen::Vector3d>& projections,
    const std::vector<Eigen::Vector2d>& projection_errors,
    const std::vector<data_point_t>& data_points) -> utils::TransformValues {
  utils::TransformValues net_step{};
  for (size_t i = 0; i < data_points.size(); i++) {
    net_step +=
        ComputeStep(translation_and_rotation, position_decomposition,
                    Rx_activations[i], Ry_activations[i], Rz_activations[i],
                    projections[i], projection_errors[i], data_points[i]);
  }
  return net_step;
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

  utils::TransformValues translation_and_rotation =
      utils::ExtractTranslationAndRotation(robot_to_field_);
  utils::TransformDecomposition decomposed_robot_to_field =
      utils::SeparateTranslationAndRotationMatrices(translation_and_rotation);
  std::vector<Eigen::Vector4d> Rx_activations(data_points.size()),
      Ry_activations(data_points.size()), Rz_activations(data_points.size());
  std::vector<Eigen::Vector3d> projections(data_points.size());
  std::vector<Eigen::Vector2d> projection_errors(data_points.size());
  double net_loss =
      Forward(decomposed_robot_to_field, Rx_activations, Ry_activations,
              Rz_activations, projections, projection_errors, data_points);
  utils::TransformValues step =
      ComputeNetStep(translation_and_rotation, decomposed_robot_to_field,
                     Rx_activations, Ry_activations, Rz_activations,
                     projections, projection_errors, data_points);
  std::cout << "Initial loss: " << net_loss
            << " initial step: " << step * step_size << std::endl;

  for (size_t i = 0; i < 1e5 && net_loss > kacceptable_reprojection_error;
       i++) {
    translation_and_rotation += step * step_size;
    double new_loss;
    do {
      decomposed_robot_to_field = utils::SeparateTranslationAndRotationMatrices(
          translation_and_rotation);
      new_loss =
          Forward(decomposed_robot_to_field, Rx_activations, Ry_activations,
                  Rz_activations, projections, projection_errors, data_points);
      if (new_loss > net_loss) {
        step_size /= 2.0;
        translation_and_rotation -= step * step_size;
        // std::cout << "Stepping down learning rate, current estimation: "
        //           << translation_and_rotation << std::endl;
        continue;
      } else {
        step_size *= 2.0;
        // std::cout << "Stepping up learning rate" << std::endl;
        break;
      }
    } while (true);
    net_loss =
        Forward(decomposed_robot_to_field, Rx_activations, Ry_activations,
                Rz_activations, projections, projection_errors, data_points);
    step = ComputeNetStep(translation_and_rotation, decomposed_robot_to_field,
                          Rx_activations, Ry_activations, Rz_activations,
                          projections, projection_errors, data_points);
    if (i % 1000 == 0) {
      std::cout << "Step size: " << step_size << " new net loss: " << net_loss
                << " new step: " << step * step_size << std::endl;
    }
  }

  std::cout << "Final step: " << step << std::endl;
  std::cout << "Final loss: " << net_loss << std::endl;

  Eigen::Matrix4d field_to_robot =
      (decomposed_robot_to_field.translation * decomposed_robot_to_field.Rz *
       decomposed_robot_to_field.Ry * decomposed_robot_to_field.Rx)
          .inverse();

  utils::PrintTransformationMatrix(utils::EigenToCvMat(field_to_robot),
                                   "Field to robot cv");
  utils::ChangeBasis(field_to_robot, utils::CV_TO_WPI);
  utils::PrintTransformationMatrix(utils::EigenToCvMat(field_to_robot),
                                   "Field to robot wpi");
  std::cout << "Field to robot:\n" << field_to_robot << std::endl;
  // Graham-Schmidt normalization because wpilib is very picky and floating-point precision sometimes makes the matrix not orthogonal
  // https://www.khanacademy.org/math/linear-algebra/alternate-bases/orthonormal-basis/v/linear-algebra-the-gram-schmidt-process
  Eigen::Matrix3d R = field_to_robot.block<3, 3>(0, 0);
  Eigen::Vector3d x = R.col(0).normalized();
  Eigen::Vector3d y = (R.col(1) - x * x.dot(R.col(1))).normalized();
  Eigen::Vector3d z = x.cross(y);

  Eigen::Matrix3d R_normalized;
  R_normalized.col(0) = x;
  R_normalized.col(1) = y;
  R_normalized.col(2) = z;

  return {
      .pose = frc::Pose3d(frc::Translation3d(field_to_robot.block<3, 1>(0, 3)),
                          frc::Rotation3d(R_normalized)),
      .variance = 0,
      .timestamp = 0};
}

}  // namespace localization
