#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

constexpr auto sq(double num) -> double {
  return num * num;
}

using frc::AprilTagFieldLayout;

JointSolver::JointSolver(
    const std::vector<camera::camera_constant_t>& camera_constants_)
    : robot_to_field_(Eigen::Matrix4d::Identity()),
      backup_solver_(camera_constants_) {
  // clang-format off
  const Eigen::Matrix4d rotate_yaw_cv = (Eigen::Matrix4d() <<
      -1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, -1, 0,
      0, 0, 0, 1).finished();
  // clang-format on
  for (const frc::AprilTag& tag : kapriltag_layout.GetTags()) {
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
    Eigen::Matrix4d camera_to_robot =
        utils::ExtrinsicsJsonToCameraToRobot(
            utils::ReadExtrinsics(camera_constant.extrinsics_path.value()))
            .ToMatrix();
    utils::ChangeBasis(camera_to_robot, utils::WPI_TO_CV);
    const Eigen::Matrix<double, 3, 4> image_to_robot =
        image_to_camera * camera_to_robot;
    camera_matrices_.push_back(
        {.image_to_robot = image_to_robot,
         .distortion_coefficients =
             utils::DistortionCoefficientsFromJson<cv::Mat>(intrinsics_json),
         .camera_matrix = utils::EigenToCvMat(camera_matrix)});
  }
}

void JointSolver::SetStartPosition(const frc::Pose3d& pose) {
  robot_to_field_ = pose.ToMatrix().inverse();
  utils::ChangeBasis(robot_to_field_, utils::WPI_TO_CV);
}

void JointSolver::ComputeResidual(
    const std::vector<data_point_t>& data_points,
    const Eigen::Matrix4d& robot_to_field, Eigen::VectorXd& residual,
    Eigen::MatrixXd* d_residual_d_twist_jacobian) {
  for (size_t i = 0; i < data_points.size(); i++) {
    const Eigen::Matrix<double, 3, 4>& image_to_robot =
        camera_matrices_[data_points[i].source_index].image_to_robot;

    const Eigen::Vector4d robot_relative_corner =
        robot_to_field * data_points[i].field_to_tag_corner_homogeneous_cv;

    Eigen::Vector3d projection = image_to_robot * robot_relative_corner;
    const double lambda = projection(2);
    projection /= lambda;

    const double x_residual =
        data_points[i].undistorted_point.x() - projection.x();
    const double y_residual =
        data_points[i].undistorted_point.y() - projection.y();
    residual(2 * i) = x_residual;
    residual(2 * i + 1) = y_residual;
    if (d_residual_d_twist_jacobian == nullptr) {
      continue;
    }

    // clang-format off
      const Eigen::Matrix<double, 2, 3> d_residual_i_d_projection_i = 
        (Eigen::MatrixXd(2, 3) <<
          -1/lambda, 0, projection.x()/lambda,
          0, -1/lambda, projection.y()/lambda).finished();
    // clang-format on

    const Eigen::Matrix3d& d_projection_i_d_robot_relative_object_point =
        image_to_robot.block<3, 3>(0, 0);

    Eigen::Matrix<double, 3, 6> d_robot_relative_object_point_d_d_twist =
        Eigen::MatrixXd(3, 6);
    d_robot_relative_object_point_d_d_twist.block<3, 3>(0, 0) =
        -utils::CrossProduct(robot_relative_corner.block<3, 1>(0, 0));
    d_robot_relative_object_point_d_d_twist.block<3, 3>(0, 3) =
        Eigen::Matrix3d::Identity();

    d_residual_d_twist_jacobian->block<2, 6>(2 * i, 0) =
        d_residual_i_d_projection_i *
        d_projection_i_d_robot_relative_object_point *
        d_robot_relative_object_point_d_d_twist;
  }
}

auto JointSolver::EstimatePosition(
    std::vector<std::vector<tag_detection_t>>& detection_batches,
    bool reject_far_tags) -> std::optional<position_estimate_t> {
  if (detection_batches.empty()) {
    return std::nullopt;
  }
  double avg_timestamp = 0;
  std::vector<data_point_t> data_points;
  for (size_t source_index = 0; source_index < detection_batches.size();
       source_index++) {
    const CameraMatrices& camera_mats = camera_matrices_[source_index];
    for (const tag_detection_t& detection : detection_batches[source_index]) {
      avg_timestamp += detection.timestamp;
      std::vector<cv::Point2d> undistorted_corners;
      cv::undistortImagePoints(detection.corners, undistorted_corners,
                               camera_mats.camera_matrix,
                               camera_mats.distortion_coefficients);
      for (size_t corner_index = 0; corner_index < undistorted_corners.size();
           corner_index++) {
        Eigen::Vector2d undistorted_image_point;
        undistorted_image_point << undistorted_corners[corner_index].x,
            undistorted_corners[corner_index].y;
        const data_point_t datapoint = {
            .undistorted_point = undistorted_image_point,
            .source_index = source_index,
            .field_to_tag_corner_homogeneous_cv =
                tag_corners_[detection.tag_id].value()[corner_index]};
        data_points.push_back(datapoint);
      }
    }
  }

  if (data_points.empty()) {
    return std::nullopt;
  }

  avg_timestamp /= data_points.size() / 4;

  Eigen::VectorXd residual(2 * data_points.size());
  Eigen::MatrixXd d_residual_d_twist_jacobian(2 * data_points.size(), 6);
  double current_error;

  for (int epoch = 0; epoch < 1e4 && current_error < kmin_acceptable_error;
       epoch++) {
    ComputeResidual(data_points, robot_to_field_, residual,
                    &d_residual_d_twist_jacobian);
    current_error = residual.cwiseSquare().sum();
    // if (epoch % 10 == 0)
    //   std::cout << "Current error: " << current_error << std::endl;
    const Eigen::Vector<double, 6> b =
        -d_residual_d_twist_jacobian.transpose() * residual;
    const Eigen::Matrix<double, 6, 6> hessian =
        d_residual_d_twist_jacobian.transpose() * d_residual_d_twist_jacobian;
    double lambda = 1;
    Eigen::Matrix<double, 6, 6> hessian_diag =
        hessian.diagonal().asDiagonal().toDenseMatrix();
    Eigen::Matrix4d candidate_robot_to_field;
    while (true) {
      const Eigen::Vector<double, 6> partial_twist =
          (hessian + lambda * hessian_diag).ldlt().solve(b);
      Eigen::Matrix4d twist_expanded = Eigen::Matrix4d::Zero();
      twist_expanded.block<3, 3>(0, 0) =
          utils::CrossProduct(partial_twist.block<3, 1>(0, 0));
      twist_expanded.block<3, 1>(0, 3) = partial_twist.block<3, 1>(3, 0);
      candidate_robot_to_field = twist_expanded.exp() * robot_to_field_;
      ComputeResidual(data_points, candidate_robot_to_field, residual);
      double candidate_error = residual.cwiseSquare().sum();
      if (candidate_error > current_error) {
        lambda *= 2;
        if (lambda > kmaximum_lambda) {
          std::cout << "Failed" << std::endl;
          return backup_solver_.EstimatePosition(detection_batches);
        }
      } else {
        lambda /= 2;
        robot_to_field_ = candidate_robot_to_field;
        break;
      }
    }
  }

  if (current_error > kmax_acceptable_error) {}

  // std::cout << "Robot to field:\n" << robot_to_field_ << std::endl;

  Eigen::Matrix4d field_to_robot = robot_to_field_.inverse();
  field_to_robot(3, 0) = 0;
  field_to_robot(3, 1) = 0;
  field_to_robot(3, 2) = 0;
  field_to_robot(3, 3) = 1;

  // std::cout << "Field to robot cv:\n" << field_to_robot << std::endl;

  // utils::PrintTransformationMatrix(utils::EigenToCvMat(field_to_robot),
  // "Field to robot cv");
  utils::ChangeBasis(field_to_robot, utils::CV_TO_WPI);
  // std::cout << "Field to robot wpi:\n" << field_to_robot << std::endl;
  // utils::PrintTransformationMatrix(utils::EigenToCvMat(field_to_robot),
  //                                  "Field to robot wpi");
  const frc::Pose3d iteratively_solved_pose(field_to_robot);
  if (false/*current_error > kmax_acceptable_error ||
      utils::PoseOffField(iteratively_solved_pose)*/) {
    std::optional<position_estimate_t> backup_solution =
        backup_solver_.EstimatePosition(detection_batches);
    if (backup_solution.has_value()) {
      SetStartPosition(backup_solution->pose);
      std::cout << "Using unambiguous detector for ts: "
                << backup_solution->timestamp << std::endl;
    }
    return backup_solution;
  }

  return std::make_optional<position_estimate_t>(
      {.pose = iteratively_solved_pose,
       .timestamp = avg_timestamp,
       .num_tags = static_cast<int>(data_points.size() / 4),
       .loss = current_error});
}

}  // namespace localization
