#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

#define IMAGE_STRIDE 4
#define LOG_PATH "/bos-logs/log181/right"
#define LR 0.05
#define EPOCHS 100
#define NORMALIZATION 500
#define BETA 0.3

using frc::AprilTagFieldLayout;
using utils::CameraMatrixFromJson;
using utils::ExtrinsicsJsonToCameraToRobot;
using utils::ReadExtrinsics;
using utils::ReadIntrinsics;

// clang-format off
const Eigen::Matrix<double, 3, 4> PI =
    (Eigen::Matrix<double, 3, 4>() << 
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0).finished();

const Eigen::Matrix<double, 4, 4> rotate_yaw =
    (Eigen::Matrix<double, 4, 4>() << 
  -1, 0, 0, 0,
  0, -1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1).finished();
// clang-format on

void NormalizeCameraMatrix(Eigen::Matrix3d& camera_matrix) {
  camera_matrix = camera_matrix / 1000;
  camera_matrix(0, 0) = 1;
}

auto JointSolver::CalculateDerivative(const Eigen::Matrix4d& robot_to_feild,
                                      const Eigen::Matrix4d& feild_to_tag,
                                      const Eigen::Matrix4d& camera_to_robot,
                                      const Eigen::Matrix3d& camera_matrix,
                                      const Eigen::Vector3d& image_point,
                                      int corner_index) -> Eigen::Matrix4d {
  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_robot * robot_to_feild * feild_to_tag *
      rotate_yaw *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_point = projected_point / projected_point[0];

  auto normalized_points_d = normalized_point - image_point;

  // clang-format off
  auto projected_point_d = (Eigen::Vector3d() << 
    (-normalized_points_d[1] * projected_point[1]) / (projected_point[0] * projected_point[0]) + 
    (-normalized_points_d[2] * projected_point[1]) / (projected_point[0] * projected_point[0]),
    normalized_points_d[1] / projected_point[0],
    normalized_points_d[2] / projected_point[0]
  ).finished();
  // clang-format on

  auto camera_matrix_xd = camera_matrix.transpose() * projected_point_d;

  auto PI_xd = PI.transpose() * camera_matrix_xd;

  auto camera_to_robot_xd = camera_to_robot.transpose() * PI_xd;

  auto robot_to_feild_d =
      camera_to_robot_xd *

      (feild_to_tag * rotate_yaw *
       localization::kapriltag_corners_eigen_homogenized[corner_index])
          .transpose();

  return robot_to_feild_d;
}

auto JointSolver::CalculateLoss(const Eigen::Matrix4d& robot_to_feild,
                                const Eigen::Matrix4d& feild_to_tag,
                                const Eigen::Matrix4d& camera_to_robot,
                                const Eigen::Matrix3d& camera_matrix,
                                const Eigen::Vector3d& image_point,
                                int corner_index) -> double {

  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_robot * robot_to_feild * feild_to_tag *
      rotate_yaw *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_point = projected_point / projected_point[0];

  auto normalized_points_d = normalized_point - image_point;

  // LOG(INFO) << "\n" << normalized_point << "\n" << image_point;
  // LOG(INFO) << normalized_points_d.array().square().sum();

  return normalized_points_d.array().square().sum();
}

JointSolver::JointSolver(
    const std::vector<camera::camera_constant_t>& camera_constants,
    const AprilTagFieldLayout& layout) {
  for (size_t i = 0; i < camera_constants.size(); i++) {
    camera_name_to_index[camera_constants[i].name] = i;

    normalized_camera_matrix_.emplace_back(
        CameraMatrixFromJson<Eigen::Matrix3d>(
            ReadIntrinsics(camera_constants[i].intrinsics_path.value())));
    NormalizeCameraMatrix(normalized_camera_matrix_[i]);

    camera_to_robot_.emplace_back(
        ExtrinsicsJsonToCameraToRobot(
            ReadExtrinsics(camera_constants[i].extrinsics_path.value()))
            .ToMatrix());
  }
}

auto JointSolver::EstimatePosition(
    const std::map<std::string, std::vector<tag_detection_t>>&
        camera_detections,
    std::optional<frc::Pose3d> initial_pose) -> position_estimate_t {
  std::vector<int> tag_ids;
  double average_timestamp = 0;
  int total_detections = 0;
  for (const auto& [_, tag_detections] : camera_detections) {
    for (const auto& tag_detection : tag_detections) {
      tag_ids.push_back(tag_detection.tag_id);
      average_timestamp += tag_detection.timestamp;
      total_detections++;
    }
  }
  average_timestamp /= total_detections;

  tape_.clearAll();
  DifferntiableTransform3d robot_to_feild(
      frc::Pose3d(initial_pose.value_or(position_receiver_.Get()))
          .ToMatrix()
          .inverse());

  JointSolver::transform3d_derrivative_t velocity;
  double loss = 0;
  for (int epoch = 0; epoch < EPOCHS; epoch++) {
    loss = 0;
    JointSolver::transform3d_derrivative_t derrivative;
    for (auto const& [camera_name, tag_detections] : camera_detections) {
      int camera_index = camera_name_to_index[camera_name];
      const auto& camera_to_robot = camera_to_robot_[camera_index];
      const auto& camera_matrix = normalized_camera_matrix_[camera_index];

      for (const auto& detection : tag_detections) {
        auto feild_to_tag =
            kapriltag_layout.GetTagPose(detection.tag_id).value().ToMatrix();

        for (int corner_index = 0; corner_index < 4; corner_index++) {
          robot_to_feild.CalculateMatrix();
          robot_to_feild.RegisterInputs(tape_);
          const auto robot_to_feild_eigen = robot_to_feild.ToEigen();
          auto image_point = (Eigen::Vector3d() << 1,
                              detection.corners[corner_index].x / NORMALIZATION,
                              detection.corners[corner_index].y / NORMALIZATION)
                                 .finished();

          auto robot_to_feild_d = CalculateDerivative(
              robot_to_feild_eigen, feild_to_tag, camera_to_robot,
              camera_matrix, image_point, corner_index);

          robot_to_feild.RegisterOutputs(tape_);
          derrivative = derrivative +
                        robot_to_feild.BackPropagate(robot_to_feild_d, tape_);

          loss +=
              CalculateLoss(robot_to_feild_eigen, feild_to_tag, camera_to_robot,
                            camera_matrix, image_point, corner_index);

          tape_.newRecording();
        }
      }
    }
    velocity = (velocity * BETA) + derrivative;
    robot_to_feild.Update(velocity, LR);
  }
  robot_to_feild.CalculateMatrix();

  Eigen::Matrix4d feild_to_robot = robot_to_feild.ToEigen().inverse();
  feild_to_robot(3, 0) = 0;
  feild_to_robot(3, 1) = 0;
  feild_to_robot(3, 2) = 0;
  feild_to_robot(3, 3) = 1;

  return position_estimate_t{
      .tag_ids = tag_ids,
      .rejected_tag_ids = {},
      .pose = frc::Pose3d(feild_to_robot),
      .variance = 1,
      .timestamp = average_timestamp,
      .num_tags = total_detections,
      .avg_tag_dist = -1,
      .latency = -1,
      .invalid = false,
      .loss = 0,
  };
}

}  // namespace localization
