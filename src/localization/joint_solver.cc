#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/intrinsics_from_json.h"

namespace localization {
using frc::AprilTagFieldLayout;

static const Eigen::MatrixXd PI = []() {
  Eigen::Matrix<double, 3, 4> pi = Eigen::Matrix<double, 3, 4>::Zero();
  pi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  return pi;
}();

JointSolver::JointSolver(const std::string& intrinsics_path,
                         const std::string& extrinsics_path,
                         AprilTagFieldLayout layout, double tag_size)
    : layout_(std::move(layout)),
      camera_matrix_(camera_matrix_from_json<Eigen::MatrixXd>(
          utils::read_intrinsics(intrinsics_path))),
      initial_solver_(utils::read_extrinsics(extrinsics_path)) {}

JointSolver::JointSolver(camera::Camera camera_config,
                         const frc::AprilTagFieldLayout& layout,
                         double tag_size)
    : JointSolver(camera::camera_constants[camera_config].intrinsics_path,
                  camera::camera_constants[camera_config].extrinsics_path,
                  layout, tag_size) {}

auto JointSolver::EstimatePosition(
    const std::vector<tag_detection_t>& detections)
    -> std::vector<position_estimate_t> {
  if (detections.empty()) {
    return {};
  }

  position_estimate_t initial_position_estimate =
      initial_solver_.EstimatePosition(detections)[0];
  initial_position_estimate.pose.ToMatrix();

  // initilaize w
  std::vector<Eigen::MatrixXd> w(detections.size());
  std::array<Eigen::Vector4d, 4> tag_corners;
  for (const auto& detection : detections) {
    auto feild_to_tag = layout_.GetTagPose(detection.tag_id)->ToMatrix();
    w.emplace_back(camera_matrix_ * PI * feild_to_tag);
  }
  Eigen::Matrix4d A = initial_position_estimate.pose.ToMatrix();
  for (int i = 0; i < detections.size(); i++) {
    for (int j = 0; j < 4; j++) {
      Eigen::Vector3d projected_points = w[i] * A * tag_corners[j];

      const double u = projected_points[0];
      const double v = projected_points[1];
      const double lambda = projected_points[2];

      const float u_true = detections[i].corners[j].x;
      const float v_true = detections[i].corners[j].y;

      Eigen::Vector3d dprojected_points(
          (u / lambda - u_true) / lambda, (v / lambda - v_true) / lambda,
          -(u / lambda - u_true) / (lambda * lambda) -
              (v / lambda - v_true) / (lambda * lambda));

      auto dA = w[i] * dprojected_points;

      LOG(INFO) << "projected points " << projected_points;
      LOG(INFO) << "dA " << dA;
    }
  }
  return {};
}

}  // namespace localization
