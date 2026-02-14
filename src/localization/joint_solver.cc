#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

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
      camera_matrix_(utils::camera_matrix_from_json<Eigen::MatrixXd>(
          utils::read_intrinsics(intrinsics_path))) {}

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
  return {};
}

auto JointSolver::EstimatePosition(
    const std::vector<std::vector<tag_detection_t>>& detections)
    -> std::vector<position_estimate_t> {
  assert(detections.size() == 1);
  return EstimatePosition(detections[0]);
}

}  // namespace localization
