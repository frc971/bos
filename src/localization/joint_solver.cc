#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

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

JointSolver::JointSolver(
    const std::vector<camera::camera_constant_t>& camera_constants,
    const AprilTagFieldLayout& layout) {
  for (int i = 0; i < camera_constants.size(); i++) {
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
    const frc::Pose3d& starting_pose) -> position_estimate_t {
  return {};
}

}  // namespace localization
