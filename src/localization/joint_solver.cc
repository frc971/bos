#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

struct Detection {
  const camera::Camera camera;
  Eigen::Vector2d image_point;
  Eigen::Vector4d field_relative_tag_corner;
};

namespace localization {
using frc::AprilTagFieldLayout;

JointSolver::JointSolver(const std::vector<camera::Camera>& camera_constants_,
                         const AprilTagFieldLayout& layout)
    : field_to_robot_(Eigen::Matrix4d::Identity()) {
  for (const frc::AprilTag& tag : layout.GetTags()) {
    tag_poses_[tag.ID] = tag.pose.ToMatrix();
  }
  Eigen::Matrix<double, 3, 4> pi = Eigen::Matrix<double, 3, 4>::Zero();
  pi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  for (const camera::Camera& camera_config : camera_constants_) {
    const Eigen::Matrix3d cam_matrix =
        utils::CameraMatrixFromJson<Eigen::Matrix3d>(utils::ReadIntrinsics(
            camera::camera_constants[camera_config].intrinsics_path));
    const Eigen::Matrix<double, 3, 4> cam2im = cam_matrix * pi;
    const Eigen::Matrix4d robot2cam =
        utils::ExtrinsicsJsonToCameraToRobot(
            utils::ReadExtrinsics(
                camera::camera_constants[camera_config].extrinsics_path))
            .ToMatrix()
            .inverse();
    const Eigen::Matrix<double, 3, 4> robot2im = cam2im * robot2cam;
    robot_to_image_.insert({camera_config, robot2im});
    std::cout << "cam_matrix:\n" << cam_matrix << "\n";
    std::cout << "cam2im:\n" << cam2im << "\n";
    std::cout << "robot2cam:\n" << robot2cam << "\n";
    std::cout << "robot2im:\n" << robot2im << "\n";
  }
}

auto JointSolver::EstimatePosition(
    const std::map<camera::Camera, std::vector<tag_detection_t>>&
        all_cam_detections) -> position_estimate_t {
  if (all_cam_detections.empty()) {
    return {};
  }
  std::vector<Detection> detections;
  for (const auto& pair : all_cam_detections) {
    for (const tag_detection_t& detection : pair.second) {
      if (!tag_poses_[detection.tag_id]) {
        continue;
      }
      Eigen::Matrix4d field_to_tag = tag_poses_[detection.tag_id].value();
      utils::ChangeBasis(field_to_tag, utils::WPI_TO_CV);
      const Eigen::Matrix4d tag_to_field = field_to_tag.inverse();
      for (size_t i = 0; i < detection.corners.size(); i++) {
        Eigen::Vector2d image_point_normalized;
        image_point_normalized << detection.corners[i].x,
            detection.corners[i].y, 0;
        image_point_normalized /= image_point_normalized.maxCoeff();
        std::cout << "Image point:\n" << detection.corners[i] << std::endl;
        std::cout
            << "kapriltag\n"
            << (Eigen::Vector4d() << kapriltag_corners_eigen[i], 1).finished()
            << std::endl;
        std::cout << "Tag_to_field\n" << tag_to_field << std::endl;
        Eigen::Vector4d field_relative_tag_corner =
            tag_to_field *
            (Eigen::Vector4d() << kapriltag_corners_eigen[i], 1).finished();
        std::cout << "Before basis:\n"
                  << field_relative_tag_corner << std::endl;
        utils::ChangeBasis(field_relative_tag_corner, utils::WPI_TO_CV);
        std::cout << "After basis:\n" << field_relative_tag_corner << std::endl;
        detections.push_back(
            Detection{.camera = pair.first,
                      .image_point = image_point_normalized,
                      .field_relative_tag_corner = field_relative_tag_corner});
      }
    }
  }
  double error = INFINITY;
  while (error > kacceptable_reprojection_error) {
    for (const Detection& detection : detections) {
      utils::PrintTransformationMatrix(utils::EigenToCvMat(field_to_robot_),
                                       "Field to robot");
      std::cout << "Full field to robot\n " << field_to_robot_ << std::endl;
      std::cout << "Field relative tag corner:\n"
                << detection.field_relative_tag_corner << std::endl;
      Eigen::Vector3d projection = robot_to_image_.at(detection.camera) *
                                   field_to_robot_ *
                                   detection.field_relative_tag_corner;
      std::cout << "Projection:\n" << projection << std::endl;
      const double scale_factor = projection.cwiseAbs().maxCoeff();
      projection /= scale_factor;
      std::cout << "Projection scaled:\n" << projection << std::endl;
      std::cout << "Image point:\n" << detection.image_point << std::endl;
      const double MSE_derivative =
          std::hypot(projection[0] - detection.image_point[0],
                     projection[1] - detection.image_point[1]);
      std::cout << "MSE_derivative: " << MSE_derivative << std::endl;
    }
    break;
  }

  return {};
}

}  // namespace localization
