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
    : robot_to_field_(Eigen::Matrix4d::Identity()) {
  for (const frc::AprilTag& tag : layout.GetTags()) {
    tag_poses_[tag.ID] = tag.pose.ToMatrix();
  }
  Eigen::Matrix<double, 3, 4> pi = Eigen::Matrix<double, 3, 4>::Zero();
  pi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  for (const camera::Camera& camera_config : camera_constants_) {
    const Eigen::Matrix3d camera_matrix =
        utils::CameraMatrixFromJson<Eigen::Matrix3d>(utils::ReadIntrinsics(
            camera::camera_constants[camera_config].intrinsics_path));
    const Eigen::Matrix<double, 3, 4> image_to_camera = camera_matrix * pi;
    const Eigen::Matrix4d camera_to_robot =
        utils::ExtrinsicsJsonToCameraToRobot(
            utils::ReadExtrinsics(
                camera::camera_constants[camera_config].extrinsics_path))
            .ToMatrix();
    const Eigen::Matrix<double, 3, 4> image_to_robot =
        image_to_camera * camera_to_robot;
    image_to_robot_.insert({camera_config, image_to_robot});
    std::cout << "cam_matrix:\n" << camera_matrix << "\n";
    std::cout << "image_to_camera:\n" << image_to_camera << "\n";
    std::cout << "camera_to_robot:\n" << camera_to_robot << "\n";
    std::cout << "image_to_robot:\n" << image_to_robot << "\n";
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
      for (size_t i = 0; i < detection.corners.size(); i++) {
        const Eigen::Vector2d image_point =
            (Eigen::Vector2d() << detection.corners[i].x,
             detection.corners[i].y)
                .finished();
        // std::cout << "Image point:\n" << image_point << std::endl;
        Eigen::Vector4d field_relative_tag_corner =
            field_to_tag *
            (Eigen::Vector4d()
                 << kapriltag_corners_eigen[kapriltag_corners_eigen.size() - i -
                                            1],
             1)
                .finished();
        // std::cout << "field corner:\n"
        // << field_relative_tag_corner << std::endl;
        detections.push_back(
            Detection{.camera = pair.first,
                      .image_point = image_point,
                      .field_relative_tag_corner = field_relative_tag_corner});
      }
    }
  }
  double error = INFINITY;
  while (error > kacceptable_reprojection_error) {
    for (const Detection& detection : detections) {
      std::cout << detection.image_point << " : "
                << detection.field_relative_tag_corner << std::endl;
    }
    std::exit(0);
    for (const Detection& detection : detections) {
      utils::PrintTransformationMatrix(utils::EigenToCvMat(robot_to_field_),
                                       "robot to field");
      std::cout << "Image point: " << detection.image_point << std::endl;
      std::cout << "Field relative tag corner:\n"
                << detection.field_relative_tag_corner << std::endl;
      Eigen::Vector3d projection = image_to_robot_.at(detection.camera) *
                                   robot_to_field_ *
                                   detection.field_relative_tag_corner;
      std::cout << "Projection:\n" << projection << std::endl;
      const double scale_factor = projection(2);
      projection /= scale_factor;
      std::cout << "Projection scaled:\n" << projection << std::endl;
      std::cout << "Image point:\n" << detection.image_point << std::endl;
      const double MSE_derivative =
          std::hypot(projection[0] - detection.image_point[0],
                     projection[1] - detection.image_point[1]);
      std::cout << "MSE_derivative: " << MSE_derivative << std::endl;
      std::exit(0);
    }
    break;
  }

  return {};
}

}  // namespace localization
