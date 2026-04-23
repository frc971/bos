#include <gtest/gtest.h>
#include "src/localization/joint_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

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

auto ProjectPoints(const frc::Pose3d& camera_pose, const frc::Pose3d& tag_pose,
                   const Eigen::Matrix3d& camera_matrix) -> Eigen::Vector3d {
  auto feild_to_camera = camera_pose.ToMatrix();
  auto feild_to_tag = tag_pose.ToMatrix();
  auto camera_to_tag = feild_to_camera.inverse() * feild_to_tag * rotate_yaw;

  Eigen::Vector3d projected_points =
      camera_matrix * PI * camera_to_tag *
      localization::kapriltag_corners_eigen_homogenized[0];
  projected_points /= projected_points[0];
  return projected_points;
}

TEST(ForwardTest, Basic) {  // NOLINT
  camera::camera_constant_t camera_constant =
      camera::GetCameraConstants().at("second_bot_right");
  camera_constant.extrinsics_path =
      "/bos/constants/misc/dev_orin_extrinsics.json";
  CHECK(camera_constant.intrinsics_path.has_value());

  std::string image_path = "/bos-logs/log181/left/9.627390.jpg";

  localization::SquareSolver square_solver(camera_constant);
  auto detector = std::make_unique<localization::OpenCVAprilTagDetector>(
      camera_constant.frame_width.value(), camera_constant.frame_height.value(),
      utils::ReadIntrinsics(camera_constant.intrinsics_path.value()));

  // [u, v] = camera_matrix * PI * camera_to_tag * tag_corners
  // feild_to_camera = feild_to_tag * 180_yaw * camera_to_tag.inv
  // feild_to_camera * camera_to_tag = feild_to_tag * 180_yaw
  // camera_to_tag = feild_to_camera.inv * feild_to_tag * 180_yaw

  cv::Mat image = cv::imread(image_path);
  camera::timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  localization::tag_detection_t detection =
      detector->GetTagDetections(timestamped_frame)[0];
  LOG(INFO) << "detected_tag_corners\n"
            << detection.corners[0] << "\n"
            << detection.corners[1] << "\n"
            << detection.corners[2] << "\n"
            << detection.corners[3];

  localization::position_estimate_t square_solver_solution =
      square_solver.EstimatePosition({detection})[0];

  auto camera_matrix = utils::CameraMatrixFromJson<Eigen::Matrix3d>(
      utils::ReadIntrinsics(camera_constant.intrinsics_path.value()));
  LOG(INFO) << "camera_matrix\n" << camera_matrix;

  // auto feild_to_camera = square_solver_solution.pose.ToMatrix();
  // LOG(INFO) << "feild_to_camera\n" << feild_to_camera;
  //
  // auto feild_to_tag =
  //     localization::kapriltag_layout.GetTagPose(detection.tag_id)->ToMatrix();
  // LOG(INFO) << "feild_to_tag\n" << feild_to_tag;
  //
  //
  // auto camera_to_tag = feild_to_camera.inverse() * feild_to_tag * rotate_yaw;
  //
  // Eigen::Vector3d projected_points =
  //     camera_matrix * PI * camera_to_tag *
  //     localization::kapriltag_corners_eigen_homogenized[0];
  // projected_points /= projected_points[0];

  auto projected_points = ProjectPoints(
      square_solver_solution.pose,
      localization::kapriltag_layout.GetTagPose(detection.tag_id).value(),
      camera_matrix);

  LOG(INFO) << "Projected points\n" << projected_points;
}
