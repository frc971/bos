#include <gtest/gtest.h>
#include <filesystem>
#include "src/localization/joint_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

#define IMAGE_STRIDE 4
#define LOG_PATH "/bos-logs/log181/left"

namespace fs = std::filesystem;

using camera::camera_constant_t;
using camera::GetCameraConstants;
using camera::timestamped_frame_t;
using localization::OpenCVAprilTagDetector;
using localization::position_estimate_t;
using localization::SquareSolver;
using localization::tag_detection_t;
using utils::CameraMatrixFromJson;
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

auto ProjectPoints(const frc::Pose3d& camera_pose, const frc::Pose3d& tag_pose,
                   const Eigen::Matrix3d& camera_matrix, int corner_index)
    -> Eigen::Vector3d {
  auto feild_to_camera = camera_pose.ToMatrix();
  auto feild_to_tag = tag_pose.ToMatrix();
  auto camera_to_tag = feild_to_camera.inverse() * feild_to_tag * rotate_yaw;

  Eigen::Vector3d projected_points =
      camera_matrix * PI * camera_to_tag *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  projected_points /= projected_points[0];
  return projected_points;
}

// TODO: Tolerance is quite high. Find out what causes the loss in precision
void CheckIsEqual(cv::Point2d image_point, Eigen::Vector3d projected_points,
                  double tolerance = 1) {
  EXPECT_NEAR(image_point.x, projected_points[1], tolerance);
  EXPECT_NEAR(image_point.y, projected_points[2], tolerance);
}

TEST(ForwardTest, Basic) {  // NOLINT
  camera_constant_t camera_constant =
      GetCameraConstants().at("second_bot_right");
  camera_constant.extrinsics_path =
      "/bos/constants/misc/dev_orin_extrinsics.json";
  CHECK(camera_constant.intrinsics_path.has_value());

  SquareSolver square_solver(camera_constant);
  const auto detector = std::make_unique<OpenCVAprilTagDetector>(
      camera_constant.frame_width.value(), camera_constant.frame_height.value(),
      ReadIntrinsics(camera_constant.intrinsics_path.value()));

  const auto camera_matrix = CameraMatrixFromJson<Eigen::Matrix3d>(
      utils::ReadIntrinsics(camera_constant.intrinsics_path.value()));

  // [u, v] = camera_matrix * PI * camera_to_tag * tag_corners
  // feild_to_camera = feild_to_tag * 180_yaw * camera_to_tag.inv
  // feild_to_camera * camera_to_tag = feild_to_tag * 180_yaw
  // camera_to_tag = feild_to_camera.inv * feild_to_tag * 180_yaw

  std::vector<fs::path> image_paths;
  for (const auto& file : fs::directory_iterator(LOG_PATH)) {
    image_paths.push_back(file.path());
  }
  std::sort(image_paths.begin(), image_paths.end());

  for (size_t i = 0; i < image_paths.size(); i += IMAGE_STRIDE) {
    cv::Mat image = cv::imread(image_paths[i]);
    timestamped_frame_t timestamped_frame{
        .frame = std::move(image), .timestamp = 0, .invalid = false};
    auto detections = detector->GetTagDetections(timestamped_frame);
    if (detections.empty()) {
      continue;
    }
    auto detection = detections[0];

    auto square_solver_solution =
        square_solver.EstimatePosition({detection})[0];

    for (int j = 0; j < 4; j++) {
      auto projected_points = ProjectPoints(
          square_solver_solution.pose,
          localization::kapriltag_layout.GetTagPose(detection.tag_id).value(),
          camera_matrix, j);
      CheckIsEqual(detection.corners[j], projected_points);
    }
  }
}
