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
using localization::kapriltag_layout;
using localization::OpenCVAprilTagDetector;
using localization::position_estimate_t;
using localization::SquareSolver;
using localization::tag_detection_t;
using utils::CameraMatrixFromJson;
using utils::ReadIntrinsics;

using transform3d_t = struct Transfrom3d {
  // Translation in meters, rotation in radians
  double t_x;
  double t_y;
  double t_z;
  double r_x;
  double r_y;
  double r_z;

  Transfrom3d(frc::Pose3d pose)
      : t_x(pose.Translation().X().value()),
        t_y(pose.Translation().Y().value()),
        t_z(pose.Translation().Z().value()),
        r_x(pose.Rotation().X().value()),
        r_y(pose.Rotation().Y().value()),
        r_z(pose.Rotation().Z().value()) {}

  Transfrom3d(frc::Transform3d pose)
      : t_x(pose.Translation().X().value()),
        t_y(pose.Translation().Y().value()),
        t_z(pose.Translation().Z().value()),
        r_x(pose.Rotation().X().value()),
        r_y(pose.Rotation().Y().value()),
        r_z(pose.Rotation().Z().value()) {}

  auto ToEigen() -> Eigen::Matrix4d {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();

    // Rotation: ZYX euler convention (common for robot poses)
    Eigen::Matrix3d rot = (Eigen::AngleAxisd(r_z, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(r_y, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(r_x, Eigen::Vector3d::UnitX()))
                              .toRotationMatrix();

    m.block<3, 3>(0, 0) = rot;
    m.block<3, 1>(0, 3) = Eigen::Vector3d{t_x, t_y, t_z};

    return m;
  }
};

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

class ForwardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    camera_constant_ = GetCameraConstants().at("second_bot_right");
    camera_constant_.extrinsics_path =
        "/bos/constants/misc/dev_orin_extrinsics.json";

    square_solver_ = std::make_unique<SquareSolver>(camera_constant_);
    detector_ = std::make_unique<OpenCVAprilTagDetector>(
        camera_constant_.frame_height.value(),
        camera_constant_.frame_height.value(),
        ReadIntrinsics(camera_constant_.intrinsics_path.value()));
    camera_matrix_ = CameraMatrixFromJson<Eigen::Matrix3d>(
        ReadIntrinsics(camera_constant_.intrinsics_path.value()));
  }

  void TearDown() override {}

  camera_constant_t camera_constant_;
  std::unique_ptr<SquareSolver> square_solver_;
  std::unique_ptr<OpenCVAprilTagDetector> detector_;
  Eigen::Matrix3d camera_matrix_;
};

auto ProjectPoints(const frc::Pose3d& camera_pose, const frc::Pose3d& tag_pose,
                   const Eigen::Matrix3d& camera_matrix, int corner_index)
    -> Eigen::Vector3d {
  auto feild_to_camera = camera_pose.ToMatrix();
  auto feild_to_tag = tag_pose.ToMatrix();
  auto camera_to_tag = feild_to_camera.inverse() * feild_to_tag * rotate_yaw;

  Eigen::Vector3d projected_points =
      camera_matrix * PI * camera_to_tag *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_points = projected_points / projected_points[0];
  return normalized_points;
}

auto CalculateDerivative(const frc::Pose3d& camera_pose,
                         const frc::Pose3d& tag_pose,
                         const Eigen::Matrix3d& camera_matrix,
                         const Eigen::Vector3d& image_point, int corner_index)
    -> Eigen::Matrix4d {
  auto feild_to_camera = camera_pose.ToMatrix();
  auto feild_to_tag = tag_pose.ToMatrix();
  auto camera_to_tag = feild_to_camera.inverse() * feild_to_tag * rotate_yaw;

  Eigen::Vector3d projected_point =
      camera_matrix * PI * camera_to_tag *
      localization::kapriltag_corners_eigen_homogenized[corner_index];
  auto normalized_point = projected_point / projected_point[0];

  auto normalized_points_d = normalized_point - image_point;

  // clang-format off
  auto projected_point_d = (Eigen::Vector3d() << 
    -normalized_points_d[1] * projected_point[1] / (projected_point[0] * projected_point[0])
    -normalized_points_d[2] * projected_point[2] / (projected_point[0] * projected_point[0]),
    normalized_points_d[1] / projected_point[0],
    normalized_points_d[2] / projected_point[0]
  ).finished();
  // clang-format on

  auto camera_matrix_xd = camera_matrix.transpose() * projected_point_d;
  auto PI_xd = PI.transpose() * camera_matrix_xd;
  auto camera_to_tag_d =
      PI_xd * localization::kapriltag_corners_eigen_homogenized[corner_index]
                  .transpose();
  return camera_to_tag_d;
}

// TODO: Tolerance is quite high. Find out what causes the loss in precision
void CheckIsEqual(cv::Point2d image_point, Eigen::Vector3d projected_points,
                  double tolerance = 1) {
  EXPECT_NEAR(image_point.x, projected_points[1], tolerance);
  EXPECT_NEAR(image_point.y, projected_points[2], tolerance);
}

TEST_F(ForwardTest, ProjectTest) {  // NOLINT
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
    auto detections = detector_->GetTagDetections(timestamped_frame);
    if (detections.empty()) {
      continue;
    }
    auto detection = detections[0];

    auto square_solver_solution =
        square_solver_->EstimatePosition({detection})[0];

    for (int j = 0; j < 4; j++) {
      auto projected_points =
          ProjectPoints(square_solver_solution.pose,
                        kapriltag_layout.GetTagPose(detection.tag_id).value(),
                        camera_matrix_, j);
      CheckIsEqual(detection.corners[j], projected_points);
    }
  }
}

TEST_F(ForwardTest, TestDerrivative) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/7.047703.jpg");
  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector_->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution =
      square_solver_->EstimatePosition({detection})[0];

  Eigen::Matrix4d camera_to_tag_d = Eigen::Matrix4d::Zero();
  for (int camera_index = 0; camera_index < 4; camera_index++) {
    auto image_point =
        (Eigen::Vector3d() << 1, detection.corners[camera_index].x,
         detection.corners[camera_index].y)
            .finished();

    camera_to_tag_d += CalculateDerivative(
        square_solver_solution.pose,
        kapriltag_layout.GetTagPose(detection.tag_id).value(), camera_matrix_,
        image_point, camera_index);
  }

  LOG(INFO) << "derrivative\n" << camera_to_tag_d;
}

TEST_F(ForwardTest, TestTranfrom3d) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/7.047703.jpg");
  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector_->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution =
      square_solver_->EstimatePosition({detection})[0];

  transform3d_t transform(square_solver_solution.pose);

  LOG(INFO) << "pose.ToMatrix()\n" << square_solver_solution.pose.ToMatrix();
  LOG(INFO) << "transform.ToEigen()\n" << transform.ToEigen();

  EXPECT_TRUE(square_solver_solution.pose.ToMatrix().isApprox(
      transform.ToEigen(), 1e-9));
}
